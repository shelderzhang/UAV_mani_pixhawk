#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>
#include "../BlockLocalPositionEstimator.hpp"

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize

static const uint32_t 		REQ_MOCAP_INIT_COUNT = 2;
static const uint32_t 		MOCAP_TIMEOUT =     500000;	// 0.5 s

void BlockLocalPositionEstimator::mocapInit()
{
	// measure
	Vector<float, n_y_mocap> y;

	if (mocapMeasure(y) != OK) {
		_mocapStats.reset();
		return;
	}
	_mocapOrigin = Vector3f(.0f, .0f, .0f);
#ifdef ONLY_MOCAP
	mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap position: "
				     "%5.2f, %5.2f, %5.2f",
					 double(y(0)),double(y(1)),double(y(2)));
	_mocapInitialized = true;
	_mocapFault = FAULT_NONE;

	if (!_altOriginInitialized) {
		_altOriginInitialized = true;
		_altOrigin = _mocapOrigin(2);
	}

#else
	// if finished
	if (_mocapStats.getCount() > REQ_MOCAP_INIT_COUNT) {
		_mocapOrigin = _mocapStats.getMean();
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap position init: "
					     "%5.2f, %5.2f, %5.2f m std %5.2f, %5.2f, %5.2f m",
					     double(_mocapStats.getMean()(0)),
					     double(_mocapStats.getMean()(1)),
					     double(_mocapStats.getMean()(2)),
					     double(_mocapStats.getStdDev()(0)),
					     double(_mocapStats.getStdDev()(1)),
					     double(_mocapStats.getStdDev()(2)));
		_mocapInitialized = true;
		_mocapFault = FAULT_NONE;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = _mocapOrigin(2);
		}
	}
#endif
}


int BlockLocalPositionEstimator::mocapMeasure(Vector<float, n_y_mocap> &y)
{
	y.setZero();
	y(Y_mocap_x) = _sub_mocap.get().x;
	y(Y_mocap_y) = _sub_mocap.get().y;
	y(Y_mocap_z) = _sub_mocap.get().z;
#if USING_MOCAP_VEL == 2
	y(Y_mocap_vx) = _sub_mocap.get().vx;
	y(Y_mocap_vy) = _sub_mocap.get().vy;
	y(Y_mocap_vz) = _sub_mocap.get().vz;
#endif
	_mocapStats.update(y);
	_time_last_mocap = _sub_mocap.get().timestamp;
	return OK;
}

void BlockLocalPositionEstimator::mocapCorrect()
{
	// measure
	Vector<float, n_y_mocap> y;

	if (mocapMeasure(y) != OK) { return; }

	// make measurement relative to origin
	y -= _mocapOrigin;

	// mocap measurement matrix, measures position
	Matrix<float, n_y_mocap, n_x> C;
	C.setZero();
	C(Y_mocap_x, X_x) = 1;
	C(Y_mocap_y, X_y) = 1;
	C(Y_mocap_z, X_z) = 1;
#if USING_MOCAP_VEL == 2
	C(Y_mocap_vx, X_vx) = 1;
	C(Y_mocap_vy, X_vy) = 1;
	C(Y_mocap_vz, X_vz) = 1;
#endif

	// noise matrix
	Matrix<float, n_y_mocap, n_y_mocap> R;
	R.setZero();
	float mocap_p_var = _mocap_p_stddev.get()* \
			    _mocap_p_stddev.get();
	R(Y_mocap_x, Y_mocap_x) = mocap_p_var;
	R(Y_mocap_y, Y_mocap_y) = mocap_p_var;
	R(Y_mocap_z, Y_mocap_z) = mocap_p_var;
#if USING_MOCAP_VEL == 2
	float mocap_v_var = _mocap_v_stddev.get()* \
			    _mocap_v_stddev.get();
	R(Y_mocap_vx, Y_mocap_vx) = mocap_v_var;
	R(Y_mocap_vy, Y_mocap_vy) = mocap_v_var;
	R(Y_mocap_vz, Y_mocap_vz) = mocap_v_var;
#endif

	// residual

	Vector<float, n_y_mocap> r = y - C * _x;

	for (int i = 0; i < 6; i ++) {
		_pub_innov.get().vel_pos_innov[i] = r(i);
		_pub_innov.get().vel_pos_innov_var[i] = R(i, i);
	}
	Matrix<float, n_y_mocap, n_y_mocap> S_I = inv<float, n_y_mocap>((C * _P * C.transpose()) + R);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_mocap]) {
		if (_mocapFault < FAULT_MINOR) {
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap fault, beta %5.2f", double(beta));
			_mocapFault = FAULT_MINOR;
		}

	} else if (_mocapFault) {
		_mocapFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap OK");
	}

	// kalman filter correction if no fault
	if (_mocapFault < fault_lvl_disable) {
		Matrix<float, n_x, n_y_mocap> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		correctionLogic(dx);
		_x += dx;
		_P -= K * C * _P;
#if USING_MOCAP_VEL == 1
		_x(X_vx) = _sub_mocap.get().vx;
		_x(X_vy) = _sub_mocap.get().vy;
		_x(X_vz) = _sub_mocap.get().vz;
		// printf("n_y_mocap is : %d\n",n_y_mocap);
#endif
	}
}

void BlockLocalPositionEstimator::mocapCheckTimeout()
{
	if (_timeStamp - _time_last_mocap > MOCAP_TIMEOUT) {
		if (_mocapInitialized) {
			_mocapInitialized = false;
			_mocapStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap timeout: %8.4f",(double)((_timeStamp - _time_last_mocap)*1e-6));
		}
	}
}
