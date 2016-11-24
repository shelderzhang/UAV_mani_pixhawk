#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>
#include "../BlockLocalPositionEstimator.hpp"

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
#ifndef ONLY_MOCAP
static const uint32_t 		REQ_MOCAP_INIT_COUNT = 10;
#endif
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

	y(Y_mocap_vx) = _sub_mocap.get().vx;
	y(Y_mocap_vy) = _sub_mocap.get().vy;
	y(Y_mocap_vz) = _sub_mocap.get().vz;

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
	for (int i = 1; i<3; i++) {
		/*minus macop position origin error -bdai<21 Nov 2016>*/
		y(i) -= _mocapOrigin(i);
	}

	// mocap measurement matrix, measures position
	Matrix<float, n_y_mocap, n_x> C;
	C.setZero();
	C(Y_mocap_x, X_x) = 1;
	C(Y_mocap_y, X_y) = 1;
	C(Y_mocap_z, X_z) = 1;
	C(Y_mocap_vx, X_vx) = 1;
	C(Y_mocap_vy, X_vy) = 1;
	C(Y_mocap_vz, X_vz) = 1;

	// noise matrix
	Matrix<float, n_y_mocap, n_y_mocap> R;
	R.setZero();
	float mocap_p_var = _mocap_p_stddev.get()* \
			    _mocap_p_stddev.get();
	R(Y_mocap_x, Y_mocap_x) = mocap_p_var;
	R(Y_mocap_y, Y_mocap_y) = mocap_p_var;
	R(Y_mocap_z, Y_mocap_z) = mocap_p_var;

	float mocap_v_var = _mocap_v_stddev.get()*_mocap_v_stddev.get();
	R(Y_mocap_vx, Y_mocap_vx) = mocap_v_var;
	R(Y_mocap_vy, Y_mocap_vy) = mocap_v_var;
	R(Y_mocap_vz, Y_mocap_vz) = mocap_v_var;

	// get delayed x and P
	float t_delay = 0;
	int i_hist = 0;

	for (i_hist = 1; i_hist < MOCAP_HIST_LEN; i_hist++) {
		t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

		if (t_delay > _mocap_p_delay.get()) {
			break;
		}
	}

	// if you are 3 steps past the delay you wanted, this
	// data is probably too old to use
	if (t_delay > MOCAP_DELAY_MAX) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap position delayed data too old: %8.4f", double(t_delay));
		return;
	}

	Vector<float, n_x> x0 = _xDelay.get(i_hist);

	for (i_hist = 1; i_hist < MOCAP_VEL_HIST_LEN; i_hist++) {
		t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

		if (t_delay > _mocap_v_delay.get()) {
			break;
		}
	}

	// if you are 3 steps past the delay you wanted, this
	// data is probably too old to use
	if (t_delay > MOCAP_VEL_DELAY_MAX) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap velocity delayed data too old: %8.4f", double(t_delay));
		return;
	}
	Vector<float, n_x> x1 = _xDelay.get(i_hist);

	/*only delay velocity observation -bdai<21 Nov 2016>*/
	for (int i = 0; i < 3; i++) {
		x0(3 + i) = x1(3 + i);
	}

	// residual
	Matrix<float, n_y_mocap, n_y_mocap> S_I = inv<float, n_y_mocap>((C * _P * C.transpose()) + R);
	Vector<float, n_y_gps> r = y - C * x0;

	for (int i = 0; i < 6; i ++) {
		_pub_innov.get().vel_pos_innov[i] = r(i);
		_pub_innov.get().vel_pos_innov_var[i] = R(i, i);
	}

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_mocap]) {
		if (_mocapFault < FAULT_MINOR) {
			//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap fault, beta %5.2f", double(beta));
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
