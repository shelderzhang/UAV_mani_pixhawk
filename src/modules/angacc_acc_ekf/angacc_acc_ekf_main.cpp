#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <float.h>
#include <errno.h>
#include <limits.h>
#include <math.h>

#include <controllib/blocks.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
// uORB Suscriptions
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_gyro.h>
// uORB Publications
#include <uORB/topics/angacc_acc.h>
#include <uORB/topics/att_pos_mocap.h>

//#define USING_ACC
//#define USING_ANGACC

using namespace matrix;
using namespace control;

extern orb_advert_t mavlink_log_pub;

extern "C" __EXPORT int angacc_acc_ekf_main(int argc, char *argv[]);

class AngaccAccEKF : public control::SuperBlock {
    // dynamics:
    //
    //	x(+) = A * x(-)
    //	y = C*x
    //
    // kalman filter
    //
    //	E[xx'] = Q
    //	E[yy'] = R
    //
    //	prediction
    //		x(+|-) = A*x(-|-)
    //		P(+|-) = A*P(-|-)*A' + Q
    //
    //	correction
    //      K = P(+|-) * C' * inav(C * P(+|-) * C' + R)
    //		x(+|+) =  x(+|-) + K * (y - C * x(+|-) )
    //      P(+|+) = (I - K * C) * P(+|-)
    //
public:
    enum {X_rx = 0, X_ry, X_rz, X_aax, X_aay, X_aaz, n_rx};
#ifdef USING_ANGACC
    enum {Y_rx = 0, Y_ry, Y_rz, Y_aax, Y_aay, Y_aaz, n_ry};
#else
    enum {Y_rx = 0, Y_ry, Y_rz, n_ry};
#endif

    enum {X_vx = 0, X_vy, X_vz, X_ax, X_ay, X_az, n_ax};
#ifdef USING_ACC
    enum {Y_vx = 0, Y_vy, Y_vz, Y_ax, Y_ay, Y_az, n_ay};
#else
    enum {Y_vx = 0, Y_vy, Y_vz, n_ay};
#endif
    
    AngaccAccEKF();
    ~AngaccAccEKF();

    int start();

private:
    bool _task_should_exit;
    int _control_task;

    int _sub_sensor_gyro;
    int _sub_ctrl_state;
    int _sub_param_update;
    int _sub_local_pos;

    orb_advert_t _pub_angacc_acc;
    orb_advert_t _pub_fake_mocap;       //for monitoring angacc and acc estimator

    struct sensor_gyro_s                _sensor_gyro;
    struct control_state_s              _ctrl_state;
    struct vehicle_local_position_s     _local_pos;
    struct angacc_acc_s                 _angacc_acc;
    struct att_pos_mocap_s              _fake_mocap;

    struct {
        param_t pn_ang_acc;
        param_t pn_ang_rate;
        param_t pn_acc;
        param_t pn_vel;
        param_t std_ang_rate;
        param_t std_ang_acc;
        param_t std_acc;
        param_t std_vel;
    }   _params_handles;

    struct {
        float pn_ang_acc;
        float pn_ang_rate;
        float pn_acc;
        float pn_vel;
        float std_ang_rate;
        float std_ang_acc;
        float std_acc;
        float std_vel;
    }   _params;

    uint64_t _pre_ang_timeStamp;
    uint64_t _pre_acc_timeStamp;
    float _dt_ang;
    float _dt_acc;

    bool _vel_updated;
    px4_pollfd_struct_t _polls;

    Vector<float, n_rx> _x_ang;		 // state Vector
	Matrix<float, n_rx, n_rx>  _P_Ang; // state covariance matrix
    Matrix<float, n_ry, n_rx>  _C_Ang;

    Matrix<float, n_rx, n_rx>  _A_Ang; // dynamics matrix
	Matrix<float, n_rx, n_rx>  _Q_Ang; // process noise covariance
	Matrix<float, n_ry, n_ry>  _R_Ang; // measurement covariance

    Vector<float, n_ax> _x_acc;		 // state Vector
	Matrix<float, n_ax, n_ax>  _P_Acc; // state covariance matrix
    Matrix<float, n_ay, n_ax>  _C_Acc;

    Matrix<float, n_ax, n_ax>  _A_Acc; // dynamics matrix
	Matrix<float, n_ax, n_ax>  _Q_Acc; // process noise covariance
    Matrix<float, n_ay, n_ay>  _R_Acc; // measurement covariance

    BlockLowPassVector<float, n_rx> _x_angaccLowPass;
    BlockLowPassVector<float, n_ax> _x_accLowPass;
    /**
	 * Update our local parameter cache.
	 */
	void	parameters_update();

    void initSS();

    void updateSSParams();
    void predict();
    void correct();

    /**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace angacc_acc_ekf
{
AngaccAccEKF	*est_instant;
}

AngaccAccEKF::AngaccAccEKF() :
    SuperBlock(NULL,"AAE"),
    _task_should_exit(false),
    _control_task(-1),
    _sub_sensor_gyro(-1),
    _sub_ctrl_state(-1),
    _sub_param_update(-1),
    _sub_local_pos(-1),
    _pub_angacc_acc(nullptr),
    _pub_fake_mocap(nullptr),
	_pre_ang_timeStamp(hrt_absolute_time()),
    _pre_acc_timeStamp(hrt_absolute_time()),
    _dt_ang(0.0f),
    _dt_acc(0.0f),
    _vel_updated(false),
	_x_angaccLowPass(this, "X_ANG_LP"),
	_x_accLowPass(this, "X_ACC_LP")
{
    memset(&_sensor_gyro, 0, sizeof(_sensor_gyro));
    memset(&_ctrl_state, 0, sizeof(_ctrl_state));
    memset(&_local_pos, 0, sizeof(_local_pos));
    memset(&_angacc_acc, 0, sizeof(_angacc_acc));
    memset(&_fake_mocap, 0, sizeof(_fake_mocap));

    memset(&_params, 0, sizeof(_params));

    _params_handles.pn_ang_acc =    param_find("AAE_PN_ANGACC");
    _params_handles.pn_ang_rate =   param_find("AAE_PN_ANGRATE");
    _params_handles.pn_acc =        param_find("AAE_PN_ACC");
    _params_handles.pn_vel =        param_find("AAE_PN_VEL");
    _params_handles.std_ang_rate =  param_find("AAE_STD_ANGRAT");
    _params_handles.std_ang_acc =	param_find("AAE_STD_ANGACC");
    _params_handles.std_acc =       param_find("AAE_STD_ACC");
    _params_handles.std_vel =       param_find("AAE_STD_VEL");
    
    parameters_update();
}

AngaccAccEKF::~AngaccAccEKF()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	angacc_acc_ekf::est_instant = nullptr;
}


void AngaccAccEKF::parameters_update()
{   
    param_get(_params_handles.pn_ang_acc, &_params.pn_ang_acc);
    param_get(_params_handles.pn_ang_rate, &_params.pn_ang_rate);
    param_get(_params_handles.pn_acc, &_params.pn_acc);
    param_get(_params_handles.pn_vel, &_params.pn_vel);
    param_get(_params_handles.std_ang_rate, &_params.std_ang_rate);
    param_get(_params_handles.std_ang_acc, &_params.std_ang_acc);
    param_get(_params_handles.std_acc, &_params.std_acc);
    param_get(_params_handles.std_vel, &_params.std_vel);
}

void AngaccAccEKF::initSS()
{
    _x_ang.setZero();
    _x_acc.setZero();

    _P_Ang.setZero();
    float p_ang_init = 1.0e-2f;
    for (int i=0; i<n_rx; i++) {
        _P_Ang(i, i) = p_ang_init;
    }

    _P_Acc.setZero();
    float p_acc_init = 1.0e-2f;
    for (int i=0; i<n_ax; i++) {
        _P_Acc(i, i) = p_acc_init;
    }

    _A_Ang.setIdentity();
    float a_ang_init = 4.0e-3f;
    _A_Ang(X_rx, X_aax) = a_ang_init;  //250 Hz
    _A_Ang(X_ry, X_aay) = a_ang_init;
    _A_Ang(X_rz, X_aaz) = a_ang_init;

    _A_Acc.setIdentity();
    float a_acc_init = 8.0e-3;
    _A_Acc(X_vx, X_ax) = a_acc_init;  //250 Hz
    _A_Acc(X_vy, X_ay) = a_acc_init;
    _A_Acc(X_vz, X_az) = a_acc_init;
    
    _C_Ang.setZero();
    for (int i=0; i< n_ry; i++) {
        _C_Ang(i, i) = 1;
    }

    _C_Acc.setZero();
    for (int i=0; i< n_ay; i++) {
        _C_Acc(i, i) = 1;
    }
    updateSSParams();
}

void AngaccAccEKF::updateSSParams()
{
    _Q_Ang.setZero();
    float pn_rat_sq = _params.pn_ang_rate * _params.pn_ang_rate;
    float pn_ang_sq = _params.pn_ang_acc * _params.pn_ang_acc;
    _Q_Ang(X_rx, X_rx) = pn_rat_sq;
    _Q_Ang(X_ry, X_ry) = pn_rat_sq;
    _Q_Ang(X_rz, X_rz) = pn_rat_sq;
    _Q_Ang(X_aax, X_aax) = pn_ang_sq;
    _Q_Ang(X_aay, X_aay) = pn_ang_sq;
    _Q_Ang(X_aaz, X_aaz) = pn_ang_sq;

    _Q_Acc.setZero();
    float pn_vel_sq = _params.pn_vel * _params.pn_vel;
    float pn_acc_sq = _params.pn_acc * _params.pn_acc;
    _Q_Acc(X_vx, X_vx) = pn_vel_sq;
    _Q_Acc(X_vy, X_vy) = pn_vel_sq;
    _Q_Acc(X_vz, X_vz) = pn_vel_sq;
    _Q_Acc(X_ax, X_ax) = pn_acc_sq;
    _Q_Acc(X_ay, X_ay) = pn_acc_sq;
    _Q_Acc(X_az, X_az) = pn_acc_sq;

    _R_Ang.setZero();
    float ang_rat_p_var = _params.std_ang_rate * _params.std_ang_rate;
    _R_Ang(Y_rx, Y_rx) = ang_rat_p_var;
	_R_Ang(Y_ry, Y_ry) = ang_rat_p_var;
	_R_Ang(Y_rz, Y_rz) = ang_rat_p_var;
#ifdef USING_ANGACC
	float ang_acc_p_var = _params.std_ang_acc * _params.std_ang_acc;
    _R_Ang(Y_aax, Y_aax) = ang_acc_p_var;
	_R_Ang(Y_aay, Y_aay) = ang_acc_p_var;
	_R_Ang(Y_aaz, Y_aaz) = ang_acc_p_var;
#endif

    _R_Acc.setZero();
    float vel_p_var = _params.std_vel * _params.std_vel;
    _R_Acc(Y_vx, Y_vx) = vel_p_var;
	_R_Acc(Y_vy, Y_vy) = vel_p_var;
	_R_Acc(Y_vz, Y_vz) = vel_p_var;
#ifdef USING_ACC
    float acc_p_var = _params.std_acc * _params.std_acc;
    _R_Acc(Y_ax, Y_ax) = acc_p_var;
	_R_Acc(Y_ay, Y_ay) = acc_p_var;
	_R_Acc(Y_az, Y_az) = acc_p_var;
#endif
}

void
AngaccAccEKF::task_main_trampoline(int argc, char *argv[])
{
	angacc_acc_ekf::est_instant->task_main();
}

void AngaccAccEKF::task_main(){
    //do subscriptions
    _sub_sensor_gyro = orb_subscribe(ORB_ID(sensor_gyro));
    _sub_ctrl_state = orb_subscribe(ORB_ID(control_state));
    _sub_param_update = orb_subscribe(ORB_ID(parameter_update));
    _sub_local_pos = orb_subscribe(ORB_ID(vehicle_local_position));
    initSS();

    parameters_update();

    
    px4_pollfd_struct_t fds[1];
    fds[0].fd = _sub_sensor_gyro;
	fds[0].events = POLLIN;

    while (!_task_should_exit) {
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 80);
        
        /* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

        /* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("angacc acc: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

        if (fds[0].revents & POLLIN) {
            uint64_t timeStamp = hrt_absolute_time();
            _dt_ang = (timeStamp - _pre_ang_timeStamp) / 1.0e6f;
            _pre_ang_timeStamp = timeStamp;
            
            orb_copy(ORB_ID(sensor_gyro), _sub_sensor_gyro, &_sensor_gyro);

            bool updated;
            orb_check(_sub_ctrl_state, &updated);
            if(updated) {
                orb_copy(ORB_ID(control_state), _sub_ctrl_state, &_ctrl_state);
            }

            orb_check(_sub_param_update, &updated);
            if (updated) {
                parameters_update();
		        updateSSParams();
            }

            orb_check(_sub_local_pos, &updated);

            _vel_updated = updated;
            if (updated) {
                _dt_acc = (timeStamp - _pre_acc_timeStamp) / 1.0e6f;
                _pre_acc_timeStamp = timeStamp;
                orb_copy(ORB_ID(vehicle_local_position), _sub_local_pos, &_local_pos);
                // PX4_INFO("local: %8.4f,%8.4f,%8.4f",(double)_local_pos.vx,(double)_local_pos.vy,(double)_local_pos.vz);
            }

            predict();
            correct();

            //publish
            Vector<float, n_rx> x_angLP = _x_angaccLowPass.getState();
            if(_x_angaccLowPass.getFCut() < 1.0e-6f) {
                x_angLP = _x_ang;
            }

            _fake_mocap.timestamp   = _angacc_acc.timestamp = timeStamp;
            _fake_mocap.q[0]        = _angacc_acc.ang_acc_x = x_angLP(X_aax);
            _fake_mocap.q[1]        = _angacc_acc.ang_acc_y = x_angLP(X_aay);
            _fake_mocap.q[2]        = _angacc_acc.ang_acc_z = x_angLP(X_aaz);
            _fake_mocap.q[3]        = _angacc_acc.valid_acc = _vel_updated;

            Vector<float, n_ax> x_accLP = _x_accLowPass.getState();
            if(_x_accLowPass.getFCut() < 1.0e-6f) {
                x_accLP = _x_acc;
            }
            if (_vel_updated) {
                 _fake_mocap.x = _angacc_acc.acc_x = x_accLP(X_ax);
                 _fake_mocap.y = _angacc_acc.acc_y = x_accLP(X_ay);
                 _fake_mocap.z = _angacc_acc.acc_z = x_accLP(X_az);
            }
            

            if (_pub_angacc_acc == nullptr) {
                _pub_angacc_acc = orb_advertise(ORB_ID(angacc_acc), &_angacc_acc);
            } else {
                orb_publish(ORB_ID(angacc_acc), _pub_angacc_acc, &_angacc_acc);
            }

            if (_pub_fake_mocap == nullptr) {
                _pub_fake_mocap = orb_advertise(ORB_ID(att_pos_mocap), &_fake_mocap);
            } else {
                orb_publish(ORB_ID(att_pos_mocap), _pub_fake_mocap, &_fake_mocap);
            }
            // PX4_INFO("angacc: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f", 
            //         (double)_angacc_acc.ang_acc_x,
            //         (double)_angacc_acc.ang_acc_y,
            //         (double)_angacc_acc.ang_acc_z,
            //         (double)_angacc_acc.acc_x,
            //         (double)_angacc_acc.acc_y,
            //         (double)_angacc_acc.acc_z );
        }
    }
}

void AngaccAccEKF::predict() {
    float h = _dt_ang;
    _A_Ang.setIdentity();
    _A_Ang(X_rx, X_aax) = h;  //250 Hz
    _A_Ang(X_ry, X_aay) = h;
    _A_Ang(X_rz, X_aaz) = h;

//     PX4_INFO("time, %8.4f",(double) h);

    _x_ang  = _A_Ang * _x_ang;
    // covPropagationLogic(_P);
    _P_Ang = _A_Ang * _P_Ang * _A_Ang.transpose() + _Q_Ang;
    
    setDt(h);       //must set time, otherwise lowpass not work

//    PX4_INFO("ang_x:, %8.4f,%8.4f,%8.4f",(double)_x_ang(3),(double)_x_ang(4),(double)_x_ang(5));
    _x_angaccLowPass.update(_x_ang);

    if (_vel_updated) {
        h = _dt_acc;
        // PX4_INFO("time2, %8.4f",(double) h);
        _A_Acc.setIdentity();
        _A_Acc(X_vx, X_ax) = h;
        _A_Acc(X_vy, X_ay) = h;
        _A_Acc(X_vz, X_az) = h;
        _x_acc  = _A_Acc * _x_acc;
        // covPropagationLogic(_P);
        _P_Acc = _A_Acc * _P_Acc * _A_Acc.transpose() + _Q_Acc;
        setDt(h);
        // PX4_INFO("_dt_acc is: %8.4f", (double)_dt_acc);
        _x_accLowPass.update(_x_acc);
    }
}

void AngaccAccEKF::correct() {

    Vector<float, n_ry> y_ang;
    Vector3f rate(_sensor_gyro.x, _sensor_gyro.y, _sensor_gyro.z);
    y_ang(Y_rx) = rate(0);
    y_ang(Y_ry) = rate(1);
    y_ang(Y_rz) = rate(2);
#ifdef USING_ANGACC
    float h = _dt_ang;
    static Vector3f pre_rate(0.0f, 0.0f, 0.0f);
    static Vector3f pre_angacc(0.0f, 0.0f, 0.0f);
    Vector3f angacc = pre_angacc * 0.1 + (rate - pre_rate) * 0.9/h;
    
    // PX4_INFO("anacc: %8.4f,%8.4f,%8.4f", (double)angacc(0), (double)angacc(1), (double)angacc(2));
    y_ang(Y_aax) = angacc(0);
    y_ang(Y_aay) = angacc(1);
    y_ang(Y_aaz) = angacc(2);
    pre_rate = rate;
    pre_angacc = angacc;

//    PX4_INFO("y_ang: %8.4f,%8.4f,%8.4f,%8.4f,%8.4f,%8.4f", (double)y_ang(Y_rx),(double)y_ang(Y_ry),(double)y_ang(Y_rz),
//    		(double)y_ang(Y_aax),(double)y_ang(Y_aay),(double)y_ang(Y_aaz));
#endif
    Matrix<float, n_ry, n_ry> S_I_Ang = inv<float, n_ry>((_C_Ang * _P_Ang * _C_Ang.transpose()) + _R_Ang);
    Matrix<float, n_ry, 1> r_ang = y_ang - _C_Ang * _x_ang;

    Matrix<float,n_rx, n_ry> _K_Ang = _P_Ang * _C_Ang.transpose() * S_I_Ang;
    // correctionLogic(dx);

    _x_ang += _K_Ang * r_ang;
    _P_Ang -= _K_Ang * _C_Ang * _P_Ang;

//     _x_ang(3) = angacc(0);
//     _x_ang(4) = angacc(1);
//     _x_ang(5) = angacc(2);

    if (_vel_updated) {
        Vector<float, n_ay> y_acc;
        y_acc(Y_vx) = _local_pos.vx;
        y_acc(Y_vy) = _local_pos.vy;
        y_acc(Y_vz) = _local_pos.vz;

#ifdef USING_ACC
        Quaternionf q(_ctrl_state.q[0], _ctrl_state.q[1],
    					_ctrl_state.q[2], _ctrl_state.q[3]);
        Dcmf R(q); //form body to word
        Vector<float, 3> gb = R.transpose() * Vector3f(0.0f, 0.0f, 9.806f);
        y_acc(Y_ax) = _ctrl_state.x_acc + gb(0);
        y_acc(Y_ay) = _ctrl_state.y_acc + gb(1);
        y_acc(Y_az) = _ctrl_state.z_acc + gb(2);
#endif

        Matrix<float, n_ay, n_ay> S_I_Acc = \
            inv<float, n_ay>((_C_Acc * _P_Acc * _C_Acc.transpose()) + _R_Acc);
        Matrix<float, n_ay, 1> r_acc = y_acc - _C_Acc * _x_acc;

        Matrix<float, n_ax, n_ay> _K_Acc = _P_Acc * _C_Acc.transpose() * S_I_Acc;
        // correctionLogic(dx);

        _x_acc += _K_Acc * r_acc;
        _P_Acc -= _K_Acc * _C_Acc * _P_Acc;
    } 
}

int AngaccAccEKF::start()
{
	/* start the task */
	_control_task = px4_task_spawn_cmd("angacc_acc_ekf",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 2,
					   5000,
					   (px4_main_t)&AngaccAccEKF::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


int angacc_acc_ekf_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: angacc_acc_ekf {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (angacc_acc_ekf::est_instant != nullptr) {
			warnx("already running");
			return 1;
		}

		angacc_acc_ekf::est_instant = new AngaccAccEKF;

		if (angacc_acc_ekf::est_instant == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != angacc_acc_ekf::est_instant->start()) {
			delete angacc_acc_ekf::est_instant;
			angacc_acc_ekf::est_instant = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (angacc_acc_ekf::est_instant == nullptr) {
			warnx("not running");
			return 1;
		}

		delete angacc_acc_ekf::est_instant;
		angacc_acc_ekf::est_instant = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (angacc_acc_ekf::est_instant) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
