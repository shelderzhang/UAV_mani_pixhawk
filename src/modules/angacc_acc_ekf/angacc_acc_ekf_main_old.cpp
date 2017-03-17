/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file angacc_acc_ekf.cpp
 *
 * Angular rate and acceleration estimator (ekf based)
 *
 * @author bdai <bdai1412@gmail.com>
 */

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
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/angacc_acc.h>
#include <uORB/topics/att_pos_mocap.h>

// #define DIFF_LOWPASS

using namespace matrix;
using namespace control;

extern orb_advert_t mavlink_log_pub;

static volatile bool thread_should_exit = false;     /**< Deamon exit flag */
static volatile bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */

extern "C" __EXPORT int angacc_acc_ekf_main(int argc, char *argv[]);
int angacc_acc_ekf_thread_main(int argc, char *argv[]);

class AngaccAccEKF : public control::SuperBlock
{
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
    enum {X_angx = 0, X_angy, X_angz, X_ratx, X_raty, X_ratz, n_x_ang};
    enum {Y_ratx = 0, Y_raty, Y_ratz, n_y_ang};
    
    enum {X_accx, X_accy, X_accz, X_velx, X_vely, X_velz, n_x_acc};
    enum {Y_accx, Y_accy, Y_accz, Y_velx, Y_vely, Y_velz, n_y_acc};

    AngaccAccEKF();
    ~AngaccAccEKF() {};
    void update();
private:
    // prevent copy and assignment
    AngaccAccEKF(const AngaccAccEKF &);
    AngaccAccEKF operator=(const AngaccAccEKF &);

	void initSS();
    void updateSSParams();
    void predict();
    void correct();

    //publications
    uORB::Publication<angacc_acc_s> _pub_angacc_acc;
    uORB::Publication<att_pos_mocap_s> _monitor_angacc_acc; //only used for monitoring angacc and acc in qgroundcontrol

    //subscriptions
    uORB::Subscription<control_state_s> _sub_state;
    uORB::Subscription<parameter_update_s> _sub_param_update;
    uORB::Subscription<vehicle_local_position_s> _sub_local_pos;

    //process noise
    BlockParamFloat _pn_angacc_noise_density;
    BlockParamFloat _pn_angrat_noise_density;
    BlockParamFloat _pn_acc_noise_density;
    BlockParamFloat _pn_vel_noise_density;

    // measurement noise
    BlockParamFloat _ang_rat_stddev;
    BlockParamFloat _acc_stddev;
    BlockParamFloat _vel_stddev;

    BlockLowPassVector<float, n_x_ang> _x_angaccLowPass;
    BlockLowPassVector<float, n_x_acc> _x_accLowPass;

    uint64_t _pre_ang_timeStamp;
    uint64_t _pre_acc_timeStamp;
    float _dt_ang;
    float _dt_acc;

    bool _valid_angacc;
    bool _valid_acc;
    bool _pos_updated;
    px4_pollfd_struct_t _polls;

    Vector<float, n_x_ang> _x_ang;		 // state Vector
	Matrix<float, n_x_ang, n_x_ang>  _P_Ang; // state covariance matrix
    Matrix<float, n_y_ang, n_x_ang>  _C_Ang;

    Matrix<float, n_x_ang, n_x_ang>  _A_Ang; // dynamics matrix
	Matrix<float, n_x_ang, n_x_ang>  _Q_Ang; // process noise covariance
	Matrix<float, n_y_ang, n_y_ang>  _R_Ang; // measurement covariance

    Vector<float, n_x_acc> _x_acc;		 // state Vector
	Matrix<float, n_x_acc, n_x_acc>  _P_Acc; // state covariance matrix
    Matrix<float, n_y_acc, n_x_acc>  _C_Acc;

    Matrix<float, n_x_acc, n_x_acc>  _A_Acc; // dynamics matrix
	Matrix<float, n_x_acc, n_x_acc>  _Q_Acc; // process noise covariance
	Matrix<float, n_y_acc, n_y_acc>  _R_Acc; // measurement covariance

};

AngaccAccEKF::AngaccAccEKF() :
    // This block has no parent, and has name AAE
    SuperBlock(NULL,"AAE"),
    _pub_angacc_acc(ORB_ID(angacc_acc), -1, &getPublications()),
    _monitor_angacc_acc(ORB_ID(att_pos_mocap), -1, &getPublications()),
    _sub_state(ORB_ID(control_state), 1000/100, 0, &getSubscriptions()),
    _sub_param_update(ORB_ID(parameter_update), 1000/2, 0, &getSubscriptions()),
    _sub_local_pos(ORB_ID(vehicle_local_position), 1000/100, 0, &getSubscriptions()),
    _pn_angacc_noise_density(this, "PN_ANGACC"),
    _pn_angrat_noise_density(this, "PN_ANGRATE"),
    _pn_acc_noise_density(this, "PN_ACC"),
    _pn_vel_noise_density(this, "PN_VEL"),
    _ang_rat_stddev(this, "STD_ANGRA"),
    _acc_stddev(this, "STD_AC"),
    _vel_stddev(this, "STD_VEL"),
    _x_angaccLowPass(this, "X_ANG_LP"),
    _x_accLowPass(this, "X_ACC_LP"),
	_pre_ang_timeStamp(hrt_absolute_time()),
    _pre_acc_timeStamp(hrt_absolute_time()),
    _dt_ang(0.0f),
    _dt_acc(0.0f),
	_valid_angacc(false),
    _valid_acc(false),
    _pos_updated(false)
{
    _polls.fd = _sub_state.getHandle();
	_polls.events = POLLIN;

	initSS();
    // intialize parameter dependent matrices
	updateParams();
}

void AngaccAccEKF::update()
{
    int ret = px4_poll(&_polls, 1, 100);

    if (ret <= 0) {
        return;
    }

    uint64_t timeStamp = hrt_absolute_time();
    _dt_ang = (timeStamp - _pre_ang_timeStamp) / 1.0e6f;
    _pre_ang_timeStamp = timeStamp;

    _pos_updated = _sub_local_pos.check_updated();
    if (_pos_updated) {
        _dt_acc = (timeStamp - _pre_acc_timeStamp) / 1.0e6f;
        _pre_acc_timeStamp = timeStamp;
    }
    updateSubscriptions();

    bool paramsUpdated = _sub_param_update.updated();

	// update parameters
	if (paramsUpdated) {
		updateParams();
		updateSSParams();
	}
    
    predict();
    correct();

    const Vector<float, n_x_ang> &x_angLP = _x_angaccLowPass.getState();
    _pub_angacc_acc.get().timestamp = timeStamp;
    _pub_angacc_acc.get().ang_acc_x = x_angLP(X_angx);
    _pub_angacc_acc.get().ang_acc_y = x_angLP(X_angy);
    _pub_angacc_acc.get().ang_acc_z = x_angLP(X_angz);
    _pub_angacc_acc.get().valid_acc = _pos_updated;

    const Vector<float, n_x_acc> &x_accLP = _x_accLowPass.getState();
    if (_pos_updated) {
        _pub_angacc_acc.get().acc_x = x_accLP(X_accx);
        _pub_angacc_acc.get().acc_y = x_accLP(X_accy);
        _pub_angacc_acc.get().acc_z = x_accLP(X_accz);
    }
    _pub_angacc_acc.update();

    // used update fake mocap information into qgroundcontrol, for monitoring angacc and acc
    {
        _monitor_angacc_acc.get().timestamp = timeStamp;
        _monitor_angacc_acc.get().q[0] = x_angLP(X_angx);
        _monitor_angacc_acc.get().q[1] = x_angLP(X_angy);
        _monitor_angacc_acc.get().q[2] = x_angLP(X_angz);
        _monitor_angacc_acc.get().q[3] = _pos_updated;
        if (_pos_updated) {
            _monitor_angacc_acc.get().x = x_accLP(X_angx);
            _monitor_angacc_acc.get().y = x_accLP(X_accy);
            _monitor_angacc_acc.get().z = x_accLP(X_accy);
        }
         _monitor_angacc_acc.update();
    }
   
}

void AngaccAccEKF::correct()
{
    Vector<float, n_y_ang> y_ang;
    y_ang(Y_ratx) = _sub_state.get().roll_rate;
    y_ang(Y_raty) = _sub_state.get().pitch_rate;
    y_ang(Y_ratz) = _sub_state.get().yaw_rate;
    Matrix<float, n_y_ang, n_y_ang> S_I_Ang = \
        inv<float, n_y_ang>((_C_Ang * _P_Ang * _C_Ang.transpose()) + _R_Ang);
    Matrix<float, n_y_ang, 1> r_ang = y_ang - _C_Ang * _x_ang;

    Matrix<float, n_x_ang, n_y_ang> _K_Ang = _P_Ang * _C_Ang.transpose() * S_I_Ang;
    // correctionLogic(dx);

    _x_ang += _K_Ang * r_ang;
    _P_Ang -= _K_Ang * _C_Ang * _P_Ang;

    if (_pos_updated) {
        Vector<float, n_y_acc> y_acc;

        Quaternionf q(_sub_state.get().q[0], _sub_state.get().q[1],
    					_sub_state.get().q[2], _sub_state.get().q[3]);
        Dcmf R(q); //form body to word
        Vector<float, 3> gb = R.transpose() * Vector3f(0.0f, 0.0f, 9.806f);
        y_acc(Y_accx) = _sub_state.get().x_acc + gb(0);
        y_acc(Y_accy) = _sub_state.get().y_acc + gb(1);
        y_acc(Y_accz) = _sub_state.get().z_acc + gb(2);

        y_acc(Y_velx) = _sub_local_pos.get().vx;
        y_acc(Y_vely) = _sub_local_pos.get().vy;
        y_acc(Y_velz) = _sub_local_pos.get().vz;

        Matrix<float, n_y_acc, n_y_acc> S_I_Acc = \
            inv<float, n_y_acc>((_C_Acc * _P_Acc * _C_Acc.transpose()) + _R_Acc);
        Matrix<float, n_y_acc, 1> r_acc = y_acc - _C_Acc * _x_acc;

        Matrix<float, n_x_acc, n_y_acc> _K_Acc = _P_Acc * _C_Acc.transpose() * S_I_Acc;
        // correctionLogic(dx);

        _x_acc += _K_Acc * r_acc;
        _P_Acc -= _K_Acc * _C_Acc * _P_Acc;
    }
}

void AngaccAccEKF::predict()
{
    float h = _dt_ang;
    _A_Ang.setIdentity();
    _A_Ang(X_ratx, X_angx) = h;  //250 Hz
    _A_Ang(X_raty, X_angy) = h;
    _A_Ang(X_ratz, X_angz) = h;
    _x_ang  = _A_Ang * _x_ang;
    // covPropagationLogic(_P);
    _P_Ang = _A_Ang * _P_Ang * _A_Ang.transpose() + _Q_Ang;

    setDt(h);       //must set time, otherwise lowpass not work
    _x_angaccLowPass.update(_x_ang);

    if (_pos_updated) {
        h = _dt_acc;
        _A_Acc.setIdentity();
        _A_Acc(X_velx, X_accx) = h;  //250 Hz
        _A_Acc(X_vely, X_accy) = h;
        _A_Acc(X_velz, X_accz) = h;
        _x_acc  = _A_Acc * _x_acc;
        // covPropagationLogic(_P);
        _P_Acc = _A_Acc * _P_Acc * _A_Acc.transpose() + _Q_Acc;
        setDt(h);
        // PX4_INFO("lowpass is: %8.4f", (double)_x_accLowPass.getFCut());
        _x_accLowPass.update(_x_acc);
    }
}


void AngaccAccEKF::initSS()
{
    _x_ang.setZero();
    _x_acc.setZero();

    _P_Ang.setZero();
    float p_ang_init = 1e-6;
    _P_Ang(X_angx, X_angx) = p_ang_init;
    _P_Ang(X_angy, X_angy) = p_ang_init;
    _P_Ang(X_angz, X_angz) = p_ang_init;
    _P_Ang(X_ratx, X_ratx) = p_ang_init;
    _P_Ang(X_raty, X_raty) = p_ang_init;
    _P_Ang(X_ratz, X_ratz) = p_ang_init;

    _P_Acc.setZero();
    float p_acc_init = 1e-5;
    _P_Acc(X_accx, X_accx) = p_acc_init;
    _P_Acc(X_accy, X_accy) = p_acc_init;
    _P_Acc(X_accz, X_accz) = p_acc_init;
    _P_Acc(X_velx, X_velx) = p_acc_init;
    _P_Acc(X_vely, X_vely) = p_acc_init;
    _P_Acc(X_velz, X_velz) = p_acc_init;

    _A_Ang.setIdentity();
    float a_ang_init = 4e-3;
    _A_Ang(X_ratx, X_angx) = a_ang_init;  //250 Hz
    _A_Ang(X_raty, X_angy) = a_ang_init;
    _A_Ang(X_ratz, X_angz) = a_ang_init;

    _A_Acc.setIdentity();
    float a_acc_init = 4e-3;
    _A_Acc(X_velx, X_accx) = a_acc_init;  //250 Hz
    _A_Acc(X_vely, X_accy) = a_acc_init;
    _A_Acc(X_velz, X_accz) = a_acc_init;
    
    _C_Ang.setZero();
    _C_Ang(Y_ratx, X_ratx) = 1;
    _C_Ang(Y_raty, X_raty) = 1;
    _C_Ang(Y_ratz, X_ratz) = 1;

    _C_Acc.setZero();
    _C_Acc(Y_accx, X_accx) = 1;
    _C_Acc(Y_accy, X_accy) = 1;
    _C_Acc(Y_accz, X_accz) = 1;
    _C_Acc(Y_velx, X_velx) = 1;
    _C_Acc(Y_vely, X_vely) = 1;
    _C_Acc(Y_velz, X_velz) = 1;

    updateSSParams();
}
    
void AngaccAccEKF::updateSSParams()
{
    _Q_Ang.setZero();
    float pn_ang_sq = _pn_angacc_noise_density.get() * _pn_angacc_noise_density.get();
    float pn_rat_sq = _pn_angrat_noise_density.get() * _pn_angrat_noise_density.get();
    _Q_Ang(X_angx, X_angx) = pn_ang_sq;
    _Q_Ang(X_angy, X_angy) = pn_ang_sq;
    _Q_Ang(X_angz, X_angz) = pn_ang_sq;
    _Q_Ang(X_ratx, X_ratx) = pn_rat_sq;
    _Q_Ang(X_raty, X_raty) = pn_rat_sq;
    _Q_Ang(X_ratz, X_ratz) = pn_rat_sq;

    _Q_Acc.setZero();
    float pn_acc_sq = _pn_acc_noise_density.get() * _pn_acc_noise_density.get();
    float pn_vel_sq = _pn_vel_noise_density.get() * _pn_vel_noise_density.get();
    _Q_Acc(X_accx, X_accx) = pn_acc_sq;
    _Q_Acc(X_accy, X_accy) = pn_acc_sq;
    _Q_Acc(X_accz, X_accz) = pn_acc_sq;
    _Q_Acc(X_velx, X_velx) = pn_vel_sq;
    _Q_Acc(X_vely, X_vely) = pn_vel_sq;
    _Q_Acc(X_velz, X_velz) = pn_vel_sq;

    _R_Ang.setZero();
    float ang_rat_p_var = _ang_rat_stddev.get() * _ang_rat_stddev.get();
    _R_Ang(Y_ratx, Y_ratx) = ang_rat_p_var;
	_R_Ang(Y_raty, Y_raty) = ang_rat_p_var;
	_R_Ang(Y_ratz, Y_ratz) = ang_rat_p_var;

    _R_Acc.setZero();
    float acc_p_var = _acc_stddev.get() * _acc_stddev.get();
    float vel_p_var = _vel_stddev.get() * _vel_stddev.get();
    _R_Acc(Y_accx, Y_accx) = acc_p_var;
	_R_Acc(Y_accy, Y_accy) = acc_p_var;
	_R_Acc(Y_accz, Y_accz) = acc_p_var;
    _R_Acc(Y_velx, Y_velx) = vel_p_var;
	_R_Acc(Y_vely, Y_vely) = vel_p_var;
	_R_Acc(Y_velz, Y_velz) = vel_p_var;
}



/**
 * Print the correct usage.
 */
static int usage(const char *reason);

static int
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: angacc_acc_ekf {start|stop|status} [-p <additional params>]\n\n");
	return 1;
}

int angacc_acc_ekf_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_INFO("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;

		deamon_task = px4_task_spawn_cmd("angacc_acc_ekf",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX,
						 5000,
						 angacc_acc_ekf_thread_main,
						 (argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			PX4_DEBUG("stop");
			thread_should_exit = true;

		} else {
			PX4_WARN("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			PX4_INFO("is running");

		} else {
			PX4_INFO("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int angacc_acc_ekf_thread_main(int argc, char *argv[])
{

	PX4_DEBUG("starting");

	AngaccAccEKF est;

	thread_running = true;

	while (!thread_should_exit) {
		est.update();
	}

	PX4_DEBUG("exiting.");

	thread_running = false;

	return 0;
}
