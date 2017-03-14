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

// uORB Suscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/control_state.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/angacc_acc.h>

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
    enum {X_angx = 0, X_angy, X_angz, X_ratx, X_raty, X_ratz,\
          X_accx, X_accy, X_accz, X_velx, X_vely, X_velz, n_x};
    enum {Y_ratx = 0, Y_raty, Y_ratz, Y_accx, Y_accy, Y_accz, Y_velx, Y_vely, Y_velz, n_y};

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
    BlockParamFloat _angular_rat_stddev;
    BlockParamFloat _acc_stddev;
    BlockParamFloat _vel_stddev;
    BlockLowPassVector<float, n_x> _xLowPass;

    uint64_t _timeStamp;
    px4_pollfd_struct_t _polls[2];

    Vector<float, n_x> _x;		 // state Vector
	Matrix<float, n_x, n_x>  _P; // state covariance matrix
    Matrix<float, n_y, n_x>  _C;

    Matrix<float, n_x, n_x>  _A; // dynamics matrix
	Matrix<float, n_x, n_x>  _Q; // process noise covariance
	Matrix<float, n_y, n_y>  _R; // measurement covariance
};

AngaccAccEKF::AngaccAccEKF() :
    // This block has no parent, and has name AAE
    SuperBlock(NULL,"AAE"),
    _pub_angacc_acc(ORB_ID(angacc_acc), -1, &getPublications()),
    _sub_state(ORB_ID(control_state), 1000/100, 0, &getSubscriptions()),
    _sub_param_update(ORB_ID(parameter_update), 1000/2, 0, &getSubscriptions()),
    _sub_local_pos(ORB_ID(vehicle_local_position), 1000/100, 0, &getSubscriptions()),
    _pn_angacc_noise_density(this, "PN_ANGACC"),
    _pn_angrat_noise_density(this, "PN_ANGRATE"),
    _pn_acc_noise_density(this, "PN_ACC"),
    _pn_vel_noise_density(this, "PN_VEL"),
    _angular_rat_stddev(this, "ANG_RATE"),
    _acc_stddev(this, "ACC_STD"),
    _vel_stddev(this, "VEL_STD"),
    _xLowPass(this, "X_LP"),
	_timeStamp(hrt_absolute_time()),
    _polls(),
    _x(), _P(), _C(), _A(), _Q(), _R()
{
    _polls[0].fd = _sub_state.getHandle();
	_polls[0].events = POLLIN;
    _polls[1].fd = _sub_local_pos.getHandle();
	_polls[1].events = POLLIN;
    _x.setZero();
	initSS();
    // intialize parameter dependent matrices
	updateParams();
}

void AngaccAccEKF::update()
{
    int ret = px4_poll(_polls,1,100);

    if (ret <= 0) {
        return;
    }

    uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// set dt for all child blocks
	setDt(dt);

    bool paramsUpdated = _sub_param_update.updated();
    updateSubscriptions();

	// update parameters
	if (paramsUpdated) {
		updateParams();
		updateSSParams();
	}

    bool reinit_P = false;

	for (int i = 0; i < n_x; i++) {
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				mavlink_and_console_log_info(&mavlink_log_pub,
							     "[lpe] reinit P (%d, %d) not finite", i, j);
				reinit_P = true;
			}

			if (i == j) {
				// make sure diagonal elements are positive
				if (_P(i, i) <= 0) {
					mavlink_and_console_log_info(&mavlink_log_pub,
								     "[lpe] reinit P (%d, %d) negative", i, j);
					reinit_P = true;
				}

			} else {
				// copy elememnt from upper triangle to force
				// symmetry
				_P(j, i) = _P(i, j);
			}

			if (reinit_P) { break; }
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
//		initP();
	}

    predict();
    correct();

    const Vector<float, n_x> &xLP = _xLowPass.getState();
    _pub_angacc_acc.get().ang_acc_x = xLP(X_angx);
    _pub_angacc_acc.get().ang_acc_y = xLP(X_angy);
    _pub_angacc_acc.get().ang_acc_z = xLP(X_angz);
    _pub_angacc_acc.get().acc_x = xLP(X_accx);
    _pub_angacc_acc.get().acc_y = xLP(X_accy);
    _pub_angacc_acc.get().acc_z = xLP(X_accz);
    _pub_angacc_acc.update();
}

void AngaccAccEKF::correct()
{
    Vector<float, n_y> y;
    y(0) = _sub_state.get().roll_rate;
    y(1) = _sub_state.get().pitch_rate;
    y(2) = _sub_state.get().yaw_rate;

    Quaternionf q(_sub_state.get().q[0], _sub_state.get().q[1],
    					_sub_state.get().q[2], _sub_state.get().q[3]);
    Dcmf R(q); //form body to word
    Vector<float, 3> gb = R.transpose() * Vector3f(0.0f, 0.0f, 9.8f);
    y(3) = _sub_state.get().x_acc + gb(0);
    y(4) = _sub_state.get().y_acc + gb(1);
    y(5) = _sub_state.get().z_acc + gb(2);

    y(6) = _sub_local_pos.get().vx;
    y(7) = _sub_local_pos.get().vy;
    y(8) = _sub_local_pos.get().vz;



    Matrix<float, n_y, n_y> S_I = inv<float, n_y>((_C * _P * _C.transpose()) + _R);
    Matrix<float, n_y, 1> r = y - _C * _x;

    Matrix<float, n_x, n_y> _K = _P * _C.transpose() * S_I;
    Vector<float, n_x> dx = _K * r;
    // correctionLogic(dx);

    _x += dx;
    _P -= _K * _C * _P;
}

void AngaccAccEKF::predict()
{
    float h = getDt();
    _A.setIdentity();
    _A(X_ratx, X_angx) = h;  //250 Hz
    _A(X_raty, X_angy) = h;
    _A(X_ratz, X_angz) = h;
    _A(X_velx, X_accx) = h;  //250 Hz
    _A(X_vely, X_accy) = h;
    _A(X_velz, X_accz) = h;

    _x  = _A * _x;
    // covPropagationLogic(_P);
    _P = _A * _P * _A.transpose() + _Q;

    _xLowPass.update(_x);
}


void AngaccAccEKF::initSS()
{
    _P.setZero();
    float p_ang_init = 1e-6;
    float p_acc_init = 1e-5;
    _P(X_angx, X_angx) = p_ang_init;
    _P(X_angy, X_angy) = p_ang_init;
    _P(X_angz, X_angz) = p_ang_init;
    _P(X_ratx, X_ratx) = p_ang_init;
    _P(X_raty, X_raty) = p_ang_init;
    _P(X_ratz, X_ratz) = p_ang_init;
    _P(X_accx, X_accx) = p_acc_init;
    _P(X_accy, X_accy) = p_acc_init;
    _P(X_accz, X_accz) = p_acc_init;
    _P(X_velx, X_velx) = p_acc_init;
    _P(X_vely, X_vely) = p_acc_init;
    _P(X_velz, X_velz) = p_acc_init;

    _A.setIdentity();
    float a_ang_init = 4e-3;
    float a_acc_init = 4e-3;
    _A(X_ratx, X_angx) = a_ang_init;  //250 Hz
    _A(X_raty, X_angy) = a_ang_init;
    _A(X_ratz, X_angz) = a_ang_init;
    _A(X_velx, X_accx) = a_acc_init;  //250 Hz
    _A(X_vely, X_accy) = a_acc_init;
    _A(X_velz, X_accz) = a_acc_init;
    
    _C.setZero();
    _C(Y_ratx, X_ratx) = 1;
    _C(Y_raty, X_raty) = 1;
    _C(Y_ratz, X_ratz) = 1;
    _C(Y_accx, X_accx) = 1;
    _C(Y_accy, X_accy) = 1;
    _C(Y_accz, X_accz) = 1;
    _C(Y_velx, X_velx) = 1;
    _C(Y_vely, X_vely) = 1;
    _C(Y_velz, X_velz) = 1;

    updateSSParams();
}
    
void AngaccAccEKF::updateSSParams()
{
    _Q.setZero();
    float pn_ang_sq = _pn_angacc_noise_density.get() * _pn_angacc_noise_density.get();
    float pn_rat_sq = _pn_angrat_noise_density.get() * _pn_angrat_noise_density.get();
    float pn_acc_sq = _pn_acc_noise_density.get() * _pn_acc_noise_density.get();
    float pn_vel_sq = _pn_vel_noise_density.get() * _pn_vel_noise_density.get();
    _Q(X_angx, X_angx) = pn_ang_sq;
    _Q(X_angy, X_angy) = pn_ang_sq;
    _Q(X_angz, X_angz) = pn_ang_sq;
    _Q(X_ratx, X_ratx) = pn_rat_sq;
    _Q(X_raty, X_raty) = pn_rat_sq;
    _Q(X_ratz, X_ratz) = pn_rat_sq;
    _Q(X_accx, X_accx) = pn_acc_sq;
    _Q(X_accy, X_accy) = pn_acc_sq;
    _Q(X_accz, X_accz) = pn_acc_sq;
    _Q(X_velx, X_velx) = pn_vel_sq;
    _Q(X_vely, X_vely) = pn_vel_sq;
    _Q(X_velz, X_velz) = pn_vel_sq;

    _R.setZero();
    float ang_rat_p_var = _angular_rat_stddev.get() * _angular_rat_stddev.get();
    float acc_p_var = _acc_stddev.get() * _acc_stddev.get();
    float vel_p_var = _vel_stddev.get() * _vel_stddev.get();
    _R(Y_ratx, Y_ratx) = ang_rat_p_var;
	_R(Y_raty, Y_raty) = ang_rat_p_var;
	_R(Y_ratz, Y_ratz) = ang_rat_p_var;
    _R(Y_accx, Y_accx) = acc_p_var;
	_R(Y_accy, Y_accy) = acc_p_var;
	_R(Y_accz, Y_accz) = acc_p_var;
    _R(Y_velx, Y_velx) = vel_p_var;
	_R(Y_vely, Y_vely) = vel_p_var;
	_R(Y_velz, Y_velz) = vel_p_var;
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
						 1500,
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

	using namespace control;

	AngaccAccEKF est;

	thread_running = true;

	while (!thread_should_exit) {
		est.update();
	}

	PX4_DEBUG("exiting.");

	thread_running = false;

	return 0;
}
