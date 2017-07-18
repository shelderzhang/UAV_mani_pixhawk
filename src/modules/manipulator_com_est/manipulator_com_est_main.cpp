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

#include "CMatrixFactory.h"
#include <uORB/uORB.h>
#include <uORB/uORB.h>
// uORB Suscriptions
#include <uORB/topics/manipulator_joint_status.h>

// uORB Publications
#include <uORB/topics/mani_com.h>


using namespace matrix;
using namespace control;

typedef SquareMatrix<float, 4> Matrix4f;

extern orb_advert_t mavlink_log_pub;
extern "C" __EXPORT int manipulator_com_est_main(int argc, char *argv[]);

struct DH_params_s {
	float d[2];
	float a[2];
	float alpha[2];
	float offset[2];
};

struct inertia_params_s {
	float mb;
	float mlink0;
	float mlink1;
	float mlink2;
	Vector3f rc0;
	Vector3f rc1;
	Vector3f rc2;
	Matrix3f Ic0;
	Matrix3f Ic1;
	Matrix3f Ic2;
};
class ManipulatorCoMEst: public control::SuperBlock {
public:
	ManipulatorCoMEst();
	~ManipulatorCoMEst();
	int start();

private:
	bool _task_should_exit;
	int _control_task;

	int _sub_manipulator_joint_status;
	orb_advert_t _pub_mani_com;

	struct manipulator_joint_status_s _manipulator_joint_status;
	struct mani_com_s _mani_com;

	struct DH_params_s mani_DH;
	struct inertia_params_s mani_iner;

	/* vector from the origin of body frame to the frame zero of the arm
	 * gyzhang <Jul 17, 2017>*/
	Vector3f rb0;

	//px4_pollfd_struct_t _polls;

	void com_estimator();

//	Shim for calling task_main from task_create.
	static void task_main_trampoline(int argc, char *argv[]);
//  Main attitude control task.

	void task_main();

};
namespace manipulator_com_est {

ManipulatorCoMEst *com_est_control;
}

ManipulatorCoMEst::ManipulatorCoMEst() :
		SuperBlock(NULL, "MANC"), _task_should_exit(false), _control_task(-1),

		/* subscriptions */
		_sub_manipulator_joint_status(-1),

		/* publications */
		_pub_mani_com(nullptr)

{
	memset(&_manipulator_joint_status, 0, sizeof(_manipulator_joint_status));
	memset(&_mani_com, 0, sizeof(_mani_com));
	mani_DH.d[0] = 0.167f;
	mani_DH.d[1] = 0.0f;
	mani_DH.a[0] = 0.0f;
	mani_DH.a[1] = 0.0375f;
	mani_DH.offset[0] = 0.0f;
	mani_DH.offset[1] = (float) M_PI / 2.0f;
	mani_DH.alpha[0] = (float) M_PI / 2.0f;
	mani_DH.alpha[1] = 0.0f;

	mani_iner.mb = 3.56f;
	mani_iner.mlink0 = 0.3210f;
	mani_iner.mlink1 = 0.3710f;
	mani_iner.mlink2 = 0.3220f;
	mani_iner.rc0 = Vector3f(-0.0016f, -7.8226e-6f, 0.0496f);
	mani_iner.rc1 = Vector3f(-0.0015f, -0.0124f, 0.009f);
	mani_iner.rc2 = Vector3f(-0.99f, 0.0f, 0.0f);

	/*
	 * 	>> mdl_arm.link0_Jc
	 *	ans =
	 *	   1.0e-03 *
	 *	    0.4319   -0.0000    0.0141
	 *	   -0.0000    0.4410   -0.0000
	 *	    0.0141   -0.0000    0.0693
	 *
	 * gyzhang <Jul 14, 2017>*/

	mani_iner.Ic0(0, 0) = 0.4319f;
	mani_iner.Ic0(0, 1) = -.0f;
	mani_iner.Ic0(0, 2) = 0.0141f;
	mani_iner.Ic0(1, 0) = -.0f;
	mani_iner.Ic0(1, 1) = 0.4410f;
	mani_iner.Ic0(1, 2) = -.0f;
	mani_iner.Ic0(2, 0) = 0.0141f;
	mani_iner.Ic0(2, 1) = -.0f;
	mani_iner.Ic0(2, 2) = 0.0693;
	mani_iner.Ic0 = mani_iner.Ic0 / 1000.0f;

	/*
	 *  >> mdl_arm.link1_Jc
	 *   ans =
	 *    1.0e-03 *
	 *    0.8325   -0.0339   -0.0133
	 *    -0.0339    0.6135   -0.1142
	 *    -0.0133   -0.1142    0.3319
	 *
	 * gyzhang <Jul 14, 2017>*/

	mani_iner.Ic1(0, 0) = 0.8325f;
	mani_iner.Ic1(0, 1) = -0.0339f;
	mani_iner.Ic1(0, 2) = -0.0133f;
	mani_iner.Ic1(1, 0) = -0.0339f;
	mani_iner.Ic1(1, 1) = 0.6135f;
	mani_iner.Ic1(1, 2) = -0.1142f;
	mani_iner.Ic1(2, 0) = -0.0133f;
	mani_iner.Ic1(2, 1) = -0.1142f;
	mani_iner.Ic1(2, 2) = 0.3319;
	mani_iner.Ic1 = mani_iner.Ic1 / 1000.0f;

	/*
	 * 	>> mdl_arm.link2_Jc
	 *  ans =
	 *   0.0010    0.0000         0
	 *   0.0000    0.0037   -0.0000
	 *        0   -0.0000    0.0028
	 *
	 * gyzhang <Jul 14, 2017>*/
	mani_iner.Ic2(0, 0) = 0.0010f;
	mani_iner.Ic2(0, 1) = 0.0f;
	mani_iner.Ic2(0, 2) = 0.0f;
	mani_iner.Ic2(1, 0) = 0.0f;
	mani_iner.Ic2(1, 1) = .0037f;
	mani_iner.Ic2(1, 2) = -.0f;
	mani_iner.Ic2(2, 0) = 0.0f;
	mani_iner.Ic2(2, 1) = -.0f;
	mani_iner.Ic2(2, 2) = 0.0028f;

	rb0 = Vector3f(0.0f,0.0f,0.6f);

}
ManipulatorCoMEst::~ManipulatorCoMEst() {
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

	manipulator_com_est::com_est_control = nullptr;
}
void ManipulatorCoMEst::com_estimator(){
	 float theta1 = _manipulator_joint_status.joint_posi_1;
	 float theta2 = _manipulator_joint_status.joint_posi_2;
	 float rate1 = _manipulator_joint_status.joint_rate_1;
	 float rate2 = _manipulator_joint_status.joint_rate_2;

	 Matrix4f A_0toB = transl_vector(rb0);
	 Matrix4f A_1to0 = H_transls(mani_DH.offset[0]+theta1,mani_DH.a[0],mani_DH.d[0],mani_DH.alpha[0]);
	 Matrix4f A_2to1 = H_transls(mani_DH.offset[1]+theta2,mani_DH.a[1],mani_DH.d[1],mani_DH.alpha[1]);

//	   T-matrix of joint coordinate frame
	   Matrix4f T_0toB = A_0toB;
	   Matrix4f T_1toB= T_0toB*A_1to0;
	   Matrix4f T_2toB = T_1toB * A_2to1;

//	   orthonormal rotation matrix of joint coordinate frame
	   Matrix3f R_0toB = ext_rotation(T_0toB);
	   Matrix3f R_1toB = ext_rotation(T_1toB);
	   Matrix3f R_2toB = ext_rotation(T_2toB);

//	   zi = [0 0 1]' unit vector of joint frame in B coordinate frame

	   Vector3f z3 = Vector3f(0.0f, 0.0f, 1.0f);
	   Vector3f zb_B = z3;
	   Vector3f z0_B = R_0toB*z3;
	   Vector3f z1_B = R_1toB*z3;
	   Vector3f z2_B = R_2toB*z3;

//	    origin vectors of joint frame origin in joint 0 coordinate frame
	   Vector3f ob_B = Vector3f(0.0f, 0.0f, 0.0f);
	   Vector3f o0_B = ext_translvector(T_0toB);
	   Vector3f o1_B = ext_translvector(T_1toB);
	   Vector3f o2_B = ext_translvector(T_2toB);

	   Matrix4f Trc0_B= T_0toB*transl_vector(mani_iner.rc0);
	   Matrix4f Trc1_B= T_1toB*transl_vector(mani_iner.rc1);
	   Matrix4f Trc2_B= T_2toB*transl_vector(mani_iner.rc2);

//	     links' CoG in body coordinate
//	     body coordinate is also 0th joint coordinate;
	   Vector3f rc0_B = ext_translvector(Trc0_B);
	   Vector3f rc1_B = ext_translvector(Trc1_B);
	   Vector3f rc2_B = ext_translvector(Trc2_B);

//	    system CoG in body coordinate
	   float m_sys = (mani_iner.mlink0+mani_iner.mlink1+mani_iner.mlink2+mani_iner.mb);
	   Vector3f rc_B = (mani_iner.mlink0*rc0_B+ mani_iner.mlink1*rc1_B+mani_iner.mlink2*rc2_B)/m_sys;
	   _mani_com.x = rc_B(0);
	   _mani_com.y = rc_B(1);
	   _mani_com.z = rc_B(2);
//	    %% differential of links' CoG in body coordinate

	   Vector3f dot_rc1_B = (zb_B%(rc1_B - ob_B))*rate1;
	   Vector3f dot_rc2_B= (zb_B%(rc2_B - ob_B))*rate1 + (z1_B%(rc2_B - o1_B))*rate2;
//
//	    % differential of system CoG in body coordinate
	   Vector3f dotrc_B = (mani_iner.mlink1*dot_rc1_B+mani_iner.mlink2*dot_rc2_B)/m_sys;
	   _mani_com.vx = dotrc_B(0);
	   _mani_com.vy = dotrc_B(1);
	   _mani_com.vz = dotrc_B(2);

//	%     % increment of inertia matrix
	   Matrix3f I_delta =R_0toB*mani_iner.Ic0*R_0toB.transpose() + R_1toB*mani_iner.Ic1*R_1toB.transpose() +R_2toB*mani_iner.Ic2*R_2toB.transpose()+
	                   - mani_iner.mlink0*rc0_B.hat()*rc0_B.hat() - mani_iner.mlink1*rc1_B.hat()*rc1_B.hat() - mani_iner.mlink2*rc2_B.hat()*rc2_B.hat();
}

void ManipulatorCoMEst::task_main() {
	_sub_manipulator_joint_status = orb_subscribe(
			ORB_ID(manipulator_joint_status));
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _sub_manipulator_joint_status;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 80);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("manipulator_com_est: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(manipulator_joint_status), _sub_manipulator_joint_status, &_manipulator_joint_status);

			com_estimator();

			if (_pub_mani_com == nullptr) {
				_pub_mani_com = orb_advertise(ORB_ID(mani_com), &_mani_com);
			} else {
				orb_publish(ORB_ID(mani_com), _pub_mani_com, &_mani_com);
			}

			 PX4_INFO("mani_com_est: %8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f",
			         (double)_mani_com.x,
			         (double)_mani_com.y,
			         (double)_mani_com.z,
			         (double)_mani_com.vx,
			         (double)_mani_com.vy,
			         (double)_mani_com.vz );
		}
	}

}

void ManipulatorCoMEst::task_main_trampoline(int argc, char *argv[]) {
	manipulator_com_est::com_est_control->task_main();
}
int ManipulatorCoMEst::start() {
	/* start the task */
	_control_task = px4_task_spawn_cmd("manipulator_com_est", SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 2, 5000,
			(px4_main_t) &ManipulatorCoMEst::task_main_trampoline, nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int manipulator_com_est_main(int argc, char *argv[]) {
	if (argc < 2) {
		warnx("usage: manipulator_com_est {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (manipulator_com_est::com_est_control != nullptr) {
			warnx("already running");
			return 1;
		}

		manipulator_com_est::com_est_control = new ManipulatorCoMEst;

		if (manipulator_com_est::com_est_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != manipulator_com_est::com_est_control->start()) {
			delete manipulator_com_est::com_est_control;
			manipulator_com_est::com_est_control = nullptr;
			warnx("start failed");
			return 1;
		}
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (manipulator_com_est::com_est_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete manipulator_com_est::com_est_control;
		manipulator_com_est::com_est_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (manipulator_com_est::com_est_control) {
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

