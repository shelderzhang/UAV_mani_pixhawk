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
#include <uORB/uORB.h>
// uORB Suscriptions
#include <uORB/topics/manipulator_joint_status.h>

// uORB Publications
#include <uORB/topics/mani_com.h>




using namespace matrix;
using namespace control;

extern orb_advert_t mavlink_log_pub;


struct DH_params_s{
	float d[2];
	float a[2];
	float alpha[2];
	float offset[2];
};

struct inertia_params_s{
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
class ManipulatorCoMEst : public control::SuperBlock {
	ManipulatorCoMEst();
	~ManipulatorCoMEst();

	    int start();

	private:
	    bool _task_should_exit;
	    int _control_task;

	    int _sub_manipulator_joint_status;
	    orb_advert_t    _pub_mani_com;

	    struct manipulator_joint_status_s         __sub_manipulator_joint_status;
	    struct mani_com_s        _mani_com;

	    struct DH_params_s mani_DH;
	    struct inertia_params_s mani_iner;

	    px4_pollfd_struct_t _polls;


	    /**
		 * Update our local parameter cache.
		 */
		void	com_estimator();



	    /**
		 * Shim for calling task_main from task_create.
		 */
		static void	task_main_trampoline(int argc, char *argv[]);

		/**
		 * Main attitude control task.
		 */
		void		task_main();

}

ManipulatorCoMEst::ManipulatorCoMEst():
		_task_should_exit(false),
		_control_task(-1),

		/* subscriptions */
		_sub_manipulator_joint_status(-1),

/* publications */
_pub_mani_com(nullptr)

{
	memset(&manipulator_joint_status_s, 0, sizeof(manipulator_joint_status_s));
	memset(&manipulator_joint_status_s, 0, sizeof(manipulator_joint_status_s));
	mani_DH.d[0]=0.167;
	mani_DH.d[1]=0.0;
	mani_DH.a[0]=0.0;
	mani_DH.a[1]=0.0;

};
