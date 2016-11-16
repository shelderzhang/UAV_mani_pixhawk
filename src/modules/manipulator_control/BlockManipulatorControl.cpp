/*
 * BlockManipulatorControl.cpp
 *
 *  Created on: 7 Nov 2016
 *      Author: bdai <bdai1412@gmailcom>
 */

#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <fcntl.h>
#include <matrix/math.hpp>

#include "BlockManipulatorControl.hpp"

static Vector3f MANI_OFFSET(-0.0183f, 0.0003f, 0.1396f);
static Vector3f MANI_FIRST_JOINT(0, 0, 0.109);

static float MANI_RANGE[6] = {0.3f, 0.45f,
		-10.0f / 180.0f * (float)M_PI, 40.0f / 180.0f * (float)M_PI,
		-40.0f / 180.0f * (float)M_PI, 40.0f / 180.0f * (float)M_PI};
enum {R_MIN = 0, R_MAX, THETA_MIN, THETA_MAX, PSI_MIN, PSI_MAX};


/*relative rest duration before start grabbing 0.5s -bdai<13 Nov 2016>*/
static uint32_t REST_DURATION = 500000;	// 0.5 s
static float ACCURACY = 0.02f;	//grab demand 0.05m accuracy


BlockManipulatorControl::BlockManipulatorControl():
	//this block has no parent, and has name MANC
	SuperBlock(NULL, "MANC"),
	// subscriptions, set rate, add to list
	_pos_sub(ORB_ID(vehicle_local_position), 1000 / 50, 0, &getSubscriptions()),
	_att_sub(ORB_ID(vehicle_attitude), 1000 / 50, 0, &getSubscriptions()),
	_manual_sub(ORB_ID(manual_control_setpoint), 1000 / 20, 0, &getSubscriptions()),
	_target_sub(ORB_ID(target_info), 1000/50, 0, &getSubscriptions()),
	_mani_status_sub(ORB_ID(endeff_frame_status), 1000 / 50 , 0, &getSubscriptions()),
	// publications
	_manip_pub(ORB_ID(target_endeff_frame), -1, &getPublications()),
	_polls(),
	// Loop performance
	_err_perf(),
	_timeStamp(),
	_timePrint(),
	_mani_triggered(false),
	_in_range(0),
	_relative_rest(false),
	_relative_rest_time(),
	_last_info_time(0)
{
	_polls[0].fd = _target_sub.getHandle();
	_polls[0].events = POLLIN;

	// initialize paramet3ers dependent matrices
//	updateParams();
}

BlockManipulatorControl::~BlockManipulatorControl(){

}

void BlockManipulatorControl::control() {

	//wait for local position update
	int ret = px4_poll(_polls, 1, 100);

	if (ret < 0)
	{
		perf_count(_err_perf);
		warnx("error");
		return;
	}

	if (ret == 0) {
		warnx("tiem out");
		return;
	}

	_timeStamp = hrt_absolute_time();
	float dt = (_timeStamp - _timePrint) / 1.0e6f;

	setDt(dt);

	// get new data
	updateSubscriptions();

	bool enableMani = (_manual_sub.get().flaps > 0.75f);
	enableMani = true;
	if (enableMani) {
		Vector3f target_pos(_target_sub.get().x, _target_sub.get().y, _target_sub.get().z);

//		warnx("target x:%8.4f, y:%8.4f, z:%8.4f",
//			(double)target_pos(0),
//			(double)target_pos(1),
//			(double)target_pos(2));

		//		Vector3f target_pos = Vector3f(0.38, 0.0, -0.65) + MANI_OFFSET;
		//		Vector3f target_vel(_target_sub.get().vx, _target_sub.get().vy, _target_sub.get().vz);

		Vector3f pos(_pos_sub.get().x, _pos_sub.get().y, _pos_sub.get().z);
//		Vector3f pos(0.0f, 0.0f, -1.0f);

		Matrix3f R_att(_att_sub.get().R);
		Vector3f distance = R_att.transpose() * (target_pos - pos) - MANI_OFFSET - MANI_FIRST_JOINT;

		Vector3f mani_sp(distance);
//		if (print_info) {
//			warnx("distance x:%8.4f, y:%8.4f, z:%8.4f",
//				(double)distance(0),
//				(double)distance(1),
//				(double)distance(2));
//		}

		_in_range = 0;
		/*when target is in range between two  -bdai<16 Nov 2016>*/
		float distance_norm = distance.norm();
		if (distance_norm < MANI_RANGE[R_MIN]) {
			mani_sp = MANI_RANGE[R_MIN] * mani_sp.normalized();
		} else if (distance_norm > MANI_RANGE[R_MAX]) {
			mani_sp = MANI_RANGE[R_MAX] * mani_sp.normalized();
		}
		else {
			_in_range |= 1;
		}

		Vector3f projection_normlized = Vector3f(mani_sp(0), mani_sp(1), 0.0f).normalized();
		float mani_sp_norm = mani_sp.norm();

//		float sin_theta = (Vector3f(mani_sp(0), mani_sp(1), 0).normalized() % mani_sp.normalized()).norm();
		Vector3f mani_sp_nomolized = mani_sp.normalized();
		float sin_theta = mani_sp_nomolized(2);
		if ( sin_theta < sinf(MANI_RANGE[THETA_MIN])) {
			Vector3f r = (projection_normlized % mani_sp).normalized();
			Quatf q;
			q.from_axis_angle(r, THETA_MIN);
			Dcmf R(q);
			mani_sp = R * projection_normlized * mani_sp_norm;
		} else if(sin_theta > sinf(MANI_RANGE[THETA_MAX])) {
			Vector3f r = (projection_normlized % mani_sp).normalized();
			Quatf q;
			q.from_axis_angle(r, THETA_MAX);
			Dcmf R(q);
			mani_sp = R * projection_normlized * mani_sp_norm;
		} else {
			_in_range |= 1<<1;
		}

		float sin_psi = projection_normlized(1);
		mani_sp_norm = mani_sp.norm();
		Vector3f projection = Vector3f(sqrt(mani_sp_norm*mani_sp_norm - mani_sp(2)*mani_sp(2)), 0, mani_sp(2));

		if ( sin_psi < sinf(MANI_RANGE[PSI_MIN])) {
			Quatf q(cosf(MANI_RANGE[PSI_MIN] / 2.0f), 0.0f, 0.0f, sinf(MANI_RANGE[PSI_MIN] / 2.0f));
			Dcmf R(q);
			mani_sp = R * projection;
		} else if(sin_theta > sinf(MANI_RANGE[PSI_MIN])) {
			Quatf q(cosf(MANI_RANGE[PSI_MIN] / 2.0f), 0.0f, 0.0f, sinf(MANI_RANGE[PSI_MIN] / 2.0f));
			Dcmf R(q);
			mani_sp = R * projection;
		} else {
			_in_range |= 1<<2;
		}

		mani_sp = mani_sp + MANI_FIRST_JOINT;

//		if (print_info) {
//			warnx("_in_range: %d", _in_range);
//		}
		if (_in_range == 7){
			if (!_mani_triggered){
				_mani_triggered = true;	/*means manipulator can move now -bdai<13 Nov 2016>*/
			}

			float mani_error = Vector3f(_manip_pub.get().x - _mani_status_sub.get().x,
					_manip_pub.get().y - _mani_status_sub.get().y,
					_manip_pub.get().z - _mani_status_sub.get().z).norm();
//			mani_error = 0.01f;

			if(!_relative_rest) {
				if (mani_error < ACCURACY)
				{
					_relative_rest_time = _timeStamp;
					_relative_rest = true;
				} else {
				_relative_rest = false;
				}
			}
		} else {
			_relative_rest = false;
			_relative_rest_time = _timeStamp;
		}

		/*hold on in init position -bdai<13 Nov 2016>*/
		if (!_mani_triggered) {
			_manip_pub.get().timestamp = _timeStamp;
			_manip_pub.get().x = .0f;
			_manip_pub.get().y = .0f;
			_manip_pub.get().z = .0f;
		}

//		if (print_info) {
//			warnx("_relative_rest_time %llu, duration %llu", _relative_rest_time, _timeStamp - _relative_rest_time);
//		}
		if (_relative_rest && (_timeStamp - _relative_rest_time) > REST_DURATION)
		{
			_manip_pub.get().arm_enable = 1;
		} else {
//			_manip_pub.get().arm_enable = 0;
		}

		//grab success
		if (_mani_status_sub.get().gripper_status == -1) {
			_manip_pub.get().z = _manip_pub.get().z - 0.1f;
		}

		_manip_pub.get().timestamp = _timeStamp;
		_manip_pub.update();
	} else {
		_manip_pub.get().timestamp = _timeStamp;
		_manip_pub.get().x = .0f;
		_manip_pub.get().y = .0f;
		_manip_pub.get().z = .0f;
		_manip_pub.get().arm_enable = 0;
		_manip_pub.get().timestamp = _timeStamp;
		_manip_pub.update();
	}
//	if (print_info) {
//		warnx("enableMani %d, x %8.4f, y %8.4f, z %8.4f, arm_enable %d",
//				enableMani,
//				(double)_manip_pub.get().x,
//				(double)_manip_pub.get().y,
//				(double)_manip_pub.get().z,
//				_manip_pub.get().arm_enable);
//	}
}









