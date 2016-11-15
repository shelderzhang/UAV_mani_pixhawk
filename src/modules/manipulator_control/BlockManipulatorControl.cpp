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
static float MANI_RANGE[6] = {0.0f, 0.5f, -0.2f, 0.2f, 0.1f, 0.5f};
enum {X_MIN = 0, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX};

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

//	usleep(100000);
	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
//	_timeStamp = newTimeStamp;

//	bool print_info = false;
//	if (newTimeStamp - _last_info_time > 500000){
//		print_info = true;
//		_last_info_time = newTimeStamp;
//	}

	setDt(dt);

	// get new data
	updateSubscriptions();


//	// set period about 0.2s
//	if (getDt() < 0.5f){
//		return;
//	}

	_timeStamp = newTimeStamp;
	bool enableMani = (_manual_sub.get().flaps > 0.75f);
	enableMani = true;
	if (enableMani) {

		Vector3f target_pos(_target_sub.get().x, _target_sub.get().y, _target_sub.get().z);

		warnx("target x:%8.4f, y:%8.4f, z:%8.4f",
			(double)target_pos(0),
			(double)target_pos(1),
			(double)target_pos(2));

		//		Vector3f target_pos = Vector3f(0.38, 0.0, -0.65) + MANI_OFFSET;

		//		Vector3f target_vel(_target_sub.get().vx, _target_sub.get().vy, _target_sub.get().vz);

		Vector3f pos(_pos_sub.get().x, _pos_sub.get().y, _pos_sub.get().z);

//		Vector3f pos(0.0f, 0.0f, -1.0f);

		Matrix3f R_att(_att_sub.get().R);

		Vector3f distance = R_att.transpose() * (target_pos - pos) - MANI_OFFSET;

//		if (print_info) {
//			warnx("distance x:%8.4f, y:%8.4f, z:%8.4f",
//				(double)distance(0),
//				(double)distance(1),
//				(double)distance(2));
//		}

		_in_range = 0;
		if (distance(0) < MANI_RANGE[X_MIN]) {
			_manip_pub.get().x = MANI_RANGE[X_MIN];
		}
		else if (distance(0) > MANI_RANGE[X_MAX]) {
			_manip_pub.get().x = MANI_RANGE[X_MAX];
		} else {
			_manip_pub.get().x = distance(0);
			_in_range |= 1;
		}

		if (distance(1) < MANI_RANGE[Y_MIN]) {
			_manip_pub.get().y = MANI_RANGE[Y_MIN];
		}
		else if(distance(1) > MANI_RANGE[Y_MAX]) {
			_manip_pub.get().y = MANI_RANGE[Y_MAX];
		} else {
			_manip_pub.get().y = distance(1);
			_in_range |= 1<<1;
		}
		if (distance(2) < MANI_RANGE[Z_MIN]) {
			_manip_pub.get().z = MANI_RANGE[Z_MIN];
		} else if (distance(2) > MANI_RANGE[Z_MAX]) {
			_manip_pub.get().z = MANI_RANGE[Z_MAX];
		} else {
			_manip_pub.get().z = distance(2);
			_in_range |= 1<<2;
		}

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









