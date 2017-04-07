/*
 * BlockManipulatorControl.h
 *
 *  Created on: 7 Nov 2016
 *      Author: bdai <bdai1412@gmailcom>
 */
#pragma once

#include <px4_posix.h>
#include <controllib/blocks.hpp>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>
#include <matrix/Matrix.hpp>

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/target_info.h>
#include <uORB/topics/endeff_frame_status.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/target_endeff_frame.h>

using namespace matrix;
using namespace control;

#define print_info(print, ...) {\
	if (print) {\
		mavlink_and_console_log_info(__VA_ARGS__);\
	}\
}

static Vector3f MANI_OFFSET(-0.021f, 0.001f, 0.193f);
static Vector3f MANI_FIRST_JOINT(0, 0, 0.109);

class BlockManipulatorControl : public control::SuperBlock
{

public:

	BlockManipulatorControl();

	virtual ~BlockManipulatorControl();

	void control();

private:

	//prevent copy and assignment
	BlockManipulatorControl(const BlockManipulatorControl &);
	BlockManipulatorControl operator=(const BlockManipulatorControl &);

	//methods
	void mani_init_position();

	//Subscriptions
	uORB::Subscription<vehicle_local_position_s> _pos_sub;
	uORB::Subscription<vehicle_attitude_s> _att_sub;
	uORB::Subscription<manual_control_setpoint_s> _manual_sub;
	uORB::Subscription<target_info_s> _target_sub;
	uORB::Subscription<endeff_frame_status_s> _mani_status_sub;

	//Publications
	uORB::Publication<target_endeff_frame_s> _manip_pub;

	px4_pollfd_struct_t _polls[1];

	// Performance counters
	perf_counter_t _err_perf;

	uint64_t _timeStamp;
	uint64_t _timePrint;

	bool _mani_triggered;

	uint8_t _in_range;

	bool _relative_rest;
	uint64_t _relative_rest_time;

	uint64_t _last_info_time;

	bool _grabbed;


};



