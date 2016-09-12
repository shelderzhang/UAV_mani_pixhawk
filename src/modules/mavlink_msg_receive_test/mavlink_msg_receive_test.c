/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file mavlink_msg_receive_test.c
 * mavlink_msg_receive application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <stdlib.h>
#include <nuttx/sched.h>

#include <uORB/uORB.h>
#include <uORB/topics/manipulator_joint_status.h>
#include <uORB/topics/target_endeff_frame.h>
#include <uORB/topics/endeff_frame_status.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit = false;		/**< mavlink_msg_receive exit flag */
static bool thread_running = false;			/**< mavlink_msg_receive status flag */
static int mavlink_msg_receive_task;			/**< Handle of mavlink_msg_receive task / thread */

/**
 * mavlink_msg_receive management function.
 */
__EXPORT int mavlink_msg_receive_test_main(int argc, char *argv[]);

/**
 * Mainloop of mavlink_msg_receive.
 */
int mavlink_msg_receive_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: mavlink_msg_receive {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The mavlink_msg_receive app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mavlink_msg_receive_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("mavlink_msg_receive already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		mavlink_msg_receive_task = px4_task_spawn_cmd("mavlink_msg_receive",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				2000,
				mavlink_msg_receive_thread_main,
				(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int mavlink_msg_receive_thread_main(int argc, char *argv[])
{

	warnx("[mavlink_msg_receive] starting\n");

	thread_running = true;

	/* subscribe to sensor_combined topic */
	int target_endeff_frame_sub_fd = orb_subscribe(ORB_ID(target_endeff_frame));


	int manipulator_joint_status_sub_fd = orb_subscribe(ORB_ID(manipulator_joint_status));
	int endeff_frame_status_sub_fd = orb_subscribe(ORB_ID(endeff_frame_status));
	orb_set_interval(target_endeff_frame_sub_fd, 100);
	orb_set_interval(manipulator_joint_status_sub_fd, 100);
	orb_set_interval(endeff_frame_status_sub_fd, 100);

	struct pollfd fds[3] = {
			{ .fd = target_endeff_frame_sub_fd,   .events = POLLIN },
			{ .fd = manipulator_joint_status_sub_fd,   .events = POLLIN },
			{ .fd = endeff_frame_status_sub_fd,   .events = POLLIN },
	};
	int error_counter = 0;


	while (!thread_should_exit) {
		int poll_ret = px4_poll(fds, 3, 1000);
		if (poll_ret == 0)
		{
			printf("[mavlink_msg_receive] Got no data within one second\n");
		}
		else if (poll_ret < 0)
		{
			if (error_counter < 10 || error_counter % 50 == 0)
			{
				printf("[mavlink_msg_receive] ERROR return value from poll(): %d\n", poll_ret);
			}
			error_counter++;
		}
		else
		{

			if (fds[0].revents & POLLIN)
			{
				struct target_endeff_frame_s data;
				orb_copy(ORB_ID(target_endeff_frame), target_endeff_frame_sub_fd, &data);
				PX4_WARN("[mavlink_msg_receive]X Y Z Position:\n\t%8.4f %8.4f %8.4f\n", (double)data.x, (double)data.y, (double)data.z);
				PX4_WARN("[mavlink_msg_receive]setpoint in rad:\n\t%8.4f %8.4f %8.4f\n", (double)data.roll, (double)data.pitch, (double)data.yaw);
				PX4_WARN("[mavlink_msg_receive]velocity in NED frame in meter / s :\n\t%8.4f %8.4f %8.4f\n", (double)data.vx, (double)data.vy, (double)data.vz);
				PX4_WARN("[mavlink_msg_receive]rate setpoint in rad/s:\n\t%8.4f %8.4f %8.4f\n", (double)data.roll_rate, (double)data.pitch_rate, (double)data.yaw_rate);
				PX4_WARN("[mavlink_msg_receive]enable robotic arm:%8.4f", (double)data.arm_enable);

			}

			if (fds[1].revents & POLLIN)
			{
				struct manipulator_joint_status_s data;
				orb_copy(ORB_ID(manipulator_joint_status), manipulator_joint_status_sub_fd, &data);
				PX4_WARN("[mavlink_msg_receive]Position of joints in pi:\n\t%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n", (double)data.joint_posi_1, (double)data.joint_posi_2, (double)data.joint_posi_3, (double)data.joint_posi_4, (double)data.joint_posi_5, (double)data.joint_posi_6, (double)data.joint_posi_7);
				PX4_WARN("[mavlink_msg_receive]speed of joints in pi/s:\n\t%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n", (double)data.joint_rate_1, (double)data.joint_rate_2, (double)data.joint_rate_3, (double)data.joint_rate_4, (double)data.joint_rate_5, (double)data.joint_rate_6, (double)data.joint_rate_7);
				PX4_WARN("[mavlink_msg_receive]torque of joints in N*m:\n\t%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n", (double)data.torque_1, (double)data.torque_2, (double)data.torque_3, (double)data.torque_4, (double)data.torque_5, (double)data.torque_6, (double)data.torque_7);

			}
			if (fds[2].revents & POLLIN)
			{
				struct endeff_frame_status_s data;
				orb_copy(ORB_ID(endeff_frame_status), endeff_frame_status_sub_fd, &data);
				PX4_WARN("[mavlink_msg_receive]X Y Z Position:\n\t%8.4f %8.4f %8.4f\n", (double)data.x, (double)data.y, (double)data.z);
				PX4_WARN("[mavlink_msg_receive]setpoint in rad:\n\t%8.4f %8.4f %8.4f\n", (double)data.roll, (double)data.pitch, (double)data.yaw);
				PX4_WARN("[mavlink_msg_receive]velocity in NED frame in meter / s :\n\t%8.4f %8.4f %8.4f\n", (double)data.vx, (double)data.vy, (double)data.vz);
				PX4_WARN("[mavlink_msg_receive]rate setpoint in rad/s:\n\t%8.4f %8.4f %8.4f\n", (double)data.roll_rate, (double)data.pitch_rate, (double)data.yaw_rate);
				PX4_WARN("[mavlink_msg_receive]enable robotic arm:%8.4f", (double)data.arm_enable);
				PX4_WARN("[mavlink_msg_receive]gripper_status:%8.4f", (double)data.gripper_status);
				PX4_WARN("[mavlink_msg_receive]gripper_position:%8.4f", (double)data.gripper_posi);
						}
		}

	}

	    warnx("[mavlink_msg_receive] exiting.\n");
		thread_running = false;

		return 0;
	}


