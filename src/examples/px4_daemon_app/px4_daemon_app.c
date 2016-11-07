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
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/att_pos_mocap.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int px4_daemon_app_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);

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

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int px4_daemon_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 px4_daemon_thread_main,
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

int px4_daemon_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");

	thread_running = true;
	orb_advert_t _att_pos_mocap_pub = NULL;
//
//	float MAXNUM = 100;
//	float index = 0;

	hrt_abstime pre = 0;
	while (!thread_should_exit) {
//		warnx("Hello daemon!\n");
//		sleep(10);
		struct att_pos_mocap_s att_pos_mocap = {};

		// Use the component ID to identify the mocap system
		att_pos_mocap.id = 1;

		hrt_abstime now = hrt_absolute_time();
		if (now - pre > 20000) {
			pre = now;
			att_pos_mocap.timestamp = now;
			att_pos_mocap.timestamp_received = now;

			att_pos_mocap.q[0] = 1;
			att_pos_mocap.q[1] = 0;
			att_pos_mocap.q[2] = 0;
			att_pos_mocap.q[3] = 0;

	//		float value = ++index / MAXNUM;
	//
	//		if (index > MAXNUM) {
	//			index = 0;
	//		}

			att_pos_mocap.x = 1;
			att_pos_mocap.y = 3;
			att_pos_mocap.z = -3;

			if (_att_pos_mocap_pub == NULL) {
				_att_pos_mocap_pub = orb_advertise(ORB_ID(att_pos_mocap), &att_pos_mocap);

			} else {
				orb_publish(ORB_ID(att_pos_mocap), _att_pos_mocap_pub, &att_pos_mocap);
			}
		} else {
			usleep(1000);
		}
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
