/*
 * manipulator_control_main.cpp
 *
 *  Created on: 1 Nov 2016
 *      Author: bdai <bdai1412@gmailcom>
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <fcntl.h>
#include <px4_posix.h>


#include "BlockManipulatorControl.hpp"

static volatile bool thread_should_exit = false;	// Deamon exit flag
static volatile bool thread_running = false;		// Deamon status flag
static int deamon_task;		// Handle of deamon task / thread

/**
 * Deamon management function.
 */
extern "C" __EXPORT int manipulator_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon
 */
int manipulator_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static int usage(const char *reason);

static int usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: manipulator_control {start|stop|status} [-p <additional params>]\n\n");
	return 1;
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */

int manipulator_control_main(int argc, char *argv[])
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

		deamon_task = px4_task_spawn_cmd("manipulator_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 3,
						 2000,
						 manipulator_control_thread_main,
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

int manipulator_control_thread_main(int argc, char *argv[])
{
	PX4_DEBUG("starting");

//	using namespace control;

	BlockManipulatorControl mani;

	thread_running = true;

	while (!thread_should_exit) {
		mani.control();
	}

	PX4_DEBUG("exiting.");

	thread_running = false;

	return 0;
}




