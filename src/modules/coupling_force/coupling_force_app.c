/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file coupling_force_app.c
 *   Created on: 2016-1-21
 *      Author: gyzhang
 * aerial manipulator coupling force sensor driver for PX4 autopilot
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>


#include <termios.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include<fcntl.h>
#include<modules/coupling_force/data_process.h>
#include <uORB/topics/coupling_force.h>

static bool thread_should_exit = false;		/**< coupling_force_app exit flag */
static bool thread_running = false;		/**< coupling_force_app status flag */
static int daemon_task;				/**< Handle of coupling_force_app task / thread */

/**
 * management function.
 */
__EXPORT int coupling_force_app_main(int argc, char *argv[]);

/**
 * Mainloop
 */
int coupling_force_app_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * uart initial.
 */
static int uart_init(char * uart_name);


static int set_uart_baudrate(const int fd, unsigned int baud);


static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

/**
 * coupling_force
 */
int coupling_force_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("coupling_force_app already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("coupling_force_app",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT,
					     2000,
					     coupling_force_app_thread_main,
					     (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}





int coupling_force_app_thread_main(int argc, char *argv[])
{
	thread_running = true;
	warnx("initializing!\n");
    char out_buffer[50];
    char byte_data = '0';
    char in_buffer[64] = "";
    float coupling_force_buff[8];
    int index=0;
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init("/dev/ttyS6");

    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,115200))
    {
        printf("[YCM]set_uart_baudrate is failed\n");
        return -1;
    }

    sleep(10);
    sprintf(out_buffer,"AT+SMPR=200\r\n");
    usleep(100);
    write(uart_read,&out_buffer,strlen(out_buffer));

    sprintf(out_buffer,"AT+GSD\r\n");
    write(uart_read,&out_buffer,strlen(out_buffer));
    usleep(10);

    warnx("initialize successfully!!\n");
    usleep(100000);
    warnx("task start \n");

    struct coupling_force_s coup_force;
    memset(&coup_force, 0, sizeof(coup_force));
    orb_advert_t coup_force_pub_fd = orb_advertise(ORB_ID(coupling_force), &coup_force);

    while(!thread_should_exit){

        read(uart_read,&byte_data,1);
        in_buffer[index] = byte_data;
        if (index ==2)
        {
        	printf("Data_byte: 0x %x\n ",in_buffer[1]);
        }

        if ((index == 0)&&(in_buffer[index] == 0xAA))
        		index =1;
        else if ((index == 1)&&(in_buffer[index] == 0x55))
		        index =2;
        else if ((index >=2)&&(index <19))
		        index++;
        else if (index ==19)
        {
        	index = 0;
        	force_sensor_data_decode(coupling_force_buff,in_buffer);

        	coup_force.force_x = coupling_force_buff[0];
        	coup_force.force_y = coupling_force_buff[1];
        	coup_force.force_z = coupling_force_buff[2];
        	coup_force.moment_x = coupling_force_buff[3];
        	coup_force.moment_y = coupling_force_buff[4];
        	coup_force.moment_z = coupling_force_buff[5];
        	orb_publish(ORB_ID(coupling_force),coup_force_pub_fd, &coup_force);
        }
        else
        {
        	index = 0;
        }

    }
    warnx("exiting.\n");
    thread_running = false;
    return 0;
}
