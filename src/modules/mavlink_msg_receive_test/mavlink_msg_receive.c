/**
 * @file mavlink_msg_receive.c
 * test for my codes -- jom
 * 2015/5/13
 * note:
 * ������������orb_set_inteval(sensor_sub_fd, 1000)��poll_ret = poll(fds, 1, 1000)���׻���.
 * orb_set_interval���ô������ݸ��±�־λ�ĸ�����Сʱ����, �ڼ���ܴ�����ݸ����˶��, ������ݸ��±�־λ������Ϊ��Сʱ��
 * ���û�ﵽ��û��λ.
 * poll��ݴ������ݸ��±�־λ��ȷ������Ƿ������.
 */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/coupling_force.h>

#include <nuttx/config.h>

#include <stdlib.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>


static bool thread_should_exit = false;		/**< mavlink_msg_receive exit flag */
static bool thread_running = false;			/**< mavlink_msg_receive status flag */
static int  mavlink_msg_receive_task;				/**< Handle of mavlink_msg_receive task / thread */

// �̹߳������
__EXPORT int mavlink_msg_receive_main(int argc, char *argv[]);

/**
 * Mainloop of mavlink_msg_receive
 * �û��߳�, ִ���û�����
 */
int mavlink_msg_receive_thread_main(int argc, char *argv[]);

/**
 * print the correct usage for mavlink_msg_receive operating
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if(reason)
		warnx("%s\n", reason);
	errx(1,"usage: mavlink_msg_receive {start|stop|status} [-p <additional params>]\n\n");
}
/**
 * mavlink_msg_receiveģ���ǰ̨�������, ��shell����, ���Կ���ģ����̵߳�������ֹͣ.
 */
int mavlink_msg_receive_main(int argc, char *argv[])
{
	if (argc < 1)
			usage("missing command");

		if (!strcmp(argv[1], "start")) {   //shell��������

			if (thread_running) {		   // ����߳��Ѿ�������
				warnx("mavlink_msg_receive already running\n");
				/* this is not an error */
				exit(0);
			}
			thread_should_exit = false;		// ���߳�״̬λ����Ϊfalse
			// �����߳�, ucos�ǳ���ͬ.
			mavlink_msg_receive_task = px4_task_spawn_cmd("mavlink_msg_receive",				// �߳���
											SCHED_DEFAULT,					// ����ģʽ, ��ͬ���ȼ��������Ƚ��ȳ�(FIFO),��ͬ���ȼ����������ȼ�
											SCHED_PRIORITY_DEFAULT,			// ���ȼ�
											1200,							// ��ջ��С
											mavlink_msg_receive_thread_main,			// �߳����
											(argv) ? (char * const *)&argv[2] : (char * const *)NULL // �̲߳���
											);
			exit(0);						// ���˳�
		}
		if (!strcmp(argv[1], "stop")) {		// shellֹͣ����
			thread_should_exit = true;
			exit(0);
		}

		if (!strcmp(argv[1], "status")) {	// shell��ѯ����, ���ڲ�ѯ�̵߳�״̬.
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

/**
 * mavlink_msg_receive�ĺ�̨�߳�, ����ִ���û�����
 */
int mavlink_msg_receive_thread_main(int argc, char *argv[])
{

	int coupl_force_sub_fd = orb_subscribe(ORB_ID(coupling_force));
	int mani_status_sub_fd = orb_subscribe(ORB_ID(manipulator_status));// ���Ĵ��������
	int vision_pos_ctl_sub_fd = orb_subscribe(ORB_ID(vision_pos_ctl));// ���Ĵ��������
	orb_set_interval(sensor_sub_fd, 1000);						// ����topic(bus)��ݶ�ȡ��Сʱ����
	orb_set_interval(coupl_force_sub_fd, 1000);
	orb_set_interval(mani_status_sub_fd, 1000);// �����û��1s, ��ô��Ϊ��ݻ��Ǿɵ�, ��ʱ����̵߳���poll�����µȴ�1s
	orb_set_interval(vision_pos_ctl_sub_fd, 1000);

	struct pollfd fds[4] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = coupl_force_sub_fd,   .events = POLLIN },
		{ .fd = mani_status_sub_fd,   .events = POLLIN },
		{ .fd = vision_pos_ctl_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;

	thread_running = true;
	while(!thread_should_exit)				// ����߳�û�б�ֹͣ
	{
		int poll_ret = poll(fds, 4, 1000);	// �̵߳ȴ���ݸ���, timeoutΪ1s,
											// ������û����, ��ô��ʱϵͳ����������л�
		if (poll_ret == 0)					// û����ݸ���
		{
			printf("[mavlink_msg_receive] Got no data within a second\n");
		}
		else if (poll_ret < 0)				// ���ִ���
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
				struct manipulator_sensor_s data;
				orb_copy(ORB_ID(manipulator_sensor), sensor_sub_fd, &data);	// �������
				// ����������shell��

				printf("[mavlink_msg_receive] manipulator_sensor:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",
					(double)data.force_x,
					(double)data.force_y,
					(double)data.force_z,
					(double)data.moment_x,
					(double)data.moment_y,
					(double)data.moment_z);
			}

			if (fds[1].revents & POLLIN)
			{
				struct coupling_force_s data;
				orb_copy(ORB_ID(coupling_force), coupl_force_sub_fd, &data);	// �������
				// ����������shell��

				printf("[mavlink_msg_receive] coupling_force:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",
					(double)data.force_x,
					(double)data.force_y,
					(double)data.force_z,
					(double)data.moment_x,
					(double)data.moment_y,
					(double)data.moment_z);
			}
			if (fds[2].revents & POLLIN)
			{
				struct manipulator_status_s data;
				orb_copy(ORB_ID(manipulator_status), mani_status_sub_fd, &data);	// �������
				// ����������shell��

				printf("[mavlink_msg_receive] mani_status:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",

					(double)data.theta_1,
					(double)data.theta_2,
					(double)data.theta_3,
					(double)data.thetadot_1,
					(double)data.thetadot_2,
					(double)data.thetadot_3);
			}
			if (fds[3].revents & POLLIN)
			{
				struct vision_pos_ctl_s data;
				orb_copy(ORB_ID(vision_pos_ctl), vision_pos_ctl_sub_fd, &data);	// �������
				// ����������shell��

				printf("[mavlink_msg_receive] vision_pos_ctl:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",

						(double)data.current_pos_x,
						(double)data.current_pos_y,
						(double)data.current_pos_z,
						(double)data.current_speed_x,
						(double)data.current_speed_y,
						(double)data.current_speed_y,
						(double)data.pos_sp_x,
						(double)data.pos_sp_y,
						(double)data.pos_sp_z,
						(double)data.sp_yaw);

			}

		}
	}
	thread_running = false;
	return 0;
}
