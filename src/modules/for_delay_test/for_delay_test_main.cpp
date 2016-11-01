/**
 * @file for_delay_test_main.cpp
 * For 动补平台 --- LiuZhong
 * 2016/11/1
 */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <uORB/uORB.h>
#include <nuttx/config.h>
#include <stdlib.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <px4_posix.h>

#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/camera_trigger.h>

static bool thread_should_exit = false;		/**< for_delay_test exit flag */
static bool thread_running = false;			/**< for_delay_test status flag */
static int  for_delay_test_task;				/**< Handle of for_delay_test task / thread */

// 线程管理程序
extern "C" __EXPORT int for_delay_test_main(int argc, char *argv[]);
// 用户线程, 执行用户代码
int for_delay_test_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
static void
usage(const char *reason)
{

}

// 线程管理程序
int for_delay_test_main(int argc, char *argv[])
{
	if (argc < 2) {
			warnx("usage: for_delay_test {start|stop|status}");
			return 1;
		}

	if (!strcmp(argv[1], "start")) {   //shell启动命令
		if (thread_running) {		   // 如果线程已经启动了
			warnx("for_delay_test already running\n");
			/* this is not an error */
			exit(0);
		}
		thread_should_exit = false;		// 将线程状态位设置为false
		for_delay_test_task = px4_task_spawn_cmd("for_delay_test",		// 线程名
										SCHED_DEFAULT,					// 调度模式
										SCHED_PRIORITY_DEFAULT,			// 优先级
										1200,							// 堆栈大小
										for_delay_test_thread_main,	    // 线程入口
										nullptr);
		if (for_delay_test_task < 0) {
				warn("task start failed");
				return -errno;
			}
		exit(0);						// 正常退出
	}
	if (!strcmp(argv[1], "stop")) {		// shell停止命令
		thread_should_exit = true;
		exit(0);
	}
	if (!strcmp(argv[1], "status")) {	// shell查询命令, 用于查询线程的状态.
		if (thread_running) {
			warnx("\t running\n");
		} else {
			warnx("\t not started\n");
		}
		exit(0);
	}
	usage("unrecognized command");
	exit(1);
}
// 线程主体
int for_delay_test_thread_main(int argc, char *argv[])
{
	thread_running=true;
	// 订阅动补信息
	int _mocap_sub;
	_mocap_sub = orb_subscribe(ORB_ID(att_pos_mocap));
	struct att_pos_mocap_s mocap_sub;
	// 返回时间信息 by CAMERA_TRIGGER
	orb_advert_t _trigger_pub;
	struct camera_trigger_s trigger_pub;
	memset(&trigger_pub, 0, sizeof(trigger_pub));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _mocap_sub;
	fds[0].events = POLLIN;
	while(!thread_should_exit)
	{
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
		// 等待数据更新100ms
		if (pret == 0) {
			continue;
		}
		if (pret < 0) {
			usleep(100000);
			continue;
		}

		// 读取主题
		orb_copy(ORB_ID(att_pos_mocap), _mocap_sub, &mocap_sub);
		// 赋值
		trigger_pub.timestamp=mocap_sub.timestamp;
		trigger_pub.seq=1; //作为标志位
		// 发布主题
		_trigger_pub = orb_advertise(ORB_ID(camera_trigger), &trigger_pub);
		orb_publish(ORB_ID(camera_trigger), _trigger_pub, &trigger_pub);
	}

	thread_running=false;
	return 0;
}
