/****************************************************************************
 *
 *   Copyright 2019 NXP.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * @file nxpcup_main.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/tasks.h>
#include <sched.h>

#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/actuator_armed.h>

#include "nxpcup_race.h"

#define TURNIGY_BRUSHLESS

#ifdef DFROBOT_BRUSHLESS
#define RANGE 100
#define DIR -1

#endif

#ifdef DFROBOT_BRUSHED
#define RANGE 250
#define DIR -1

#endif

#ifdef TURNIGY_BRUSHLESS
#define RANGE 100
#define DIR 1

#endif

static int daemon_task;             /* Handle of deamon task / thread */

bool threadShouldExit = false;
bool threadIsRunning = false;

void roverSteering(float direction, int fd)
{
	if (direction < -1) {
		direction = -1;

	} else if (direction > 1) {
		direction = 1;
	}

	int steer = 1500 - DIR * (int)(500 * direction);
	px4_ioctl(fd, PWM_SERVO_SET(1), steer); // Motor 1 PWM (Steering)
}

void roverSpeed(float speed, int fd)
{
	int speed_intern;

	if (speed < -1) {
		speed = -1;

	} else if (speed > 1) {
		speed = 1;
	}

	speed_intern = 1500 + (int)(speed * RANGE);
	px4_ioctl(fd, PWM_SERVO_SET(3), speed_intern); // Motor 3 PWM (Throttle)
#ifdef DFROBOT_BRUSHED
	px4_ioctl(fd, PWM_SERVO_SET(4), speed_intern); // Motor 4 PWM (Throttle)
#endif
#ifdef DFROBOT_BRUSHLESS
	px4_ioctl(fd, PWM_SERVO_SET(4), speed_intern); // Motor 4 PWM (Throttle)
#endif
}

int race_thread_main(int argc, char **argv)
{
	threadIsRunning = true;

	/* Rover motor control variables */
	roverControl motorControl;
	bool wait  = 1;

	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	/* Start condition of the race */
	bool start = 0;

	/* Pre-arming of the safety-switch - Safety switch useable as start/stop button */
	struct actuator_armed_s _actuator_armed;
	memset(&_actuator_armed, 0, sizeof(_actuator_armed));
	orb_advert_t actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &_actuator_armed);
	_actuator_armed.prearmed = 1;
	orb_publish(ORB_ID(actuator_armed), actuator_armed_pub, &_actuator_armed);

	/* Status safet switch request */
	struct safety_s safety;
	uORB::Subscription safety_sub{ORB_ID(safety)};
	safety_sub.copy(&safety);

	px4_ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE); // Start PWM Test Mode


	Pixy2 pixy;


	usleep(5000);

	if (pixy.init() == 0) {

		pixy.getVersion();
		pixy.version->print();
		usleep(1000);

		while (1) {
			safety_sub.copy(&safety);

			pixy.line.getAllFeatures(LINE_VECTOR, wait);

			switch (safety.safety_off) {
			case 0:
				motorControl.speed = 0;
				motorControl.steer = 0;
				break;

			case 1:
				start = true;

				if (start) {
					motorControl = raceTrack(pixy);

				} else {
					motorControl.speed = 0;
					motorControl.steer = 0;
				}

				break;
			}

			roverSteering(motorControl.steer, fd);
			roverSpeed(motorControl.speed, fd);

			if (threadShouldExit) {
				threadIsRunning = false;
				roverSpeed(0, fd);
				roverSteering(0, fd);
				PX4_INFO("Exit NXPCup Race Thread!\n");
				return 1;
			}
		}
	}

	return 0;
}


extern "C" __EXPORT int nxpcup_main(int argc, char *argv[]);
int nxpcup_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: nxpcup {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (threadIsRunning) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		threadShouldExit = false;
		daemon_task = px4_task_spawn_cmd("nxpcup",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 race_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		threadShouldExit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (threadIsRunning) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	PX4_WARN("usage: race {start|stop|status}\n");
	return 1;
}
