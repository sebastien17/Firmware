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
 * @file px4_adc_reader_app.cpp
 * Daemon application to send adc data over orb for PX4 autopilot
 */
/*
 * Rewritting in C++ of px4_adc_reader_app.c
 */


#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <uORB/uORB.h>
#include <uORB/topics/adc_raw_data.h>
#include <drivers/drv_adc.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#define BLE_FACTOR 0.2

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int px4_adc_reader_app_main(int argc, char *argv[]);

class BrownSimpleExpo
{
    public:
        BrownSimpleExpo()
        {
        	alpha = 0.5;
        	smoothed = 0.0;
        }
        BrownSimpleExpo(double f)
        {
        	alpha = f;
        	smoothed = 0.0;
        }

        void set_factor(float factor)
        {
        	alpha = factor;
        }
        void step(double measurement)
        {
            smoothed = alpha * measurement + (1 - alpha) * smoothed;
        }

        double get(){ return smoothed; }

    private:
        double smoothed;
        double alpha;
};

class BrownLinearExpo
{
    public:
        BrownLinearExpo()
        {
        	estimate = 0.0;
        	double_smoothed = 0.0;
        	single_smoothed = 0.0;
        	a = 0.5;
        }
        BrownLinearExpo(double f)
        {
        	a = f;
        	estimate = 0.0;
        	double_smoothed = 0.0;
        	single_smoothed = 0.0;
        }
        void set_factor(float factor) { a = factor; }
        void step(double measurement)
        {
            single_smoothed = a * measurement + (1 - a) * single_smoothed;
            double_smoothed = a * single_smoothed + (1 - a) * double_smoothed;
            double est_a = (2*single_smoothed - double_smoothed);
            double est_b = (a / (1-a) )*(single_smoothed - double_smoothed);
            estimate = est_a + est_b;
        }
        double get()
        {
        	return estimate;
        }

    private:
        double estimate, double_smoothed, single_smoothed;
        double a;
 };


class Px4_adc_reader
{
public:
	/**
	 * Constructor
	 */
	Px4_adc_reader();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~Px4_adc_reader();
	/**
		 * Start the task.
		 *
		 * @return		OK on success.
		 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */
	int 	_fd;						/*file descriptor*/
	orb_advert_t _adc_raw_data_t_h; 				/* file handle that will be used for publishing */
	struct adc_msg_s _adc_data[12];       				/* make space for a maximum of twelve channels */
	adc_raw_data_s _adc_data_m;       			/* Structure with additional info as mean value or validity make space for a maximum of twelve channels */


	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);
	/**
	 * Main attitude control task.
	 */
	void	task_main();
};
namespace px4_adc_reader_app
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

Px4_adc_reader	*g_control;
}

/*Px4_adc_reader :: Px4_adc_reader():

	_task_should_exit(false),
	_control_task(-1),
{
		initialisation
};*/

Px4_adc_reader::Px4_adc_reader():

	_task_should_exit(false),
	_control_task(-1),
	_fd(-1),
	_adc_raw_data_t_h(-1)
{
	memset(&_adc_data, 0, sizeof(_adc_data));
	memset(&_adc_data_m, 0, sizeof(_adc_data_m));
}


Px4_adc_reader::~Px4_adc_reader()
{
	if (_control_task != -1) {
			/* task wakes up every 100ms or so at the longest */
			_task_should_exit = true;

			/* wait for a second for the task to quit at our request */
			unsigned i = 0;

			do {
				/* wait 20ms */
				usleep(20000);

				/* if we have given up, kill it */
				if (++i > 50) {
					task_delete(_control_task);
					break;
				}
			} while (_control_task != -1);
		}

		px4_adc_reader_app::g_control = nullptr;
};



void
Px4_adc_reader::task_main_trampoline(int argc, char *argv[])
{
	px4_adc_reader_app::g_control->task_main();
}

int
Px4_adc_reader::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("px4_adc_reader_app",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       (main_t)&Px4_adc_reader::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warnx("task start failed");
		return -errno;
	}

	return OK;
}

void
Px4_adc_reader::task_main()
{
	warnx("daemon] px4_adc_reader_app started");
	fflush(stdout);
	/*Open adc signal */
	_fd = open(ADC_DEVICE_PATH, O_RDONLY);
	if (_fd < 0) {
		warnx("ERROR: can't open ADC device");
		_exit(0);
		}
	BrownLinearExpo _ble[12];
	for(int i; i<11; i++)
	{
		_ble[i].set_factor(BLE_FACTOR);
	}

	while (!_task_should_exit)
	{
		/*Main Loop */

		/* read all channels available */
		ssize_t count = read(_fd, &_adc_data, sizeof(_adc_data));
		if (count < 0) {
			warnx("[daemon] px4_adc_reader_app : error reading adc device\n");
			_exit(0);
		}
		/* Data treatment */

		for(int i=0; i<=11;i++)
		{
			/* Brow Linear Expo Low Pass Filter */
			_ble[i].step((double)_adc_data[i].am_data);
			/* ADC Data structure filling */
			_adc_data_m[i].am_channel = _adc_data[i].am_channel;
			_adc_data_m[i].am_data = _adc_data[i].am_data;
			_adc_data_m[i].am_mean_value = (int32_t)_ble[i].get();
		}

		/* Data publication */
		if (_adc_raw_data_t_h > 0) {
			orb_publish(ORB_ID(adc_raw_data), _adc_raw_data_t_h, &_adc_data_m); /* Publish data with additional info */
		} else {
			_adc_raw_data_t_h = orb_advertise(ORB_ID(adc_raw_data), &_adc_data_m); /* Publish data with additional info for the first time */
		}
		usleep(100000); /* 10 Hz update rate */
	}
	warnx("[daemon] px4_adc_reader_app exiting\n");
	_control_task = -1;
	_exit(0);
};

int px4_adc_reader_app_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: px4_adc_reader_app {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (px4_adc_reader_app::g_control != nullptr)
			errx(1, "already running");

		px4_adc_reader_app::g_control = new Px4_adc_reader;

		if (px4_adc_reader_app::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != px4_adc_reader_app::g_control->start()) {
			delete px4_adc_reader_app::g_control;
			px4_adc_reader_app::g_control = nullptr;
			errx(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (px4_adc_reader_app::g_control == nullptr)
			errx(1, "not running");

		delete px4_adc_reader_app::g_control;
		px4_adc_reader_app::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (px4_adc_reader_app::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}

