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
 * @file px4_adc_reader_app.c
 * Daemon application example for PX4 autopilot
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <nuttx/analog/adc.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <uORB/uORB.h>
#include <uORB/topics/adc_raw_data.h>

#include <drivers/drv_adc.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>



/*Declaration variables*/
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */
orb_advert_t adc_raw_data_t_h; 				/* file handle that will be used for publishing */
adc_raw_data_s adc_data;       				/* make space for a maximum of twelve channels */


/*Declaration functions*/
__EXPORT int px4_adc_reader_app_main(int argc, char *argv[]);   /*px4_adc_reader_app daemon management function*/
int px4_adc_reader_app_thread(int argc, char *argv[]);            /*px4_adc_reader_app daemon main loop*/
static void usage(const char *reason);                          /*Print correct usage of px4_adc_reader_app*/


int px4_adc_reader_app_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("daemon",
					     SCHED_RR,
					     SCHED_PRIORITY_DEFAULT,
					     4096,
					     px4_adc_reader_app_thread,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
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

int px4_adc_reader_app_thread(int argc, char *argv[])
{
	warnx("[daemon] px4_adc_reader_app starting\n");
    thread_running = true;

    /*Opening ADC device */
    int fd = open(ADC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		warnx("ERROR: can't open ADC device");
		return 1;
	}

    while (!thread_should_exit) {
        /*Main Loop */
        
        /* read all channels available */
        ssize_t count = read(fd, adc_data, sizeof(adc_data));
        if (count < 0) {
            warnx("[daemon] px4_adc_reader_app : error reading adc device\n");
            thread_running = false;
            return 1;
        }
        /* publish local position setpoint */
		if (adc_raw_data_t_h > 0) {
			orb_publish(ORB_ID(adc_raw_data), adc_raw_data_t_h, &adc_data);
		} else {
			adc_raw_data_t_h = orb_advertise(ORB_ID(adc_raw_data), &adc_data);
		}
        usleep(50000); /* 20 Hz update rate */
    }
    warnx("[daemon] px4_adc_reader_app exiting\n");
    thread_running = false;
	return 0;
}

static void usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}
