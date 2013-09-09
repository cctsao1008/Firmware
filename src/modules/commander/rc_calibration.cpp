/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file rc_calibration.cpp
 * Remote Control calibration routine
 */

#include "rc_calibration.h"
#include "commander_helper.h"

#include <poll.h>
#include <unistd.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

int do_rc_calibration(int mavlink_fd)
{
	mavlink_log_info(mavlink_fd, "trim calibration starting");

	int sub_man = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s sp;
	bool changed;
	orb_check(sub_man, &changed);

	if (!changed) {
		mavlink_log_critical(mavlink_fd, "no manual control, aborting");
		return ERROR;
	}

	orb_copy(ORB_ID(manual_control_setpoint), sub_man, &sp);

	/* set parameters */
	float p = sp.roll;
	param_set(param_find("TRIM_ROLL"), &p);
	p = sp.pitch;
	param_set(param_find("TRIM_PITCH"), &p);
	p = sp.yaw;
	param_set(param_find("TRIM_YAW"), &p);

	/* store to permanent storage */
	/* auto-save */
	int save_ret = param_save_default();

	if (save_ret != 0) {
		mavlink_log_critical(mavlink_fd, "TRIM CAL: WARN: auto-save of params failed");
		close(sub_man);
		return ERROR;
	}

	mavlink_log_info(mavlink_fd, "trim calibration done");
	close(sub_man);
	return OK;
}
