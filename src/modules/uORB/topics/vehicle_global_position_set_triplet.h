/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file vehicle_global_position_setpoint.h
 * Definition of the global WGS84 position setpoint uORB topic.
 */

#ifndef TOPIC_VEHICLE_GLOBAL_POSITION_SET_TRIPLET_H_
#define TOPIC_VEHICLE_GLOBAL_POSITION_SET_TRIPLET_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

#include "vehicle_global_position_setpoint.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Global position setpoint triplet in WGS84 coordinates.
 *
 * This are the three next waypoints (or just the next two or one).
 */
struct vehicle_global_position_set_triplet_s
{
	bool previous_valid;					/**< flag indicating previous position is valid */
	bool next_valid;					/**< flag indicating next position is valid */

	struct vehicle_global_position_setpoint_s previous;
	struct vehicle_global_position_setpoint_s current;
	struct vehicle_global_position_setpoint_s next;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_global_position_set_triplet);

#endif
