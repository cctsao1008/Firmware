/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file uorb_blocks.cpp
 *
 * uorb block library code
 */

#include "blocks.hpp"

namespace control
{

BlockWaypointGuidance::BlockWaypointGuidance(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	_xtYawLimit(this, "XT2YAW"),
	_xt2Yaw(this, "XT2YAW"),
	_psiCmd(0)
{
}

BlockWaypointGuidance::~BlockWaypointGuidance() {};

void BlockWaypointGuidance::update(vehicle_global_position_s &pos,
				   vehicle_attitude_s &att,
				   position_setpoint_s &missionCmd,
				   position_setpoint_s &lastMissionCmd)
{

	// heading to waypoint
	float psiTrack = get_bearing_to_next_waypoint(
				 (double)pos.lat / (double)1e7d,
				 (double)pos.lon / (double)1e7d,
				 missionCmd.lat,
				 missionCmd.lon);

	// cross track
	struct crosstrack_error_s xtrackError;
	get_distance_to_line(&xtrackError,
			     (double)pos.lat / (double)1e7d,
			     (double)pos.lon / (double)1e7d,
			     lastMissionCmd.lat,
			     lastMissionCmd.lon,
			     missionCmd.lat,
			     missionCmd.lon);

	_psiCmd = _wrap_2pi(psiTrack -
			    _xtYawLimit.update(_xt2Yaw.update(xtrackError.distance)));
}

BlockUorbEnabledAutopilot::BlockUorbEnabledAutopilot(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	// subscriptions
	_att(&getSubscriptions(), ORB_ID(vehicle_attitude), 20),
	_attCmd(&getSubscriptions(), ORB_ID(vehicle_attitude_setpoint), 20),
	_ratesCmd(&getSubscriptions(), ORB_ID(vehicle_rates_setpoint), 20),
	_pos(&getSubscriptions() , ORB_ID(vehicle_global_position), 20),
	_missionCmd(&getSubscriptions(), ORB_ID(position_setpoint_triplet), 20),
	_manual(&getSubscriptions(), ORB_ID(manual_control_setpoint), 20),
	_status(&getSubscriptions(), ORB_ID(vehicle_status), 20),
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz
	// publications
	_actuators(&getPublications(), ORB_ID(actuator_controls_0))
{
}

BlockUorbEnabledAutopilot::~BlockUorbEnabledAutopilot() {};

} // namespace control

