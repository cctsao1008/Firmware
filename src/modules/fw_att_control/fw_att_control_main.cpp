/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier
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
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);

class FixedwingAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingAttitudeControl();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_att_sub;				/**< vehicle attitude subscription */
	int		_accel_sub;				/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 	_params_sub;			/**< notification of parameter updates */
	int 	_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_1_pub;		/**< actuator control group 1 setpoint (Airframe) */

	struct vehicle_attitude_s				_att;			/**< vehicle attitude */
	struct accel_report						_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct airspeed_s						_airspeed;		/**< airspeed */
	struct vehicle_control_mode_s			_vcontrol_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;		/**< actuator control inputs */
	struct vehicle_global_position_s		_global_pos;	/**< global position */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_airspeed_valid;		/**< flag if the airspeed measurement is valid */

	struct {
		float tconst;
		float p_p;
		float p_d;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float p_roll_feedforward;
		float r_p;
		float r_d;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_ff;
		float y_roll_feedforward;
		float y_integrator_max;
		float y_coordinated_min_speed;
		float y_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t tconst;
		param_t p_p;
		param_t p_d;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t p_roll_feedforward;
		param_t r_p;
		param_t r_d;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_d;
		param_t y_ff;
		param_t y_roll_feedforward;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;
	}		_parameter_handles;		/**< handles for interesting parameters */


	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();


	/**
	 * Check for airspeed updates.
	 */
	bool		vehicle_airspeed_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};

namespace att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingAttitudeControl	*g_control;
}

FixedwingAttitudeControl::FixedwingAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_accel_sub(-1),
	_airspeed_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),

/* publications */
	_rate_sp_pub(-1),
	_actuators_0_pub(-1),
	_attitude_sp_pub(-1),
	_actuators_1_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw att control")),
/* states */
	_setpoint_valid(false),
	_airspeed_valid(false)
{
	/* safely initialize structs */
	_att = {0};
	_accel = {0};
	_att_sp = {0};
	_manual = {0};
	_airspeed = {0};
	_vcontrol_mode = {0};
	_actuators = {0};
	_actuators_airframe = {0};
	_global_pos = {0};


	_parameter_handles.tconst = param_find("FW_ATT_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");
	_parameter_handles.p_roll_feedforward = param_find("FW_P_ROLLFF");

	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
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

	att_control::g_control = nullptr;
}

int
FixedwingAttitudeControl::parameters_update()
{

	param_get(_parameter_handles.tconst, &(_parameters.tconst));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));
	param_get(_parameter_handles.p_roll_feedforward, &(_parameters.p_roll_feedforward));

	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.tconst);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));
	_pitch_ctrl.set_roll_ff(_parameters.p_roll_feedforward);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.tconst);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	return OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

bool
FixedwingAttitudeControl::vehicle_airspeed_poll()
{
	/* check if there is a new position */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
//		warnx("airspeed poll: ind: %.4f,  true: %.4f", _airspeed.indicated_airspeed_m_s, _airspeed.true_airspeed_m_s);
		return true;
	}

	return false;
}

void
FixedwingAttitudeControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
FixedwingAttitudeControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedwingAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
FixedwingAttitudeControl::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	/* rate limit attitude control to 50 Hz (with some margin, so 17 ms) */
	orb_set_interval(_att_sub, 17);

	parameters_update();

	/* get an initial update for all sensor and status data */
	(void)vehicle_airspeed_poll();
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

			_airspeed_valid = vehicle_airspeed_poll();

			vehicle_setpoint_poll();

			vehicle_accel_poll();

			vehicle_control_mode_poll();

			vehicle_manual_poll();

			global_pos_poll();

			/* lock integrator until control is started */
			bool lock_integrator;

			if (_vcontrol_mode.flag_control_attitude_enabled) {
				lock_integrator = false;

			} else {
				lock_integrator = true;
			}

			/* Simple handling of failsafe: deploy parachute if failsafe is on */
			if (_vcontrol_mode.flag_control_termination_enabled) {
				_actuators_airframe.control[1] = 1.0f;
//				warnx("_actuators_airframe.control[1] = 1.0f;");
			} else {
				_actuators_airframe.control[1] = 0.0f;
//				warnx("_actuators_airframe.control[1] = -1.0f;");
			}

			/* decide if in stabilized or full manual control */

			if (_vcontrol_mode.flag_control_attitude_enabled) {

				/* scale around tuning airspeed */

				float airspeed;

				/* if airspeed is smaller than min, the sensor is not giving good readings */
				if (!_airspeed_valid ||
				    (_airspeed.indicated_airspeed_m_s < 0.5f * _parameters.airspeed_min) ||
				    !isfinite(_airspeed.indicated_airspeed_m_s)) {
					airspeed = _parameters.airspeed_trim;

				} else {
					airspeed = _airspeed.indicated_airspeed_m_s;
				}

				float airspeed_scaling = _parameters.airspeed_trim / airspeed;
				//warnx("aspd scale: %6.2f act scale: %6.2f", airspeed_scaling, actuator_scaling);

				float roll_sp = 0.0f;
				float pitch_sp = 0.0f;
				float throttle_sp = 0.0f;

				if (_vcontrol_mode.flag_control_velocity_enabled || _vcontrol_mode.flag_control_position_enabled) {
					roll_sp = _att_sp.roll_body;
					pitch_sp = _att_sp.pitch_body;
					throttle_sp = _att_sp.thrust;

					/* reset integrals where needed */
					if (_att_sp.roll_reset_integral)
						_roll_ctrl.reset_integrator();

				} else {
					/*
					 * Scale down roll and pitch as the setpoints are radians
					 * and a typical remote can only do 45 degrees, the mapping is
					 * -1..+1 to -45..+45 degrees or -0.75..+0.75 radians.
					 *
					 * With this mapping the stick angle is a 1:1 representation of
					 * the commanded attitude. If more than 45 degrees are desired,
					 * a scaling parameter can be applied to the remote.
					 */
					roll_sp = _manual.roll * 0.75f;
					pitch_sp = _manual.pitch * 0.75f;
					throttle_sp = _manual.throttle;
					_actuators.control[4] = _manual.flaps;

					/*
					 * in manual mode no external source should / does emit attitude setpoints.
					 * emit the manual setpoint here to allow attitude controller tuning
					 * in attitude control mode.
					 */
					struct vehicle_attitude_setpoint_s att_sp;
					att_sp.timestamp = hrt_absolute_time();
					att_sp.roll_body = roll_sp;
					att_sp.pitch_body = pitch_sp;
					att_sp.yaw_body = 0.0f;
					att_sp.thrust = throttle_sp;

					/* lazily publish the setpoint only once available */
					if (_attitude_sp_pub > 0) {
						/* publish the attitude setpoint */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &att_sp);

					} else {
						/* advertise and publish */
						_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
					}
				}

				/* Prepare speed_body_u and speed_body_w */
				float speed_body_u = 0.0f;
				float speed_body_v = 0.0f;
				float speed_body_w = 0.0f;
				if(_att.R_valid) 	{
					speed_body_u = _att.R[0][0] * _global_pos.vel_n + _att.R[1][0] * _global_pos.vel_e + _att.R[2][0] * _global_pos.vel_d;
					speed_body_v = _att.R[0][1] * _global_pos.vel_n + _att.R[1][1] * _global_pos.vel_e + _att.R[2][1] * _global_pos.vel_d;
					speed_body_w = _att.R[0][2] * _global_pos.vel_n + _att.R[1][2] * _global_pos.vel_e + _att.R[2][2] * _global_pos.vel_d;
				} else	{
					warnx("Did not get a valid R\n");
				}

				/* Run attitude controllers */
				if (isfinite(roll_sp) && isfinite(pitch_sp)) {
					_roll_ctrl.control_attitude(roll_sp, _att.roll);
					_pitch_ctrl.control_attitude(pitch_sp, _att.roll, _att.pitch, airspeed);
					_yaw_ctrl.control_attitude(_att.roll, _att.pitch,
							speed_body_u, speed_body_v, speed_body_w,
							_roll_ctrl.get_desired_rate(), _pitch_ctrl.get_desired_rate()); //runs last, because is depending on output of roll and pitch attitude

					/* Run attitude RATE controllers which need the desired attitudes from above */
					float roll_u = _roll_ctrl.control_bodyrate(_att.pitch,
							_att.rollspeed, _att.yawspeed,
							_yaw_ctrl.get_desired_rate(),
							_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
					_actuators.control[0] = (isfinite(roll_u)) ? roll_u : 0.0f;
					if (!isfinite(roll_u)) {
						warnx("roll_u %.4f", roll_u);
					}

					float pitch_u = _pitch_ctrl.control_bodyrate(_att.roll, _att.pitch,
							_att.pitchspeed, _att.yawspeed,
							_yaw_ctrl.get_desired_rate(),
							_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
					_actuators.control[1] = (isfinite(pitch_u)) ? pitch_u : 0.0f;
					if (!isfinite(pitch_u)) {
						warnx("pitch_u %.4f, _yaw_ctrl.get_desired_rate() %.4f, airspeed %.4f, airspeed_scaling %.4f, roll_sp %.4f, pitch_sp %.4f, _roll_ctrl.get_desired_rate() %.4f, _pitch_ctrl.get_desired_rate() %.4f att_sp.roll_body %.4f",
								pitch_u, _yaw_ctrl.get_desired_rate(), airspeed, airspeed_scaling, roll_sp, pitch_sp, _roll_ctrl.get_desired_rate(), _pitch_ctrl.get_desired_rate(), _att_sp.roll_body);
					}

					float yaw_u = _yaw_ctrl.control_bodyrate(_att.roll, _att.pitch,
							_att.pitchspeed, _att.yawspeed,
							_pitch_ctrl.get_desired_rate(),
							_parameters.airspeed_min, _parameters.airspeed_max, airspeed, airspeed_scaling, lock_integrator);
					_actuators.control[2] = (isfinite(yaw_u)) ? yaw_u : 0.0f;
					if (!isfinite(yaw_u)) {
						warnx("yaw_u %.4f", yaw_u);
					}

					/* throttle passed through */
					_actuators.control[3] = (isfinite(throttle_sp)) ? throttle_sp : 0.0f;
					if (!isfinite(throttle_sp)) {
						warnx("throttle_sp %.4f", throttle_sp);
					}
				} else {
					warnx("Non-finite setpoint roll_sp: %.4f, pitch_sp %.4f", roll_sp, pitch_sp);
				}

				// warnx("aspd: %s: %6.2f, aspd scaling: %6.2f, controls: %5.2f %5.2f %5.2f %5.2f", (_airspeed_valid) ? "valid" : "unknown",
				// 			airspeed, airspeed_scaling, _actuators.control[0], _actuators.control[1],
				// 			_actuators.control[2], _actuators.control[3]);

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				vehicle_rates_setpoint_s rates_sp;
				rates_sp.roll = _roll_ctrl.get_desired_rate();
				rates_sp.pitch = _pitch_ctrl.get_desired_rate();
				rates_sp.yaw = _yaw_ctrl.get_desired_rate();

				rates_sp.timestamp = hrt_absolute_time();

				if (_rate_sp_pub > 0) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_rates_setpoint), _rate_sp_pub, &rates_sp);

				} else {
					/* advertise and publish */
					_rate_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);
				}

			} else {
				/* manual/direct control */
				_actuators.control[0] = _manual.roll;
				_actuators.control[1] = _manual.pitch;
				_actuators.control[2] = _manual.yaw;
				_actuators.control[3] = _manual.throttle;
				_actuators.control[4] = _manual.flaps;
			}

			_actuators.control[5] = _manual.aux1;
			_actuators.control[6] = _manual.aux2;
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp = hrt_absolute_time();

			if (_actuators_0_pub > 0) {
				/* publish the attitude setpoint */
				orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

			} else {
				/* advertise and publish */
				_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
			}

			if (_actuators_1_pub > 0) {
				/* publish the attitude setpoint */
				orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_airframe);
//				warnx("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
//						(double)_actuators_airframe.control[0], (double)_actuators_airframe.control[1], (double)_actuators_airframe.control[2],
//						(double)_actuators_airframe.control[3], (double)_actuators_airframe.control[4], (double)_actuators_airframe.control[5],
//						(double)_actuators_airframe.control[6], (double)_actuators_airframe.control[7]);

			} else {
				/* advertise and publish */
				_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_airframe);
			}

		}

		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_exit(0);
}

int
FixedwingAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&FixedwingAttitudeControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_att_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: fw_att_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr)
			errx(1, "already running");

		att_control::g_control = new FixedwingAttitudeControl;

		if (att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr)
			errx(1, "not running");

		delete att_control::g_control;
		att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
