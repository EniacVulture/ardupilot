#include "Copter.h"

//TODO should be parameters, not hard coded
#define TRACK_MIN_TARGET_CLIMB_RATE_TAKEOFF 10.0f
#define TRACK_ALT_TARGET 300.0f
#define TRACK_Z_SPEED 50.0f
#define TRACK_Z_ACCEL 50.0f
#define TRACK_XY_SPEED 50.0f
#define TRACK_XY_ACCEL 50.0f

// track_init - initialise track controller
bool Copter::track_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
	// do not allow helis to enter Track if the Rotor Runup is not complete
	if (!ignore_checks && !motors.rotor_runup_complete()){
		return false;
	}
#endif

	// Initialize maximum speeds and accels
	pos_control.set_speed_z(-TRACK_Z_SPEED, TRACK_Z_SPEED);
	pos_control.set_accel_z(TRACK_Z_ACCEL);
	pos_control.set_jerk_xy_to_default();
	pos_control.set_speed_xy(TRACK_XY_SPEED);
	pos_control.set_accel_xy(TRACK_XY_ACCEL);

	// If z-controller has been active recently
	if(!pos_control.is_active_z()){
		// Set desired targets
		pos_control.set_alt_target_to_current_alt();
		pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());
	}

	takeoff_stop();

	return true;
}

void Copter::track_run()
{
	TrackModeState track_state;


	// Initialize maximum speeds and accels
	pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
	pos_control.set_accel_z(g.pilot_accel_z);
	pos_control.set_speed_xy(TRACK_XY_SPEED);
	pos_control.set_accel_xy(TRACK_XY_ACCEL);

	float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
	target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
	bool takeoff_triggered = (ap.land_complete && (target_climb_rate > TRACK_MIN_TARGET_CLIMB_RATE_TAKEOFF) && motors.rotor_runup_complete());
#else
	bool takeoff_triggered = (ap.land_complete && (target_climb_rate > TRACK_MIN_TARGET_CLIMB_RATE_TAKEOFF));
#endif

	// Track state machine determination
	if (!motors.armed() || !motors.get_interlock()){
		track_state = Track_MotorStopped;
	} else if (takeoff_state.running || takeoff_triggered) {
		track_state = Track_Takeoff;
	} else if (!ap.auto_armed || ap.land_complete) {
		track_state = Track_Landed;
	} else {
		track_state = Track_Flying;
	}

	switch (track_state) {

	case Track_MotorStopped:
		motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
#if FRAME_CONFIG == HELI_FRAME
		//pos_control.set_alt_target_from_climb_rate(-abs(g.land_speed), G_dt, false);
#else
		attitude_control.reset_rate_controller_I_terms();
		attitude_control.set_yaw_target_to_current_heading();
		pos_control.relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
#endif
		pos_control.update_z_controller();
		break;

	case Track_Takeoff:
		// Set motor to full range
		motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

		// Initiate take-off
		if(!takeoff_state.running) {
			takeoff_timer_start(constrain_float(g.pilot_takeoff_alt, 0.0f,1000.0f));
			set_land_complete(false);
			set_throttle_takeoff();
		}

		pos_control.set_alt_target(TRACK_ALT_TARGET);
		pos_control.update_z_controller();

		break;

	case Track_Landed:
		// Set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
		if(target_climb_rate < 0.0f) {
			motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
		} else {
			motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
		}

		// Relax all controllers.
		attitude_control.reset_rate_controller_I_terms();
		attitude_control.set_yaw_target_to_current_heading();
		attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
		pos_control.relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
		pos_control.update_z_controller();
		break;

	case Track_Flying:
		motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

		if(directionfinder.has_data()){
			// set pos_control xy velocity to 0
			pos_control.set_desired_velocity_xy(0.0f, 0.0f);
		} else {
			// update pos control xy velocity based on directionfinder
			float vel_x, vel_y;
			calculate_velocity_xy(directionfinder.direction(), directionfinder.magnitude(), &vel_x, &vel_y);
			pos_control.set_desired_velocity_xy(vel_x, vel_y);
		}

		pos_control.set_alt_target(TRACK_ALT_TARGET);
		pos_control.update_z_controller();
		pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler, false);
		break;
	}
}

void calculate_velocity_xy(uint8_t direction, uint8_t magnitude, float *vel_x, float *vel_y)
{
	float angle = (360 / 256) * direction;
	*vel_x = sinf(angle) * magnitude;
	*vel_y = sqrt((magnitude * magnitude) - (*vel_x * *vel_x));
}
