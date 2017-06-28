#include "Copter.h"

// track_init - initialise track controller
bool Copter::track_init(bool ignore_checks)
{
#if FRAME_CONFIG != QUAD_FRAME
	//Track flight mode currently only supports quad frames
	return false;
#endif

	//pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
	pos_control.set_accel_z(g.pilot_accel_z);

}

void Copter::track_run()
{
}
