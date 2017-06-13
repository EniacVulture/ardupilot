/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "DirectionFinder.h"
#include "AP_DirectionFinder_UART.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo DirectionFinder::var_info[] = {
	// @Param: 1_TYPE
	// @DisplayName: DirectionFinder type
	// @Description: What type of DirectionFinder device that is connected
	// @Values: 0:None,1:UART
	// @User: Standard
	AP_GROUPINFO("1_TYPE",    0, DirectionFinder, _type[0], 0),

	// @Param: 1_PIN
	// @DisplayName: DirectionFinder pin
	// @Description: Analog pin that DirectionFinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
	// @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
	// @User: Standard
	AP_GROUPINFO("1_PIN",     1, DirectionFinder, _pin[0], -1),

	// @Param: 1_ADDR
	// @DisplayName: Bus address of sensor
	// @Description: This sets the bus address of the sensor, where applicable. Used to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
	// @Range: 0 127
	// @Increment: 1
	// @User: Standard
	AP_GROUPINFO("1_ADDR", 2, DirectionFinder, _address[0], 0),

#if DIRECTIONFINDER_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second DirectionFinder type
    // @Description: What type of DirectionFinder device that is connected
    // @Values: 0:None,1:UART
    // @User: Advanced
    AP_GROUPINFO("2_TYPE",    3, DirectionFinder, _type[1], 0),

    // @Param: 2_PIN
    // @DisplayName: DirectionFinder pin
    // @Description: Analog pin that DirectionFinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("2_PIN",     4, DirectionFinder, _pin[1], -1),

	// @Param: _ADDR
	// @DisplayName: Bus address of sensor
	// @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
	// @Range: 0 127
	// @Increment: 1
	// @User: Standard
	AP_GROUPINFO("2_ADDR", 5, DirectionFinder, _address[1], 0),
#endif
};

DirectionFinder::DirectionFinder(AP_SerialManager &_serial_manager) :
    num_instances(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}

void DirectionFinder::init()
{
	if(num_instances != 0) {
		return;
	}

	for(uint8_t i=0; i< DIRECTIONFINDER_MAX_INSTANCES; ++i) {
		detect_instance(i);
		if(drivers[i] != nullptr) {
			num_instances = i+1;
		}

		state[i].pre_arm_check = false;
		state[i].status = DirectionFinder_NotConnected;
	}
}

void DirectionFinder::update()
{
	for (uint8_t i=0; i<num_instances; i++) {
	        if (drivers[i] != nullptr) {
	            if (_type[i] == DirectionFinder_TYPE_NONE) {
	                // allow user to disable a DirectionFinder at runtime
	                state[i].status = DirectionFinder_NotConnected;
	                continue;
	            }
	            drivers[i]->update();
	            update_pre_arm_check(i);
	        }
	    }
}

void DirectionFinder::detect_instance(uint8_t instance)
{
	enum DirectionFinder_Type type = (enum DirectionFinder_Type)_type[instance].get();
	switch(type) {
	case DirectionFinder_TYPE_UART:
		if (AP_DirectionFinder_UART::detect(*this, instance, serial_manager)) {
			state[instance].instance = instance;
			drivers[instance] = new AP_DirectionFinder_UART(*this, instance, state[instance], serial_manager);
		}
		break;
	default:
		break;
	}
}
