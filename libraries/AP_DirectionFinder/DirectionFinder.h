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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

// Maximum number of direction finder instances available on this platform.
#define DIRECTIONFINDER_MAX_INSTANCES 2

class AP_DirectionFinder_Backend;

class DirectionFinder
{
public:
	friend class AP_DirectionFinder_Backend;

	static const struct AP_Param::GroupInfo var_info[];

	DirectionFinder(AP_SerialManager &_serial_manager);

	enum DirectionFinder_Type {
		DirectionFinder_TYPE_NONE 		=	0,
		DirectionFinder_TYPE_UART		=	1,
	};

	enum DirectionFinder_Status {
		DirectionFinder_NotConnected 	= 	0,
		DirectionFinder_NoData			=	1,
		DirectionFinder_Good			=	2,
	};

	struct DirectionFinder_Reading {
		uint8_t direction;
		uint8_t magnitude;
	};

	struct DirectionFinder_State {
		uint8_t							instance;		// The instance number of this DirectionFinder.
		enum DirectionFinder_Status		status;			// DirectionFinder status.
		uint8_t 						valid_count;	// Number of contiguous valid readings
		bool							pre_arm_check;	// True if sensor has passed pre-arm checks.

		DirectionFinder_Reading 		reading;		// The latest reading.
	};

    // parameters for each instance
	AP_Int8  _type[DIRECTIONFINDER_MAX_INSTANCES];
	AP_Int8  _pin[DIRECTIONFINDER_MAX_INSTANCES];
    AP_Int8  _address[DIRECTIONFINDER_MAX_INSTANCES];

	// Return the number of DirectionFinders.
	uint8_t num_sensors(void) const {
		return num_instances;
	}

    // detect and initialise any available rangefinders
    void init(void);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);

	// Returns true if pre-arm checks have passed for all DirectionFinders.
	bool pre_arm_check() const;

private:
    DirectionFinder_State state[DIRECTIONFINDER_MAX_INSTANCES];
	AP_DirectionFinder_Backend *drivers[DIRECTIONFINDER_MAX_INSTANCES];
	uint8_t num_instances:1;
	AP_SerialManager &serial_manager;

	void detect_instance(uint8_t instance);
	void update_instance(uint8_t instance);
	void update_pre_arm_check(uint8_t instance);
	bool _add_backend(AP_DirectionFinder_Backend *driver);
};
