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

#include "AP_DirectionFinder_MAVLink.h"
#include <AP_HAL/AP_HAL.h>


extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the DirectionFinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the DirectionFinder
*/
AP_DirectionFinder_MAVLink::AP_DirectionFinder_MAVLink(DirectionFinder &_directionfinder, uint8_t instance, DirectionFinder::DirectionFinder_State &_state) :
    AP_DirectionFinder_Backend(_directionfinder, instance, _state)
{
}

/*
   detect if a MAVLink DirectionFinder is connected. We'll detect by
   checking a parameter.
*/
bool AP_DirectionFinder_MAVLink::detect(DirectionFinder &_directionfinder, uint8_t instance)
{
    // Assume that if the user set the DirectionFinder_TYPE parameter to MAVLink,
    // there is an attached MAVLink DirectionFinder
    return true;
}

/*
   Set the distance based on a MAVLINK message
*/
void AP_DirectionFinder_MAVLink::handle_msg(mavlink_message_t *msg)
{
	mavlink_directionfinder_t packet;
	mavlink_msg_directionfinder_decode(msg, &packet);

    last_reading_ms = AP_HAL::millis();
    state.reading.direction = packet.direction;
    state.reading.magnitude = packet.magnitude;
}

/*
   update the state of the sensor
*/
void AP_DirectionFinder_MAVLink::update(void)
{
    //Time out on incoming data; if we don't get new
    //data in 500ms, dump it
    if(AP_HAL::millis() - last_reading_ms > AP_DIRECTIONFINDER_MAVLINK_TIMEOUT_MS) {
        set_status(DirectionFinder::DirectionFinder_NoData);
    } else {
        update_status();
    }
}
