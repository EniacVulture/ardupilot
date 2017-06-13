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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "DirectionFinder.h"
#include "DirectionFinder_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor.
  This incorporates initialisation as well.
*/

AP_DirectionFinder_Backend::AP_DirectionFinder_Backend(DirectionFinder &_directionFinder, uint8_t instance, DirectionFinder::DirectionFinder_State &_state) :
        directionFinder(_directionFinder),
        state(_state)
{
    _sem = hal.util->new_semaphore();
}

// update status based on distance measurement
void AP_DirectionFinder_Backend::update_status()
{
	set_status(DirectionFinder::DirectionFinder_Good);
}

// set status and update valid count
void AP_DirectionFinder_Backend::set_status(DirectionFinder::DirectionFinder_Status status)
{
    state.status = status;

    // update valid count
	if (status == DirectionFinder::DirectionFinder_Good) {
		if (state.valid_count < 10) {
			state.valid_count++;
		}
	} else {
		state.valid_count = 0;
	}
}
