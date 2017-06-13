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
#include "DirectionFinder.h"

class AP_DirectionFinder_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_DirectionFinder_Backend(DirectionFinder &_directionFinder, uint8_t instance, DirectionFinder::DirectionFinder_State &_state);

    // we declare a virtual destructor so that DirectionFinder drivers can
    // override with a custom destructor if need be
    virtual ~AP_DirectionFinder_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    virtual void handle_msg(mavlink_message_t *msg) { return; }

protected:

    // update status based on direction measurement
    void update_status();

    // set status
    void set_status(DirectionFinder::DirectionFinder_Status status);

    DirectionFinder &directionFinder;
    DirectionFinder::DirectionFinder_State &state;

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;
};
