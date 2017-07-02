#pragma once

#include "DirectionFinder.h"
#include "DirectionFinder_Backend.h"

// Data timeout
#define AP_DIRECTIONFINDER_MAVLINK_TIMEOUT_MS 500

class AP_DirectionFinder_MAVLink : public AP_DirectionFinder_Backend
{

public:
    // constructor
    AP_DirectionFinder_MAVLink(DirectionFinder &directionfinder, uint8_t instance, DirectionFinder::DirectionFinder_State &_state);

    // static detection function
    static bool detect(DirectionFinder &directionfinder, uint8_t instance);

    // update state
    void update(void);

    // Get update from mavlink
    void handle_msg(mavlink_message_t *msg);

private:
    uint32_t last_reading_ms = 0;

    // start a reading
    static bool start_reading(void);
    static bool get_reading(DirectionFinder::DirectionFinder_Reading *reading);
};
