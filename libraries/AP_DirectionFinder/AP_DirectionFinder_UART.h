#pragma once

#include "DirectionFinder.h"
#include "DirectionFinder_Backend.h"

// Maximum number of contiguous readings used to make an aggregate reading
#define DIRECTIONFINDER_MAX_READINGS 5

class AP_DirectionFinder_UART : public AP_DirectionFinder_Backend
{

public:
    // constructor
    AP_DirectionFinder_UART(DirectionFinder &directionFinder, uint8_t instance, DirectionFinder::DirectionFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(DirectionFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;

    bool get_reading(DirectionFinder::DirectionFinder_Reading *reading);
};
