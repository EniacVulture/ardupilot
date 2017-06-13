/*
 *  RangeFinder test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_DirectionFinder/AP_DirectionFinder.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_SerialManager serial_manager;
static DirectionFinder directionFinder {serial_manager};

void setup()
{
    // print welcome message
    hal.console->printf("Range Finder library test\n");

    // setup for analog pin 13
    AP_Param::set_object_value(&directionFinder, directionFinder.var_info, "_TYPE", DirectionFinder::DirectionFinder_TYPE_UART);
    AP_Param::set_object_value(&directionFinder, directionFinder.var_info, "_PIN", -1.0f);
    AP_Param::set_object_value(&directionFinder, directionFinder.var_info, "_SCALING", 1.0f);

    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    directionFinder.init();
    hal.console->printf("RangeFinder: %d devices detected\n", directionFinder.num_sensors());
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    directionFinder.update();
}
AP_HAL_MAIN();
