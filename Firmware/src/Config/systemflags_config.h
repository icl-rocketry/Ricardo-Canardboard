#pragma once

#include <stdint.h>
#include <type_traits>

enum class SYSTEM_FLAG:uint32_t{
    //state flags
    STATE_IDLE = (1 << 0), 
    //flags
    DEBUG = (1 << 7),
    //critical messages 
    ERROR_SERIAL = (1 << 10),
    //if rocket is inverted
    ERROR_ORIENTATION = (1 << 23),
    ERROR_IMU = (1 << 15),
    ERROR_CAN = (1 << 21),
    //warn
    WARN_BATT = (1 << 24),
    //FLIGHTPHASES 
    FLIGHTPHASE_BOOST = (1 << 25),
    FLIGHTPHASE_COAST = (1 << 26),
    FLIGHTPHASE_APOGEE = (1 << 27),
    ERROR_SD = (1<<19)
    
};

using system_flag_t = uint32_t;

