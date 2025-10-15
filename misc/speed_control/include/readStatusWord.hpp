#ifndef READSTATUS_HPP
#define READSTATUS_HPP

#include <stdlib.h>
#include <stdio.h>
#include "../include/frameManager.hpp"

typedef enum
{
    NOT_READY_TO_SWITCH_ON = 0,
    SWITCH_ON_DISABLED = 64,
    READY_TO_SWITCH_ON = 33,
    SWITCHED_ON = 35,
    OPERATION_ENABLED = 39,
    QUICK_STOP_ACTIVE = 7,
    FAULT_REACTION_ACTIVE = 15,
    FAULT = 8,
} DSP402_STATES;

typedef enum 
{
    POSITION_MODE = 1,
    VELOCITY_MODE = 3,
    HOMING_MODE = 6,
    INTERPOLATED_POSITION_MODE = 7, 
} DSP402_MODES;

bool readBit(short word, int bitPosition);

DSP402_STATES read_status_word(short word, DSP402_MODES mode, bool halt);

DSP402_STATES request_status_word(int tty_fd);

#endif