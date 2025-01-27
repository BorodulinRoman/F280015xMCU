#ifndef TIMERS_H
#define TIMERS_H

#include <stdint.h>
#include "device.h"
#include "driverlib.h"

//
// Initializes CPU Timer0 to overflow every 'period_us' microseconds.
// It registers cpuTimer0ISR() and starts the timer.
//
void initCPUTimer0(float period_us);

#endif // TIMERS_H
