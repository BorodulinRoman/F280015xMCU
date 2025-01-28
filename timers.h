#ifndef TIMERS_H
#define TIMERS_H

#include <stdint.h>
#include <stdbool.h>
#include "device.h"
#include "driverlib.h"

extern volatile uint32_t systemTimeCounter; // System time counter in milliseconds
extern volatile bool systemTimeElapsed; // Flag to indicate if system time has elapsed

//
// initCPUTimer0():
//   Configures CPU Timer0 to overflow every 'period_us' microseconds.
//   The Timer0 ISR will call 'uartMsgTxMaster()' (master transmit).
//
void initCPUTimer0(float period_us);

//
// initSlaveTimer2ms():
//   Configures CPU Timer1 for ~2 ms at 100 MHz. The timer remains
//   stopped until the slave receives a 20-byte packet in scibRxISR().
//   Then scibRxISR() starts Timer1. After 2 ms, Timer1 ISR calls 'uartMsgTxSlave()'.
//
void initSlaveTimer2ms(void);


// System Timer APIs
void initSystemTimer(void);         // Initialize the system timer
void resetSystemTimer(void);        // Reset the system timer counter
uint32_t getSystemTime(void);       // Get the current system time in milliseconds
bool hasSystemTimeElapsed(void);    // Check if 20 seconds have elapsed

#endif // TIMERS_H
