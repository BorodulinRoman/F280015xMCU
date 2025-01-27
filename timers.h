#ifndef TIMERS_H
#define TIMERS_H

#include <stdint.h>
#include <stdbool.h>
#include "device.h"
#include "driverlib.h"

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

#endif // TIMERS_H
