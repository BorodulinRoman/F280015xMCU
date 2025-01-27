/******************************************************************************
 * File:    main.c
 * Device:  TMS320F280015x (or similar)
 *
 * Purpose:
 *   - Initialize system & SCI (UART).
 *   - Initialize CPU Timer0 to interrupt every 10 ms (or another rate).
 *   - Timer0 ISR calls uartMsgTxMaster() directly, sending a packet each period.
 ******************************************************************************/

#include "device.h"
#include "driverlib.h"
#include "timers.h"
#include "uart_operation_master.h"

//
// Let's define 10 ms = 10,000 microseconds => 50 Hz half-period
//
#define TIMER_PERIOD_US   10000.0f

void main(void)
{
    //---------------------------------------------------------------------
    // 1) Basic device init (clocks, watchdog)
    //---------------------------------------------------------------------
    Device_init();
    Device_initGPIO();

    //---------------------------------------------------------------------
    // 2) Initialize interrupt module & vector table
    //---------------------------------------------------------------------
    Interrupt_initModule();
    Interrupt_initVectorTable();

    //---------------------------------------------------------------------
    // 3) Initialize UART Master (pins, baud=115200, FIFO, etc.)
    //---------------------------------------------------------------------
    uartConfigMaster();

    //---------------------------------------------------------------------
    // 4) Initialize CPU Timer0 for 10 ms period
    //---------------------------------------------------------------------
    initCPUTimer0(TIMER_PERIOD_US);

    //---------------------------------------------------------------------
    // 5) Enable global interrupts
    //---------------------------------------------------------------------
    EINT;   // Enable CPU interrupts
    ERTM;   // Enable real-time debug interrupt

    //---------------------------------------------------------------------
    // 6) Main loop
    //---------------------------------------------------------------------
    while(1)
    {
        //
        // Timer0 ISR will call uartMsgTxMaster() automatically every 10 ms.
        // No need to check a flag or call it manually.
        //

        // If you also receive data, check the newDataReceivedMaster flag here:
        if(newDataReceivedMaster)
        {
            // We have new data in rxBufferMaster

            UINT8 someByte = rxBufferMaster[0];
            newDataReceivedMaster = false;
            // Parse or handle the data
            // ...
        }

        // Other background tasks...
        NOP;
    }
}
