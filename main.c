/******************************************************************************
 * File:    main.c
 * Device:  TMS320F280015x (or similar)
 *
 * Purpose:
 *   - Initializes system, SCIA (Master), SCIB (Slave).
 *   - CPU Timer0 triggers Master transmissions every 10 ms (SCI-A).
 *   - When Slave (SCI-B) receives 20 bytes, it starts Timer1 for 2 ms.
 *     After 2 ms, Timer1 ISR calls `uartMsgTxSlave()` to send a random packet.
 *   - The main loop checks flags from both Master and Slave to see if data
 *     arrived, and can process it if needed.
 ******************************************************************************/

#include "device.h"
#include "driverlib.h"
#include "timers.h"
#include "uart_operation_master.h"
#include "uart_operation_slave.h"    // <--- Include the slave interface

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
    // 3) Initialize UART Master (SCI-A) and Timer0
    //---------------------------------------------------------------------
    uartConfigMaster();                  // SCIA pins: 28=RX, 29=TX
    initCPUTimer0(TIMER_PERIOD_US);      // 10 ms period for Master transmissions

    //---------------------------------------------------------------------
    // 4) Initialize UART Slave (SCI-B)
    //    This also calls initSlaveTimer2ms() for the 2 ms delayed response
    //---------------------------------------------------------------------
    uartConfigSlave();                   // SCIB pins: 23=RX, 40=TX
    // Timer1 remains stopped until scibRxISR sees 20 bytes.

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
        // MASTER:
        // Timer0 ISR calls uartMsgTxMaster() every 10 ms automatically.
        // If new data arrives on SCIA (master), sciaRxISR sets newDataReceivedMaster:
        //
        if(newDataReceivedMaster)
        {
            // We have new data in rxBufferMaster
            // For example, read the last CRC byte at index 19:
            UINT8 someMasterByte = rxBufferMaster[19];
            newDataReceivedMaster = false;

            // Parse or handle the data from the Master side
            // ...
        }

        //
        // SLAVE:
        // If new data arrives on SCIB, scibRxISR sets newDataReceivedSlave,
        // then starts Timer1 for a 2 ms delay. The Timer1 ISR calls
        // uartMsgTxSlave() to send random data. If you want to process
        // the Slave's received data in the main loop:
        //
        if(newDataReceivedSlave)
        {
            // We have new data in rxBufferSlave
            UINT8 someSlaveByte = rxBufferSlave[0];
            newDataReceivedSlave = false;

            // Parse or handle the data from the Slave side
            // ...
        }

        // Other background tasks...
        NOP;
    }
}
