/******************************************************************************
 * File:    main.c
 * Device:  TMS320F2800157
 * Author:  Your Name
 *
 * Description:
 *    - Initializes system clock, peripherals, and interrupt controller.
 *    - Configures UART as defined in uart_operation_master.c.
 *    - Transmits a message over UART every ~10 ms using a software delay.
 *    - Checks if new data is received in each loop iteration.
 ******************************************************************************/

#include "device.h"
#include "driverlib.h"
#include "uart_operation_master.h"

// Optional: If you don't have DEVICE_DELAY_US, you can do your own loop-based delay:
//  Example: ~10 ms at 100 MHz CPU (very approximate!)
//  static void softwareDelay10ms(void)
//  {
//      volatile uint32_t i;
//      for (i = 0; i < 1000000UL; i++)
//      {
//          // do nothing
//      }
//  }

int main(void)
{
    // -------------------------------------------------------------------------
    // System Initialization
    // -------------------------------------------------------------------------
    // Set up device clocks and peripherals
    Device_init();

    // Disable watchdog (so it doesn't reset the device during our delay)
    SysCtl_disableWatchdog();

    // Initialize GPIO
    Device_initGPIO();

    // Initialize interrupt module
    Interrupt_initModule();

    // Initialize the PIE vector table
    Interrupt_initVectorTable();

    // -------------------------------------------------------------------------
    // UART Initialization
    // -------------------------------------------------------------------------
    uartConfigMaster();  // from uart_operation_master.c

    // Optionally enable global interrupts if not enabled in uartConfigMaster
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global real-time interrupt DBGM

    // -------------------------------------------------------------------------
    // Main Application Loop
    // -------------------------------------------------------------------------
    while(1)
    {
        // 1) Transmit a message
        uartMsgTxMaster();
        //    This will cause the TX interrupt to fire and send out the data.

        // 2) Delay ~10 ms (busy-wait)
        //    If you have the macro, use it:
        DEVICE_DELAY_US(10000);
        //    Otherwise, use your own function or for-loop based delay:
        // softwareDelay10ms();

        // 3) Check if we received a complete message
        if(newDataReceivedMaster)
        {
            // We have new data in rxBufferMaster
            // Process or parse the data
            uartMsgRxMaster();

            // Reset the flag so we can detect new arrivals
            newDataReceivedMaster = false;
        }
    }

    // Should never get here
    return 0;
}
