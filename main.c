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
// Define the states
#define PBIT                        1
#define IDNTIF_ACU_GROUND_BASE      2
#define IDNTIF_ACU_RFM              3
#define READY                       4
#define ARMING                      5
#define ARMED                       6
#define DET                         7
#define END_MISSION                 8
#define DISARM                      9
#define ACU_ERROR                   10
#define RFM_ERROR                   11
#define SW_MIS_ACTIVITION           12
volatile int Current_state = PBIT;

void main(void)
{
    UINT8 stateTriggerByte;
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
            newDataReceivedMaster = false;
        }

        if(newDataReceivedSlave)
        {
            stateTriggerByte = g_lastMspMsg.payload[2];
            newDataReceivedSlave = false;
        }

        switch (Current_state)
           {
           case PBIT: // 1: Power-on Built-In Test


               if (stateTriggerByte == 0x02)
               {
                   Current_state = IDNTIF_ACU_GROUND_BASE; // Transition to IDENT_ACU
               }
               break;

           case IDNTIF_ACU_GROUND_BASE: // 2: Identify ACU - Ground Base


               if (stateTriggerByte == 0x03)
               {
                   Current_state = IDNTIF_ACU_RFM; // Transition to ACU-RFM
               }
               break;

           case IDNTIF_ACU_RFM: // 3: ACU-RFM Communication Established


               if (stateTriggerByte == 0x04)
               {
                   Current_state = READY; // Transition to READY
               }
               break;

           case READY: // 4: System Ready


               if (stateTriggerByte == 0x05)
               {
                   Current_state = ARMING; // Transition to ARMING
               }
               break;

           case ARMING: // 5: Arming State


               if (stateTriggerByte == 0x06)
               {
                   Current_state = ARMED; // Transition to ARMED
               }
               break;
           case ARMED: // 6: Armed State


               if (stateTriggerByte == 0x07)
               {
                   Current_state = DET;
               }
               break;
           case DET: // 7: DET State

               if (stateTriggerByte == 0x08)
               {
                   Current_state = PBIT;
               }
               break;
           case END_MISSION: // 8: DET State


               if (stateTriggerByte == 0x08)
               {
                   Current_state = PBIT;
               }
               break;
           case DISARM: // 9: DISARM State


               if (stateTriggerByte == 0x08)
               {
                   Current_state = PBIT;
               }
               break;
           case ACU_ERROR: // 10: ACU_ERROR State


               if (stateTriggerByte == 0x08)
               {
                   Current_state = PBIT;
               }
               break;
           case RFM_ERROR: // 11: RFM_ERROR State


               if (stateTriggerByte == 0x08)
               {
                   Current_state = PBIT;
               }
               break;
           case SW_MIS_ACTIVITION: // 12: SW_MIS_ACTIVITION State


               if (stateTriggerByte == 0x08)
               {
                   Current_state = PBIT;
               }
               break;


           default:
               Current_state = PBIT;
               break;
           }


    }
}
