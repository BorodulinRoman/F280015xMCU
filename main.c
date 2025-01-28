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
#include "GenericTypeDefs.h"
#include "device.h"
#include "driverlib.h"
#include "timers.h"
#include "uart_operation_master.h"
#include "uart_operation_slave.h"
#include "globals_and_gpio.h"
#include "filters.h"
//
// Let's define 10 ms = 10,000 microseconds => 50 Hz half-period
//
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

volatile UINT32 SystemTime = 0;
volatile bool pbit_ok = false;
volatile bool p3v3_ok = false;
volatile bool safety_5v = false;

void main(void){
    UINT8 stateTriggerByte;
    Current_state = PBIT;
    //---------------------------------------------------------------------
    // 1) Basic device init (clocks, watchdog)
    //---------------------------------------------------------------------
    Device_init();
    Device_initGPIO();
    setupGPIOs();
    //---------------------------------------------------------------------
    // 2) Initialize interrupt module & vector table
    //---------------------------------------------------------------------
    Interrupt_initModule();
    Interrupt_initVectorTable();
    initSystemTimer();
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

    resetSystemTimer(); // Reset the timer
    while (SystemTime <= WAKE_UP)
    {
        SystemTime = getSystemTime();
    }
    resetSystemTimer(); // Reset the timer
    //---------------------------------------------------------------------
    // 6) Main loop
    //---------------------------------------------------------------------
    while(1)
    {
        SystemTime = getSystemTime();
        if(newDataReceivedMaster)
        {
            newDataReceivedMaster = false;
        }

        if(newDataReceivedSlave)
        {
            stateTriggerByte = g_lastMspMsg.payload[2];
            resetSystemTimer(); // Reset the timer
            newDataReceivedSlave = false;

        }

        switch (Current_state)
           {
           case PBIT: // 1: Power-on Built-In Test
               if (!p3v3_ok)
                   p3v3_ok = gpio_stability_filter(25, 80,P3V3_GOOD); // Check if P3V3 is stable for 50 ms and over 80%
//               if (!safety_5v)
//                   safety_5v = gpio_stability_filter(25, 80,P5V_SAFETY_GOOD_ISO);

               if (p3v3_ok) // add && 5v_safety at main project
                   Current_state = IDNTIF_ACU_GROUND_BASE; // Transition to IDNTIF_ACU_GROUND_BASE

               if(SystemTime > TIMEOUT_PBIT && Current_state == PBIT)
                       Current_state = RFM_ERROR; // Transition to IDNTIF_ACU_GROUND_BASE

               break;

           case IDNTIF_ACU_GROUND_BASE: // 2: Identify ACU - Ground Base

               if (g_lastMspMsg.function == 0x84)
               {
                   Current_state = IDNTIF_ACU_RFM; // Transition to ACU-RFM
               }

               if(gpio_stability_filter(25, 80,SW1_EN_GPIO))
                   Current_state = ACU_ERROR; // Transition to IDNTIF_ACU_GROUND_BASE
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
