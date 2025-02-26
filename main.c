/******************************************************************************
 * File:    main.c
 * Device:  TMS320F280015x (or similar)
 *
 * Purpose:
 *   - Initializes system, SCIA (Master), and SCIB (ACU).
 *   - Uses Timer2 to count system time with a 1 ms tick.
 *   - Every 10 ms, Timer2 ISR triggers master transmission via uartMsgTxMaster().
 *   - Every 100 ms, Timer2 ISR triggers ACU transmission via uartMsgTxACU(), but only if enabled.
 *   - ACU transmission is enabled after 1000 seconds (1,000,000 ms).
 *   - When systemTimeCounter reaches 60,000 ms (60 seconds), a flag is set.
 *   - The main loop can check flags and process data as needed.
 *
 * Note:
 *   - Timer0 and Timer1 functionalities have been removed to consolidate timing on Timer2.
 *   - Both uartMsgTxMaster() and uartMsgTxACU() are triggered from the Timer2 ISR.
 ******************************************************************************/
#include "GenericTypeDefs.h"
#include "device.h"
#include "driverlib.h"
#include "timers.h"
#include "uart_operation_master.h"
#include "uart_operation_acu.h"
#include "globals_and_gpio.h"
#include "filters.h"
//#include "c28x_core_registers.h"

volatile UINT32 SystemTime = 0;
volatile bool filtred_p3v3_ok = false;
volatile bool filtred_safety_pin = false;
volatile bool filtred_comOkMaster = false;
volatile bool filtred_comOkSlave = false;
volatile bool filtred_hvCup_safe = false;
volatile bool filtred_hvCup_armed = false;

void main(void)
    {
    Current_state = PBIT;
    uint32_t SystemTime = 0;
    //---------------------------------------------------------------------
    // 1) Basic device init (clocks, watchdog, GPIOs)
    //---------------------------------------------------------------------
    Device_init();
    Device_initGPIO();
    setupGPIOs();
    //---------------------------------------------------------------------
    // 2) Initialize interrupt module & vector table
    //---------------------------------------------------------------------
    Interrupt_initModule();
    Interrupt_initVectorTable();
    //---------------------------------------------------------------------
    // 3) Initialize system timer (Timer2)
    //---------------------------------------------------------------------
    initSystemTimer();
    //---------------------------------------------------------------------
    // 4) Initialize UART modules for Master (SCIA) and ACU (SCIB)
    //---------------------------------------------------------------------
    uartConfigMaster();  // Master: SCIA pins: 28=RX, 29=TX
    uartConfigACU();     // ACU:    SCIB pins: 23=RX, 40=TX
    //---------------------------------------------------------------------
    // 5) Enable global interrupts
    //---------------------------------------------------------------------
    EINT;   // Enable CPU interrupts
    ERTM;   // Enable real-time debug interrupt

    resetSystemTimer(); // Reset the timer

    // Wait until 1000 seconds (1,000,000 ms) have elapsed before enabling ACU TX
    while(SystemTime < WAKE_UP)
    {
        SystemTime = getSystemTime();
    }
    acuEnableTx = true;
    resetSystemTimer(); // Reset the timer


    while(1)
    {
        SystemTime = getSystemTime();
        switch (Current_state)
           {
            case PBIT: // 1: Power-on Built-In Test
            {
                // Check power supplies, we expect P3V3 to be High and Safety 5V to be Low.
                // With the filter, we require at least 7 out of 10 measurements (70%) to match.
                if (!filtred_p3v3_ok){
                    filtred_p3v3_ok = gpio_stability_filter(FILTER_GPIO, FILTER_ACCURACY, P3V3_GOOD, true);
                }

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //if (!filtred_safety_pin){
                //    filtred_safety_pin = gpio_stability_filter(FILTER_GPIO, FILTER_ACCURACY, P5V_SAFETY_GOOD_ISO, false);
                //}
                filtred_safety_pin = true;
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                // Communication check:
                if(!filtred_comOkMaster){
                    filtred_comOkMaster = checkComOkMaster(FILTER_PROTOCOL);
                }

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                // (Optional) Add any additional SPI-based verification for the accelerometer here.
                // For example: accVerified = spi_verifyAccelerometer();
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                // Use the system timer (SystemTime) and TIMEOUT_PBIT to decide when to evaluate the results.
                if(SystemTime >= TIMEOUT_PBIT){
                    // At the end of the PBIT period, check if all conditions are met.
                    if(filtred_p3v3_ok && filtred_safety_pin && filtred_comOkMaster){
                        // All conditions met, proceed to the next state.
                        Current_state = IDNTIF_RFM_FLIGHT_MODULE;
                        filtred_comOkMaster = false;
                        filtred_safety_pin = true;
                        resetSystemTimer(); // Reset the timer
                    }
                    else{
                        // Conditions not met, transition to error state.
                        Current_state = FAIL_SAFE;
                    }
                }
                // (Else, if SystemTime has not reached TIMEOUT_PBIT, remain in PBIT and keep sampling.)
            }
            break;

            case IDNTIF_RFM_FLIGHT_MODULE: // 2: Identify RFM - Flight module
            {

                // Check if master communication is OK by verifying if at least 3 master packets have been received.
                // (Commented out in this example.)
                 if(!filtred_comOkMaster){
                     filtred_comOkMaster = checkComOkMaster(FILTER_PROTOCOL);
                 }
                //Similarly, check the slave communication.
                if(!filtred_comOkSlave){
                    filtred_comOkSlave = checkComOkACU(FILTER_PROTOCOL);
                }
                // Use adc_filter to verify that HV_CUP reading is less than 1.07V.
                if(!filtred_hvCup_safe){
                    // ExpectedGreater == false: average should be less than threshold.
                    filtred_hvCup_safe = adc_filter(FILTER_GPIO, HV_CUP_ADC_CHANNEL, SAFETY_CUP_VOLT, false);
                }
                // If the received MSP message indicates function 0x84 and both communication and voltage conditions are met, transition to next state.
                if ((g_lastMspMsg.function == IDENTIFY_FUNC) && filtred_comOkMaster && filtred_hvCup_safe){
                     Current_state = IDNTIF_RFM_ACU; // Transition to ACU-RFM
                     filtred_comOkMaster = false;
                     filtred_safety_pin = false;
                     ////resetSystemTimer(); // Reset the timer
                }

                // Error check: if hvCup_safe indicates a false condition, transition to FAIL_SAFE.
                if(!filtred_hvCup_safe){
                     Current_state = FAIL_SAFE;
                }
            }
            break;

           case IDNTIF_RFM_ACU: // 3:
           {

               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               //safety_pin = gpio_stability_filter(FILTER_GPIO, FILTER_ACCURACY, P5V_SAFETY_GOOD_ISO, false);
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

               // At slave, send status each 1 sec (1Hz) -> Receive message 3.4.2.2 #39899128,
               // update decodeMspV2Message OSD Message with function 0x3004 if ACU OK and identification data received.
               // Check if master communication is OK (requires at least 5 master packets).
               if (!filtred_comOkMaster){
                   filtred_comOkMaster = checkComOkMaster(FILTER_PROTOCOL);
               }

//               // Process the received MSP message.
               if (g_lastMspMsg.function == IDENTIFY_RFM_ACU)
               {
                    Current_state = READY_FOR_SAFETY_PIN; // Transition to READY state
                    filtred_comOkMaster = false;
                    filtred_safety_pin = false;
                    ////resetSystemTimer(); // Reset the timer
               }

               // If the safety condition is not acceptable, transition to FAIL_SAFE.
               if (!filtred_safety_pin)
               {
                    Current_state = FAIL_SAFE;
               }
           }
           break;

           case READY_FOR_SAFETY_PIN: // 4: System Ready for safety pin
           {
               // Send status each 1 sec (1Hz) -> Receive message 3.4.2.2 #39899128 from state.
               // Send OSD message with function 0x3004 updating payload "ready for remove safety pin".
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               //safety_pin = gpio_stability_filter(FILTER_GPIO, FILTER_ACCURACY, P5V_SAFETY_GOOD_ISO, false);
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

               if(!filtred_comOkMaster){
                   filtred_comOkMaster = checkComOkMaster(FILTER_PROTOCOL);
               }

               if (!filtred_safety_pin){
                   Current_state = ARM_DELEY; // Transition to ARMING
                   filtred_comOkMaster = false;
                   filtred_safety_pin = false;
                   //resetSystemTimer(); // Reset the timer
               }

               if(!filtred_comOkMaster && (SystemTime >= TIMEOUT_ARM_READY_FOR_SAFETY_PIN))
                   Current_state = FAIL_SAFE;
           }
           break;

           case ARM_DELEY: // 5: Arming State
           {
               // Send status each 1 sec (1Hz) -> Receive message 3.4.2.2 #39899128 from state.
               // Send OSD message with function 0x3004 updating payload "ready for remove safety pin".
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               //safety_pin = gpio_stability_filter(FILTER_GPIO, 70, P5V_SAFETY_GOOD_ISO, false);
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

               if (!filtred_comOkMaster){
                   filtred_comOkMaster = checkComOkMaster(FILTER_PROTOCOL);
               }
               if (filtred_safety_pin){
                   Current_state = READY_FOR_SAFETY_PIN; // Transition to READY_FOR_SAFETY_PIN
               }
               else {
                   Current_state = WAIT_FOR_ARM; // Transition to WAIT_FOR_ARM
                   filtred_comOkMaster = false;
                   filtred_comOkSlave = false;
                   filtred_safety_pin = false;
                   //resetSystemTimer(); // Reset the timer
               }

               if (!filtred_comOkMaster && (SystemTime >= TIMEOUT_ARM_DELEY))
                   Current_state = FAIL_SAFE;
           }
           break;

           case WAIT_FOR_ARM: // 6: Waiting for Arm Command
           {
               // Send status each 1 sec (1Hz) -> Receive message 3.4.2.2 #39899128 from state.
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               //safety_pin = gpio_stability_filter(FILTER_GPIO, 70, P5V_SAFETY_GOOD_ISO, false);
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

               if (!filtred_comOkMaster){
                   filtred_comOkMaster = checkComOkMaster(FILTER_PROTOCOL);
               }
               // Placeholder: Implement logic to wait for an arm command.
               // For example, if a specific trigger is received:
               // if(stateTriggerByte == 0x07)
               //    Current_state = ARMING_HAND_SHAKE;
           }
           break;

           case ARMING_HAND_SHAKE: // 7: Handshake During Arming
           {
               // If handshake protocol is OK:
               //    Current_state = ARMING; // Transition to ARMING
               // If timeout occurs and handshake is not received:
               //    Current_state = DISARM; // Transition to DISARM
               // Placeholder: Process handshake messages.
           }
           break;

           case ARMING: // 8: Arming in Progress
           {
               // Start charge Cup SW1_2_EN, send clock to L2.
               filtred_hvCup_armed = adc_filter(FILTER_GPIO, HV_CUP_ADC_CHANNEL, ARMED_CUP_VOLT, true);
               // Placeholder: Implement arming procedures. Transition to ARMED when complete.
           }
           break;

           case ARMED: // 9: System Armed
           {
               // Send status each 1 sec (1Hz) -> Receive message 3.4.2.2 #39899128 from state.
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               //safety_pin = gpio_stability_filter(FILTER_GPIO, 70, P5V_SAFETY_GOOD_ISO, false);
               ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               filtred_hvCup_armed = adc_filter(FILTER_GPIO, HV_CUP_ADC_CHANNEL, ARMED_CUP_VOLT, true);
               if (!filtred_comOkMaster){
                   filtred_comOkMaster = checkComOkMaster(FILTER_PROTOCOL);
               }

               if (filtred_safety_pin){
                   Current_state = FAIL_SAFE; // Transition to FAIL_SAFE
               }

               // Placeholder: Return status to ACU via OSD message.
               // If disarm command received or no communication for 15 sec:
               //    Current_state = DISARM;
               // If detection command or acceleration threshold exceeded:
               //    Current_state = DET;
           }
           break;

           case DET: // 10: Detection / Detonation
           {
               // Activate TRIG_N and TRIG_P for 2ms in a loop.
           }
           break;

           case DISARM: // 11: Disarm State
           {
               // Placeholder: Process disarm commands.
           }
           break;

           case FAIL_SAFE: // 12: Fail Safe / Emergency State
           {
               // Placeholder: Maintain emergency state.
               Current_state = PBIT;
           }
           break;

           case SEC_DET_ATTEMPT: // 13: Secondary Detection Attempt
           {
               // Placeholder: Attempt alternative detection strategy.
           }
           break;

           case DUD: // 14: Inoperative or DUD State
           {
               // Placeholder: Handle inoperative system; log error and take safety measures.
           }
           break;

           default:
           {
               // If an undefined state is encountered, default back to PBIT.
               Current_state = PBIT;
           }
           break;
           } // end switch
    }
}
