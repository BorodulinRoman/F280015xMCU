#ifndef GLOBALS_AND_GPIO_H
#define GLOBALS_AND_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "device.h"
#include "driverlib.h"

// OUT GPIO Configurations
#define CS_MCU1               0U // ACCELEROMETER KX134ACR-LBZE2
#define MOSI_MCU1             2U // ACCELEROMETER KX134ACR-LBZE2
#define SCLK_MCU1             3U // ACCELEROMETER KX134ACR-LBZE2
#define SW1_EN_GPIO           7U
#define GCLK_TO_MCU2          16U
#define TRIG_IZUM_P           32U
#define TRIG_IZUM_N           33U

// IN GPIO Configurations
#define MISO_MCU1             1U // ACCELEROMETER KX134ACR-LBZE2
#define P3V3_GOOD             24U
//#define P5V_SAFETY_GOOD_ISO   29U


// Global Variables
extern volatile int Current_state;       // Tracks the current state in the state machine
extern volatile bool newDataReceivedMaster;
extern volatile bool newDataReceivedSlave;

// STATE MACHINE DEFINITIONS
#define TIMER_PERIOD_US                             10000.0f
#define WAKE_UP                                     20
#define TIMEOUT_PBIT                                100
#define TIMEOUT_ARM_DELAY                           10000
#define TIMEOUT_WAIT_FOR_ARM_NO_ACU_COMMUNICATION   15000
#define TIMEOUT_ARM_HANDSHAKE                       15000
#define TIMEOUT_ARMING                              2000
#define TIMEOUT_ARMED_NO_ACU_COMMUNICATION          15000
#define TIMEOUT_DISARM                              30000
#define TIMEOUT_DISARM_NO_ACU_COMMUNICATION         15000


// Function Prototypes
void setupGPIOs(void);

#endif // GLOBALS_AND_GPIO_H
