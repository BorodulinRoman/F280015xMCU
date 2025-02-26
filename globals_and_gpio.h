#ifndef GLOBALS_AND_GPIO_H
#define GLOBALS_AND_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "device.h"
#include "driverlib.h"
#include "GenericTypeDefs.h"
#include "common_defines.h"

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
#define HV_CUP_ADC_CHANNEL    29U

#define SAFETY_CUP_VOLT       1070U
#define ARMED_CUP_VOLT        2040U
#define FILTER_GPIO           10U   // Number of samples to take
#define FILTER_ACCURACY       70U   // Require at least 70% matching
#define FILTER_PROTOCOL       1U

// Global Variables
extern volatile int Current_state;       // Tracks the current state in the state machine
extern volatile bool newDataReceivedMaster;
extern volatile bool newDataReceivedSlave;

// Declare Master RX/ TX buffer variables to be shared with the Master module:
extern volatile UINT8 rxBufferMaster[41];   // Our MSP message is 41 bytes
extern volatile UINT16 rxIndexMaster;
extern volatile UINT8 txBufferMaster[41];     // TX buffer for a 41-byte packet
extern volatile UINT16 txIndexMaster;
extern volatile UINT16 txLengthMaster;



#define TIMER_PERIOD_US                             10000.0f
#define WAKE_UP                                     20
#define TIMEOUT_PBIT                                100
#define TIMEOUT_ARM_DELEY                           10000
#define TIMEOUT_ARM_READY_FOR_SAFETY_PIN            15000
#define TIMEOUT_ARM_HANDSHAKE                       15000
#define TIMEOUT_ARMING                              2000
#define TIMEOUT_ARMED_NO_ACU_COMMUNICATION          15000
#define TIMEOUT_DISARM                              30000
#define TIMEOUT_DISARM_NO_ACU_COMMUNICATION         15000

// Function Prototypes
void setupGPIOs(void);

#endif // GLOBALS_AND_GPIO_H
