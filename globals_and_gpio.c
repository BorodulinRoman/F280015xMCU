#include "globals_and_gpio.h"

// Global Variables

volatile int Current_state;
volatile bool newDataReceivedMaster = false;
volatile bool newDataReceivedSlave = false;

// Function to set up the GPIOs
void setupGPIOs(void)
{
    // Configure OUT GPIOs
    GPIO_setPinConfig(GPIO_0_GPIO0);  // CS_MCU1
    GPIO_setPinConfig(GPIO_2_GPIO2);  // MOSI_MCU1
    GPIO_setPinConfig(GPIO_3_GPIO3);  // SCLK_MCU1
    GPIO_setPinConfig(GPIO_7_GPIO7);  // SW1_EN_GPIO
    GPIO_setPinConfig(GPIO_16_GPIO16); // GCLK_TO_MCU2
    GPIO_setPinConfig(GPIO_32_GPIO32); // TRIG_IZUM_P
    GPIO_setPinConfig(GPIO_33_GPIO33); // TRIG_IZUM_N

    GPIO_setDirectionMode(CS_MCU1, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(MOSI_MCU1, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(SCLK_MCU1, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(SW1_EN_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(GCLK_TO_MCU2, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(TRIG_IZUM_P, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(TRIG_IZUM_N, GPIO_DIR_MODE_OUT);

    // Set default output levels
    GPIO_writePin(CS_MCU1, 1);
    GPIO_writePin(MOSI_MCU1, 0);
    GPIO_writePin(SCLK_MCU1, 0);
    GPIO_writePin(SW1_EN_GPIO, 1);
    GPIO_writePin(GCLK_TO_MCU2, 0);
    GPIO_writePin(TRIG_IZUM_P, 0);
    GPIO_writePin(TRIG_IZUM_N, 0);

    // Configure IN GPIOs
    GPIO_setPinConfig(GPIO_1_GPIO1);  // MISO_MCU1
    GPIO_setPinConfig(GPIO_24_GPIO24); // P3V3_GOOD
    //GPIO_setPinConfig(GPIO_29_GPIO29); // P5V_SAFETY_GOOD_ISO (if needed)

    GPIO_setDirectionMode(MISO_MCU1, GPIO_DIR_MODE_IN);
    GPIO_setDirectionMode(P3V3_GOOD, GPIO_DIR_MODE_IN);
    //GPIO_setDirectionMode(P5V_SAFETY_GOOD_ISO, GPIO_DIR_MODE_IN);

    GPIO_setQualificationMode(MISO_MCU1, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(P3V3_GOOD, GPIO_QUAL_ASYNC);
    //GPIO_setQualificationMode(P5V_SAFETY_GOOD_ISO, GPIO_QUAL_ASYNC);
}
