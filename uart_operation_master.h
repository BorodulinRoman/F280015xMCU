/****************************************************************************************
 * Project:     ESA L1
 * Device:      F2800157
 * File:        uart_operation_master.h
 * Declaration: Includes declarations of all global variables and functions related
 *              to the UART Master model
 * Author:      Roman Borodulin
 * Created:     20/01/2025
 *****************************************************************************************/

#ifndef UART_OPERATION_MASTER_H_
#define UART_OPERATION_MASTER_H_

#include "GenericTypeDefs.h"
#include "device.h"
#include "driverlib.h"
#include "sci.h"
#include "gpio.h"
#include "common_defines.h"

#define UART_BAUD_RATE 115200
#define DEVICE_GPIO_PIN_SCIRXDA     28U
#define DEVICE_GPIO_PIN_SCITXDA     29U
#define DEVICE_GPIO_CFG_SCIRXDA     GPIO_28_SCIA_RX
#define DEVICE_GPIO_CFG_SCITXDA     GPIO_29_SCIA_TX

#define TX_M_MSG_HEADER_BYTE0 0xE5
#define TX_M_MSG_HEADER_BYTE1 0xED
#define TX_M_MSG_LENGTH 20
#define RX_M_MSG_LENGTH 20

extern volatile UINT8 rxBufferMaster[RX_MSG_LENGTH];
extern volatile bool newDataReceivedMaster;

void uartConfigMaster(void);
void uartMsgTxMaster(void);
void uartMsgRxMaster(void);
UINT16 crcCalculationMaster(volatile UINT8 *data, UINT length);

#endif /* UART_OPERATION_MASTER_H_ */

