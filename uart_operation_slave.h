/******************************************************************************
 * File:        uart_operation_slave.h
 * Description: UART slave with MSP V2 decoding for a total of 29 bytes
 *              (8-byte header + 20-byte payload + 1-byte CRC).
 ******************************************************************************/

#ifndef UART_OPERATION_SLAVE_H_
#define UART_OPERATION_SLAVE_H_

#include "GenericTypeDefs.h"
#include "device.h"
#include "driverlib.h"
#include "sci.h"
#include "gpio.h"
#include "common_defines.h"

//
// SCI-B pins (example):
//
#define UART_BAUD_RATE_SLAVE     115200
#define DEVICE_GPIO_PIN_SCIRXDB  23U
#define DEVICE_GPIO_PIN_SCITXDB  40U
#define DEVICE_GPIO_CFG_SCIRXDB  GPIO_23_SCIB_RX
#define DEVICE_GPIO_CFG_SCITXDB  GPIO_40_SCIB_TX

//
// The total message is 29 bytes for MSP V2 in your scenario:
//   - 8-byte header
//   - 20-byte payload
//   - 1-byte CRC
//
#define RX_S_MSG_LENGTH          29

// We'll define a 29-byte TX if you want to respond in the same format
// or keep it separate. Example below:
#define TX_S_MSG_LENGTH          29

// Example headers if you do respond
#define TX_S_MSG_HEADER_BYTE0    0xE6
#define TX_S_MSG_HEADER_BYTE1    0xEE

//
// Flag to indicate a fully decoded, valid MSP message
//
extern volatile bool newDataReceivedSlave;
extern volatile UINT8 rxBufferSlave[RX_S_MSG_LENGTH];

//
// MSP V2 structure for your 29-byte fixed message
//   total = 8 + 20 + 1 = 29
//
typedef struct {
    UINT8  mspType;       // e.g. '<','>','!'
    UINT8  flags;         // offset 3
    UINT16 function;      // offset 4..5, little-endian
    UINT16 payloadSize;   // offset 6..7, 20 in your case
    UINT8  payload[20];   // offset 8..27
} mspV2Message_t;

extern mspV2Message_t g_lastMspMsg;

//
// Public functions
//
void uartConfigSlave(void);
void uartMsgTxSlave(void);  // If you want to send a 29-byte response
bool decodeMspV2Message(const UINT8 *packet, mspV2Message_t *outMsg, UINT16 length);

#endif /* UART_OPERATION_SLAVE_H_ */
