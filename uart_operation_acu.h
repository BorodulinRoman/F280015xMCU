#ifndef UART_OPERATION_ACU_H_
#define UART_OPERATION_ACU_H_

#include "GenericTypeDefs.h"
#include "device.h"
#include "driverlib.h"
#include "sci.h"
#include "gpio.h"
#include "globals_and_gpio.h"
#include "common_defines.h"
#include "uart_operation_master.h"  // To share MSPMsg_t and computeCRC_MSP()

// ACU UART configuration
#define DEVICE_GPIO_PIN_SCIRXDB  23U
#define DEVICE_GPIO_PIN_SCITXDB  40U
#define DEVICE_GPIO_CFG_SCIRXDB  GPIO_23_SCIB_RX
#define DEVICE_GPIO_CFG_SCITXDB  GPIO_40_SCIB_TX

#define TX_MSG_HEADER_BYTE0_ACU 0xE5
#define TX_MSG_HEADER_BYTE1_ACU 0xED

// For ACU, we'll use the response type '>' when transmitting.
#define MSP_TYPE_RECEIVE       '>'

extern volatile UINT8 rxBufferACU[RX_MSG_LENGTH];
extern volatile bool newDataReceivedACU;


// Global flag to enable ACU transmissions.
extern volatile bool acuEnableTx;
// Global packet counter for ACU transmissions.
extern volatile UINT16 acuPacketCount;

void uartConfigACU(void);
void uartMsgTxACU(void);
void uartMsgRxACU(void);

// Updated CRC function: returns 8-bit value.
// (We share computeCRC_MSP() from the Master module if identical.)
UINT8 crcCalculationACU(volatile UINT8 *data, UINT length);
void acuBuildPayload(MSPMsg_t *msg);
void acuDecodePayload(const MSPMsg_t *msg);
static UINT8 crc8_dvb_s2(UINT8 crc, unsigned char a);

bool checkComOkACU(uint16_t filter_protocol);

#endif /* UART_OPERATION_ACU_H_ */
