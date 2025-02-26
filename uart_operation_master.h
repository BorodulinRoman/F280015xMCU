#ifndef UART_OPERATION_MASTER_H_
#define UART_OPERATION_MASTER_H_

#include "GenericTypeDefs.h"
#include "device.h"
#include "driverlib.h"
#include "sci.h"
#include "gpio.h"
#include "globals_and_gpio.h"

// Master UART configuration

#define DEVICE_GPIO_PIN_SCIRXDA 28U
#define DEVICE_GPIO_PIN_SCITXDA 29U
#define DEVICE_GPIO_CFG_SCIRXDA GPIO_28_SCIA_RX
#define DEVICE_GPIO_CFG_SCITXDA GPIO_29_SCIA_TX

// Global Master message instance (last built packet)
extern MSPMsg_t g_lastMspMsg;

// Global packet counter for Master transmissions.
extern volatile UINT16 masterPacketCount;

void uartConfigMaster(void);
void uartMsgTxMaster(void);
void uartMsgRxMaster(void);

// Computes the CRC8 (using DVB-S2 polynomial 0xD5) over a given data array.
UINT8 computeCRC_MSP(const MSPMsg_t *msg);
void masterBuildPayload(MSPMsg_t *msg);
void masterDecodePayload(const MSPMsg_t *msg);
// Returns true if masterPacketCount is at least filter_protocol, then resets the counter.
bool checkComOkMaster(uint16_t filter_protocol);

#endif /* UART_OPERATION_MASTER_H_ */
