//#ifndef UART_OPERATION_SLAVE_H_
//#define UART_OPERATION_SLAVE_H_
//
//#include "device.h"
//#include "driverlib.h"
//#include "GenericTypeDefs.h"
//
//#define UART_BAUD_RATE 115200
//#define DEVICE_GPIO_PIN_SCIRXDB     23U
//#define DEVICE_GPIO_PIN_SCITXDB     40U
//#define DEVICE_GPIO_CFG_SCIRXDB     GPIO_23_SCIB_RX
//#define DEVICE_GPIO_CFG_SCITXDB     GPIO_40_SCIB_TX
//
//#define TX_S_MSG_HEADER_BYTE0 0xB5
//#define TX_S_MSG_HEADER_BYTE1 0xBD
//#define TX_S_MSG_LENGTH 25
//#define RX_S_MSG_LENGTH 25
//
//extern volatile UINT8 rxBufferSlave[RX_MSG_LENGTH];
//extern volatile bool newDataReceivedSlave;
//
//void uartConfigSlave(void);
//void uartMsgTxSlave(void);
//void uartMsgRxSlave(void);
//UINT16 crcCalculationSlave(volatile UINT8 *data, UINT length);
//
//
//#endif /* UART_OPERATION_SLAVE_H_ */
