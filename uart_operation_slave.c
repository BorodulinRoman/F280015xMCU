/******************************************************************************
 * File:        uart_operation_slave.c
 * Description: UART slave using SCI-B, decoding MSP V2 messages of total 29 bytes:
 *              [0..7]=header, [8..27]=20-byte payload, [28]=CRC.
 *              If valid, sets newDataReceivedSlave=true and stores to g_lastMspMsg.
 ******************************************************************************/

#include "uart_operation_slave.h"
#include <string.h>

volatile bool  newDataReceivedSlave = false;
volatile UINT8 rxBufferSlave[RX_S_MSG_LENGTH];
mspV2Message_t g_lastMspMsg;

// Local variables for reception and transmission
static volatile UINT16 rxIndexSlave = 0;
static volatile UINT8  txBufferSlave[TX_S_MSG_LENGTH];
static volatile UINT16 txIndexSlave  = 0;
static volatile UINT16 txLengthSlave = 0;

static volatile UINT16 msgCounterSlave = 0;  // For response counters

// Forward declarations
__interrupt void scibRxISR(void);
__interrupt void scibTxISR(void);

// Static CRC implementation (dvb_s2)
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    int ii;
    crc ^= a;
    for (ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// Decode MSP V2 message
bool decodeMspV2Message(const UINT8 *packet, mspV2Message_t *outMsg, UINT16 length)
{
// 24 58 3C 00 10 11 14 00 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 F0 - Example for Test
    int i;
    if (length != RX_S_MSG_LENGTH) return false;

    // Validate header
    if (packet[0] != '$' || packet[1] != 'X') return false;

    // Calculate and validate CRC
    uint8_t expectedCrc = packet[28];
    uint8_t calcCrc = 0;
    for (i = 0; i < 28; i++) {
        calcCrc = crc8_dvb_s2(calcCrc, packet[i]);
    }
    if ((calcCrc & 0xFF) != expectedCrc) return false;

    // Parse the message
    outMsg->mspType    = packet[2];
    outMsg->flags      = packet[3];
    outMsg->function   = (packet[5] << 8) | packet[4]; // Little-endian
    outMsg->payloadSize= (packet[7] << 8) | packet[6];
    memcpy((void *)outMsg->payload, &packet[8], 20); // Cast to non-volatile

    return true;
}

// UART Configuration
void uartConfigSlave(void)
{
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDB);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDB, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDB, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDB, GPIO_QUAL_ASYNC);

    SCI_performSoftwareReset(SCIB_BASE);
    SCI_setConfig(SCIB_BASE, DEVICE_LSPCLK_FREQ, UART_BAUD_RATE_SLAVE,
                  SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE);

    SCI_enableFIFO(SCIB_BASE);
    SCI_setFIFOInterruptLevel(SCIB_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1);
    SCI_resetTxFIFO(SCIB_BASE);
    SCI_resetRxFIFO(SCIB_BASE);

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_RXFF | SCI_INT_TXFF);

    Interrupt_register(INT_SCIB_RX, scibRxISR);
    Interrupt_register(INT_SCIB_TX, scibTxISR);
    Interrupt_enable(INT_SCIB_RX);
    Interrupt_enable(INT_SCIB_TX);

    SCI_enableModule(SCIB_BASE);

    EINT;
    ERTM;
}

// Send the decoded message back
void uartMsgTxSlave(void)
{
    int i;
    txBufferSlave[0] = TX_S_MSG_HEADER_BYTE0;
    txBufferSlave[1] = TX_S_MSG_HEADER_BYTE1;

    txBufferSlave[2] = (msgCounterSlave >> 8) & 0xFF;
    txBufferSlave[3] = msgCounterSlave & 0xFF;
    msgCounterSlave++;

    memcpy((void *)&txBufferSlave[4], g_lastMspMsg.payload, 20); // Cast to non-volatile

    uint8_t crc = 0;
    for (i = 0; i < 28; i++) {
        crc = crc8_dvb_s2(crc, txBufferSlave[i]);
    }
    txBufferSlave[28] = crc;

    txIndexSlave  = 0;
    txLengthSlave = TX_S_MSG_LENGTH;

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF);
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_TXFF);
}

// RX ISR
__interrupt void scibRxISR(void)
{
    while (SCI_getRxFIFOStatus(SCIB_BASE) != SCI_FIFO_RX0)
    {
        rxBufferSlave[rxIndexSlave++] = SCI_readCharNonBlocking(SCIB_BASE);
        if (rxIndexSlave == RX_S_MSG_LENGTH)
        {
            if (decodeMspV2Message((const UINT8 *)rxBufferSlave, &g_lastMspMsg, RX_S_MSG_LENGTH))
            {
                newDataReceivedSlave = true;
                uartMsgTxSlave();
            }
            rxIndexSlave = 0;
        }
    }

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

// TX ISR
__interrupt void scibTxISR(void)
{
    while (txIndexSlave < txLengthSlave && SCI_getTxFIFOStatus(SCIB_BASE) < 16)
    {
        SCI_writeCharNonBlocking(SCIB_BASE, txBufferSlave[txIndexSlave++]);
    }

    if (txIndexSlave >= txLengthSlave)
    {
        SCI_disableInterrupt(SCIB_BASE, SCI_INT_TXFF);
    }

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
