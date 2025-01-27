/******************************************************************************
 * File:        uart_operation_slave.c
 * Description: UART slave using SCI-B, decoding MSP V2 messages of total 29 bytes:
 *              [0..7]=header, [8..27]=20-byte payload, [28]=CRC.
 *              If valid, sets newDataReceivedSlave=true and stores to g_lastMspMsg.
 ******************************************************************************/
#include "GenericTypeDefs.h"
#include "uart_operation_slave.h"
#include <string.h>

volatile bool  newDataReceivedSlave = false;
volatile UINT8 rxBufferSlave[RX_S_MSG_LENGTH];
mspV2Message_t g_lastMspMsg;

// Local reception
static volatile UINT16 rxIndexSlave = 0;

// Transmission
static volatile UINT8  txBufferSlave[TX_S_MSG_LENGTH];
static volatile UINT16 txIndexSlave  = 0;
static volatile UINT16 txLengthSlave = 0;

static volatile UINT16 msgCounterSlave = 0;

// Forward declarations
__interrupt void scibRxISR(void);
__interrupt void scibTxISR(void);

//
// dvb_s2 CRC for MSP V2
//
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

//
// decodeMspV2Message - for total=29 bytes:
//   [0]='$',[1]='X',[2]=type,[3]=flags,[4..5]=func,[6..7]=size,[8..27]=payload(20),
//   [28]=CRC
//
bool decodeMspV2Message(const UINT8 *packet, mspV2Message_t *outMsg, UINT16 length)
{
    if (length != 29) {
        return false;
    }

    // Check for '$','X'
    if (packet[0] != '$' || packet[1] != 'X') {
        return false;
    }

    // The CRC is at byte 28
    uint8_t expectedCrc = packet[28];

    // Calculate dvb_s2 over bytes [2..27] (26 bytes total)
    uint8_t calcCrc = 0;
    int i;
    for (i = 0; i < 28; i++) {
        calcCrc = crc8_dvb_s2(calcCrc, packet[i]);
    }

    if ((calcCrc & 0xFF) != expectedCrc) {
        return false;
    }

    // Now parse
    outMsg->mspType    = packet[2];   // '<','>','!'
    outMsg->flags      = packet[3];
    outMsg->function   = (packet[5] << 8) | packet[4]; // little-endian
    outMsg->payloadSize= (packet[7] << 8) | packet[6]; // also LE, expected 20
    // Copy the 20-byte payload
    memcpy(outMsg->payload, &packet[8], 20);

    return true;
}

//---------------------------------------------------------------------------
// uartConfigSlave
//---------------------------------------------------------------------------
void uartConfigSlave(void)
{
    // SCIB: 23=RX, 40=TX
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDB);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDB, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDB, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDB, GPIO_QUAL_ASYNC);

    SCI_performSoftwareReset(SCIB_BASE);

    SCI_setConfig(SCIB_BASE,
                  DEVICE_LSPCLK_FREQ,
                  UART_BAUD_RATE_SLAVE,
                  (SCI_CONFIG_WLEN_8 |
                   SCI_CONFIG_STOP_ONE |
                   SCI_CONFIG_PAR_NONE));

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

//---------------------------------------------------------------------------
// uartMsgTxSlave
//   Example of sending a 29-byte message. If you want it to match MSP v2,
//   you'd do $X... . For demonstration, we do something simpler.
//---------------------------------------------------------------------------
void uartMsgTxSlave(void)
{
    UINT16 i;

    // Fill a 29-byte packet
    // e.g. Bytes[0..1] = 0xE6,0xEE
    txBufferSlave[0] = TX_S_MSG_HEADER_BYTE0;
    txBufferSlave[1] = TX_S_MSG_HEADER_BYTE1;

    // Simple counter in [2..3]
    txBufferSlave[2] = (UINT8)((msgCounterSlave >> 8) & 0xFF);
    txBufferSlave[3] = (UINT8)(msgCounterSlave & 0xFF);
    msgCounterSlave++;

    // For [4..27], fill random or placeholders, last byte [28] for CRC
    for(i = 4; i < (TX_S_MSG_LENGTH - 1); i++)
    {
        txBufferSlave[i] = (UINT8)i;
    }
    // If you want dvb_s2 here, do that. We'll just store 0xBB:
    txBufferSlave[TX_S_MSG_LENGTH - 1] = 0xBB;

    // Start TX
    txIndexSlave  = 0;
    txLengthSlave = TX_S_MSG_LENGTH;

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF);
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_TXFF);
}

//---------------------------------------------------------------------------
// scibRxISR
//   Reads 29 bytes, then calls decodeMspV2Message
//---------------------------------------------------------------------------
__interrupt void scibRxISR(void)
{
    UINT16 intStatus = SCI_getInterruptStatus(SCIB_BASE);
    bool ok = false;
    if(intStatus & SCI_INT_RXFF)
    {
        while(SCI_getRxFIFOStatus(SCIB_BASE) != SCI_FIFO_RX0)
        {
            UINT16 received = SCI_readCharNonBlocking(SCIB_BASE);

            if(rxIndexSlave < RX_S_MSG_LENGTH)
            {
                rxBufferSlave[rxIndexSlave++] = (UINT8)received;
            }
            else
            {
                rxIndexSlave = 0;
            }

            // If we have 29 bytes total
            if(rxIndexSlave == RX_S_MSG_LENGTH)
            {
                mspV2Message_t tmp;
                ok = decodeMspV2Message((const UINT8 *)rxBufferSlave, &tmp, RX_S_MSG_LENGTH);
                if(ok)
                {
                    g_lastMspMsg = tmp;
                    newDataReceivedSlave = true;
                }
                else
                {
                    newDataReceivedSlave = false;
                }
                rxIndexSlave = 0;
            }
        }
    }

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//---------------------------------------------------------------------------
// scibTxISR
//   Transmit up to 29 bytes in txBufferSlave
//---------------------------------------------------------------------------
__interrupt void scibTxISR(void)
{
    UINT16 intStatus = SCI_getInterruptStatus(SCIB_BASE);

    if(intStatus & SCI_INT_TXFF)
    {
        UINT16 fifoFree = 16U - SCI_getTxFIFOStatus(SCIB_BASE);

        while((fifoFree > 0) && (txIndexSlave < txLengthSlave))
        {
            SCI_writeCharNonBlocking(SCIB_BASE, txBufferSlave[txIndexSlave++]);
            fifoFree--;
        }

        if(txIndexSlave >= txLengthSlave)
        {
            SCI_disableInterrupt(SCIB_BASE, SCI_INT_TXFF);
            txIndexSlave = 0;
            txLengthSlave = 0;
        }
    }

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
