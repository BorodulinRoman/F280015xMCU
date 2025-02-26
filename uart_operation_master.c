/******************************************************************************
 * File:    uart_operation_master.c
 * Device:  TMS320F280015x (or similar)
 *
 * Purpose:
 *   Implements the Master MSP protocol with a 32-byte payload.
 *   The message structure is as follows:
 *     Byte 0: '$'
 *     Byte 1: 'X'
 *     Byte 2: type         (for Master TX, '<' indicates a request)
 *     Byte 3: flag         (0)
 *     Bytes 4-5: function  (little-endian)
 *     Bytes 6-7: payloadSize (little-endian, value 32 normally)
 *     Bytes 8-39: payload  (32 bytes)
 *     Byte 40: CRC8 computed over bytes 2 to 39 using DVB-S2 polynomial (0xD5)
 *
 *   If the function code equals 0x84, an "empty" message is sent (payloadSize = 0).
 *
 *   The receive function decodes an incoming 41-byte MSP message (expected type '>')
 *   and validates its header and CRC.
 *
 * Author: Roman Borodulin (Updated for Master)
 * Created: 20/01/2025 (Updated Protocol)
 *****************************************************************************/

#include "uart_operation_master.h"
#include "globals_and_gpio.h"

// Global packet counter for Master transmissions.
volatile UINT16 masterPacketCount = 0;

// Global Master message instance.
MSPMsg_t g_lastMspMsg;

// (rxBufferMaster, txBufferMaster, rxIndexMaster, txIndexMaster, and txLengthMaster are
// defined in globals_and_gpio.c and declared as extern in globals_and_gpio.h)

// Internal message counter for transmitted messages.
static volatile UINT16 msgCounter = 0;

// -----------------------------------------------------------------------------
// Static CRC8 Calculation using DVB-S2 polynomial 0xD5 (internal linkage)
// -----------------------------------------------------------------------------
static UINT8 crc8_dvb_s1(UINT8 crc, unsigned char a)
{
    crc ^= a;
    int ii;  // Declare loop index variable outside loop header.
    for (ii = 0; ii < 8; ii++)
    {
        if (crc & 0x80)
        {
            crc = (crc << 1) ^ 0xD5;
        }
        else
        {
            crc = crc << 1;
        }
    }
    return crc;
}

// Computes the CRC8 over bytes 2 through 39 (38 bytes) of the message.
UINT8 computeCRC_MSP(const MSPMsg_t *msg)
{
    const uint8_t *ptr = ((const uint8_t *)msg) + 2; // Skip start and header.
    UINT length = 38;  // For a 41-byte packet, compute over bytes 2..39.
    UINT8 crc = 0;
    UINT i;  // Declare loop index variable outside loop header.
    for (i = 0; i < length; i++)
    {
        crc = crc8_dvb_s1(crc, ptr[i]);
    }
    return crc;
}

// -----------------------------------------------------------------------------
// Interrupt Service Routine: sciaRxISR
// Purpose: Handle RX FIFO interrupt events (SCIA).
// -----------------------------------------------------------------------------
__interrupt void sciaRxISR(void)
{
    UINT16 intStatus = SCI_getInterruptStatus(SCIA_BASE);

    if (intStatus & SCI_INT_RXFF)
    {
        // While RX FIFO not empty, read data out.
        while (SCI_getRxFIFOStatus(SCIA_BASE) != SCI_FIFO_RX0)
        {
            UINT16 received = SCI_readCharNonBlocking(SCIA_BASE);

            // Store in rxBufferMaster if space allows.
            if (rxIndexMaster < RX_MSG_LENGTH)
            {
                rxBufferMaster[rxIndexMaster++] = (UINT8)received;
            }
            else
            {
                // If exceeded buffer, reset the index.
                rxIndexMaster = 0;
            }

            // Check if a full (41-byte) message has been received.
            if (rxIndexMaster == RX_MSG_LENGTH)
            {
                uartMsgRxMaster();
                newDataReceivedMaster = true;
                rxIndexMaster = 0;

            }
        }
    }

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

// -----------------------------------------------------------------------------
// Interrupt Service Routine: sciaTxISR
// Purpose: Handle TX FIFO interrupt events (SCIA).
// -----------------------------------------------------------------------------
__interrupt void sciaTxISR(void)
{
    UINT16 intStatus = SCI_getInterruptStatus(SCIA_BASE);

    if (intStatus & SCI_INT_TXFF)
    {
        UINT16 fifoFree = 16U - SCI_getTxFIFOStatus(SCIA_BASE);

        // Fill TX FIFO while space remains and data is left to send.
        while ((fifoFree > 0) && (txIndexMaster < txLengthMaster))
        {
            SCI_writeCharNonBlocking(SCIA_BASE, txBufferMaster[txIndexMaster++]);
            fifoFree--;
        }

        if (txIndexMaster >= txLengthMaster)
        {
            SCI_disableInterrupt(SCIA_BASE, SCI_INT_TXFF);
            txIndexMaster = 0;
            txLengthMaster = 0;
        }
    }

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

// -----------------------------------------------------------------------------
// uartConfigMaster()
// Configures SCIA for UART communication.
// -----------------------------------------------------------------------------
void uartConfigMaster(void)
{
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    SCI_performSoftwareReset(SCIA_BASE);

    SCI_setConfig(SCIA_BASE,
                  DEVICE_LSPCLK_FREQ,
                  UART_BAUD_RATE,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));

    SCI_enableFIFO(SCIA_BASE);
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXFF | SCI_INT_TXFF);

    Interrupt_register(INT_SCIA_RX, sciaRxISR);
    Interrupt_register(INT_SCIA_TX, sciaTxISR);

    Interrupt_enable(INT_SCIA_RX);
    Interrupt_enable(INT_SCIA_TX);

    SCI_enableModule(SCIA_BASE);

    EINT;
    ERTM;
}

// -----------------------------------------------------------------------------
// uartMsgTxMaster()
// Builds the MSP message (41 bytes) and triggers transmission.
// -----------------------------------------------------------------------------
void uartMsgTxMaster(void)
{
    UINT i;  // Declare loop index variable outside loop header.

    // Build the MSP message.
    g_lastMspMsg.start = MSP_START_CHAR;
    g_lastMspMsg.header = MSP_HEADER_CHAR;
    g_lastMspMsg.type = MSP_TYPE_REQUEST;  // For Master TX, use '<'
    g_lastMspMsg.flag = MSP_DEFAULT_FLAG;
    g_lastMspMsg.function = MSP_FUNCTION_ID;

    // If function code is 0x84, send an "empty" message.
    if(g_lastMspMsg.function == 0x0084)
    {
        g_lastMspMsg.payloadSize = 0;


        for(i = 0; i < MSP_PAYLOAD_SIZE; i++)
        {
            g_lastMspMsg.payload[i] = 0;
        }
    }
    else
    {
        g_lastMspMsg.payloadSize = MSP_PAYLOAD_SIZE;
        //masterBuildPayload(&receivedMsg);
        for(i = 0; i < MSP_PAYLOAD_SIZE; i++)
        {
            g_lastMspMsg.payload[i] = (uint8_t)i;
        }
    }

    // Compute CRC over bytes 2 to 39.
    g_lastMspMsg.crc = computeCRC_MSP(&g_lastMspMsg);

    // Copy the 41-byte message into the TX buffer.
    {
        UINT j;  // Declare loop index variable outside loop header.
        for (j = 0; j < sizeof(MSPMsg_t); j++)
        {
            txBufferMaster[j] = ((const uint8_t *)&g_lastMspMsg)[j];
        }
        txLengthMaster = sizeof(MSPMsg_t);
    }

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXFF);
}

// -----------------------------------------------------------------------------
// uartMsgRxMaster()
// Receives and decodes a 41-byte MSP message for Master.
// -----------------------------------------------------------------------------
void uartMsgRxMaster(void)
{
    if (newDataReceivedMaster)
    {
        MSPMsg_t receivedMsg;
        UINT i;  // Declare loop index variable outside loop header.
        for (i = 0; i < sizeof(MSPMsg_t); i++)
        {
            ((uint8_t *)&receivedMsg)[i] = rxBufferMaster[i];
        }

        // Validate header and type (for responses, Master expects type '>')
        if (receivedMsg.start == MSP_START_CHAR &&
            receivedMsg.header == MSP_HEADER_CHAR &&
            receivedMsg.type == '>')
        {
            // Validate CRC.
            UINT8 computed_crc = computeCRC_MSP(&receivedMsg);
            if (computed_crc == receivedMsg.crc)
            {
                masterPacketCount++;
                masterDecodePayload(&receivedMsg);
                // Successfully decoded the message.
                // Process receivedMsg.function, receivedMsg.payloadSize, and receivedMsg.payload as needed.
            }
            else
            {
                // CRC error: handle as needed.
            }
        }
        else
        {

        }
        newDataReceivedMaster = false;
    }
}

// Builds the payload for the Master message.
// If the function code is 0x84, it produces an "empty" message (payloadSize = 0).
void masterBuildPayload(MSPMsg_t *msg)
{
    UINT i;  // Declare loop index variable outside the loop header.

    if (msg->function == 0x0084)
    {
        msg->payloadSize = 0;
        for (i = 0; i < MSP_PAYLOAD_SIZE; i++)
        {
            msg->payload[i] = 0;
        }
    }
    else
    {
//        msg->payloadSize = MSP_PAYLOAD_SIZE;
//        msg->payload[0] = 0xAAAA
//        msg->payload[1] = 0xAAAA
//        msg->payload[2] = 0xAAAA
//        msg->payload[3] = 0xAAAA
//        msg->payload[4] = g_lastMspMsg.payload[4]
//        msg->payload[5] = g_lastMspMsg.payload[5]
//        msg->payload[6] = g_lastMspMsg.payload[6]
//        msg->payload[7] = g_lastMspMsg.payload[7]
//        msg->payload[8] = g_lastMspMsg.payload[8]
//        msg->payload[9] = g_lastMspMsg.payload[9]
//        msg->payload[10] = g_lastMspMsg.payload[10]
//        msg->payload[11] = g_lastMspMsg.payload[11]
        for (i = 0; i < MSP_PAYLOAD_SIZE; i++)
        {
            msg->payload[i] = (uint8_t)(i + 1);  // Sample data; replace with your actual data.
        }
    }
    // Update the CRC after building the payload.
    msg->crc = computeCRC_MSP(msg);
}

// Decodes the payload from a received Master message.
// Replace the processing code below with your application-specific decoding.
void masterDecodePayload(const MSPMsg_t *msg)
{
    if (msg->payloadSize == 0)
    {
        // Empty payload: handle accordingly (e.g., no data to process).
    }
    else
    {
        UINT i;  // Declare loop index variable outside the loop header.
        // Example: iterate over each payload byte and process it.
        for (i = 0; i < msg->payloadSize; i++)
        {
            // Process each byte (for example, store or print the value).
            // For instance: processMasterData(msg->payload[i]);
        }
    }
}

// -----------------------------------------------------------------------------
// checkComOkMaster()
// Returns true if masterPacketCount is at least filter_protocol, then resets the counter.
// -----------------------------------------------------------------------------
bool checkComOkMaster(uint16_t filter_protocol)
{
    if (masterPacketCount >= filter_protocol)
    {
        masterPacketCount = 0;
        return true;
    }
    return false;
}
