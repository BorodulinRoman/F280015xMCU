/******************************************************************************
 * File:    uart_operation_acu.c
 * Device:  TMS320F280015x (or similar)
 *
 * Purpose:
 *   Implements the ACU MSP protocol using a 41-byte message with a 32-byte payload.
 *   The message structure is as follows:
 *     Byte 0: TX_MSG_HEADER_BYTE0_ACU (0xE5)
 *     Byte 1: TX_MSG_HEADER_BYTE1_ACU (0xED)
 *     Byte 2: type         (for ACU TX, use MSP_TYPE_RECEIVE, i.e. '>')
 *     Byte 3: flag         (0)
 *     Bytes 4-5: function  (little-endian)
 *     Bytes 6-7: payloadSize (little-endian; normally 32)
 *     Bytes 8-39: payload  (32 bytes)
 *     Byte 40: CRC8 computed over bytes 2 to 39 using DVB-S2 polynomial (0xD5)
 *
 *   For function code 0x84, an "empty" message is sent (payloadSize = 0).
 *
 * Author: Roman Borodulin (Updated for ACU)
 * Created: 20/01/2025 (Updated Protocol)
 *****************************************************************************/

// Forward declarations for ACU ISR functions.
__interrupt void scibRxISR(void);
__interrupt void scibTxISR(void);

#include "uart_operation_acu.h"

// Global flag to enable ACU transmission.
volatile bool acuEnableTx = false;
// Global packet counter for ACU transmissions.
volatile UINT16 acuPacketCount = 0;

volatile UINT8 rxBufferACU[RX_MSG_LENGTH];
volatile bool newDataReceivedACU = false;

// ACU RX decoders
volatile UINT16 acu_header1 = 0;             // Initialize as needed
volatile UINT16 acu_header2 = 0;
volatile UINT16 rfm_logic_1_ID = 0;
volatile UINT16 rfm_logic_2_ID = 0;
volatile UINT16 acu_logic_1_ID = 0;
volatile UINT16 acu_logic_2_ID = 0;
volatile UINT8  rfm_status_and_impact = 0;
volatile UINT8  rfm_logic_sate = 0;
volatile UINT16 acu_last_received_counter = 0;
volatile UINT16 comm_counter = 0;

// ACU TX decoders (use unique names to avoid conflict with RX decoders)
volatile UINT16 acu_header1_tx = 0xAAAA;
volatile UINT16 acu_header2_tx = 0xAAAA;
volatile UINT16 mc1_logic_1_ID = 0;
volatile UINT16 mc1_logic_2_ID = 0;
volatile UINT16 mc2_logic_1_ID = 0;
volatile UINT16 mc2_logic_2_ID = 0;
volatile UINT8  mc_logic_sate = 1;
volatile UINT16 mc_last_received_counter = 0;
volatile UINT16 comm_counter_tx = 0;
volatile UINT8  capacitor_voltage = 0;

// Static TX buffers and counters for ACU.
static volatile UINT8 txBufferACU[TX_MSG_LENGTH];
static volatile UINT16 txIndexACU = 0;
static volatile UINT16 txLengthACU = 0;

// Message counter for ACU transmissions.
static volatile UINT16 acuMsgCounter = 0;

// -----------------------------------------------------------------------------
// Static CRC8 Calculation using DVB-S2 polynomial 0xD5 (internal linkage)
// -----------------------------------------------------------------------------
static UINT8 crc8_dvb_s2(UINT8 crc, unsigned char a)
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

UINT8 crcCalculationACU(volatile UINT8 *data, UINT length)
{
    UINT8 crc = 0;  // Start with 0.
    UINT i;         // Declare loop index variable outside loop header.
    for (i = 0; i < length; i++)
    {
        crc = crc8_dvb_s2(crc, data[i]);
    }
    return crc;
}

// -----------------------------------------------------------------------------
// uartConfigACU()
// Configures SCIB for UART communication at 115200 baud, 8N1, FIFO mode.
// -----------------------------------------------------------------------------
void uartConfigACU(void)
{
    // 1) Configure GPIOs for SCIB.
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDB);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDB);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDB, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDB, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDB, GPIO_QUAL_ASYNC);

    // 2) Reset/initialize SCIB.
    SCI_performSoftwareReset(SCIB_BASE);

    // 3) Set configuration: 115200 baud, 8N1.
    SCI_setConfig(SCIB_BASE,
                  DEVICE_LSPCLK_FREQ,
                  UART_BAUD_RATE,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));

    // 4) Enable FIFO and set interrupt trigger levels.
    SCI_enableFIFO(SCIB_BASE);
    SCI_setFIFOInterruptLevel(SCIB_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1);
    SCI_resetTxFIFO(SCIB_BASE);
    SCI_resetRxFIFO(SCIB_BASE);

    // 5) Clear any pending status.
    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF | SCI_INT_RXFF);

    // 6) Enable SCI interrupt sources.
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_RXFF | SCI_INT_TXFF);

    // 7) Register ISR handlers.
    Interrupt_register(INT_SCIB_RX, scibRxISR);
    Interrupt_register(INT_SCIB_TX, scibTxISR);

    // 8) Enable interrupts in PIE.
    Interrupt_enable(INT_SCIB_RX);
    Interrupt_enable(INT_SCIB_TX);

    // 9) Enable the SCI module.
    SCI_enableModule(SCIB_BASE);

    // 10) Enable CPU interrupts.
    EINT;
    ERTM;
}

// Builds the payload for the ACU message.
// If the function code is 0x84, it produces an "empty" message (payloadSize = 0).
void acuBuildPayload(MSPMsg_t *msg)
{
    UINT i;  // Declare loop index variable outside the loop header.
    if (comm_counter == 65535)
        comm_counter = 0;

    if (msg->function == 0x0084 && Current_state == IDNTIF_RFM_FLIGHT_MODULE)
    {
        msg->payloadSize = 0;
        for (i = 0; i < MSP_PAYLOAD_SIZE; i++)
        {
            msg->payload[i] = 0;
        }
    }
    else
    {
        msg->payload[0] = (acu_header1 >> 8) & 0xFF;
        msg->payload[1] = acu_header1 & 0xFF;
        msg->payload[2] = (acu_header2 >> 8) & 0xFF;
        msg->payload[3] = acu_header2 & 0xFF;
        msg->payload[4] = (mc1_logic_1_ID >> 8) & 0xFF;
        msg->payload[5] = mc1_logic_1_ID & 0xFF;
        msg->payload[6] = (mc1_logic_2_ID >> 8) & 0xFF;
        msg->payload[7] = mc1_logic_2_ID & 0xFF;
        msg->payload[8] = (mc2_logic_1_ID >> 8) & 0xFF;
        msg->payload[9] = mc2_logic_1_ID & 0xFF;
        msg->payload[10] = (mc2_logic_2_ID >> 8) & 0xFF;
        msg->payload[11] = mc2_logic_2_ID & 0xFF;
        msg->payload[12] = rfm_status_and_impact;
        msg->payload[13] = Current_state;
        msg->payload[14] = (mc_last_received_counter >> 8) & 0xFF;
        msg->payload[15] = mc_last_received_counter & 0xFF;
        msg->payload[16] = (comm_counter >> 8) & 0xFF;
        msg->payload[17] = comm_counter & 0xFF;
        msg->payload[18] = capacitor_voltage;

        msg->payloadSize = MSP_PAYLOAD_SIZE;
        for (i = 19; i < MSP_PAYLOAD_SIZE; i++)
        {
            msg->payload[i] = (uint8_t)(i + 1);  // Sample data.
        }
    }

    msg->crc = computeCRC_MSP(msg);
    comm_counter ++;
}

// Decodes the payload from a received ACU message.
// Replace the processing code below with your application-specific decoding.
void acuDecodePayload(const MSPMsg_t *msg)
{
    if (msg->payloadSize == 0)
    {
        // Empty payload: nothing to process.
    }
    else
    {
        acu_header1 =               (msg->payload[0] << 8) | msg->payload[1];
        acu_header2 =               (msg->payload[2] << 8) | msg->payload[3];
        rfm_logic_1_ID =            (msg->payload[4] << 8) | msg->payload[5];
        rfm_logic_2_ID =            (msg->payload[6] << 8) | msg->payload[7];
        acu_logic_1_ID =            (msg->payload[8] << 8) | msg->payload[9];
        acu_logic_2_ID =            (msg->payload[10] << 8) | msg->payload[11];
        rfm_status_and_impact =      msg->payload[12];
        rfm_logic_sate =             msg->payload[13];
        acu_last_received_counter = (msg->payload[14] << 8) | msg->payload[15];
        comm_counter =              (msg->payload[16] << 8) | msg->payload[17];
        mc_last_received_counter = comm_counter;
    }
}

// -----------------------------------------------------------------------------
// uartMsgTxACU()
// Builds the MSP message (41 bytes) for ACU and triggers transmission.
// -----------------------------------------------------------------------------
void uartMsgTxACU(void)
{
    if (!acuEnableTx)
    {
        return;
    }
    MSPMsg_t msg;

    // Build the MSP message.
    msg.start = TX_MSG_HEADER_BYTE0_ACU;
    msg.header = TX_MSG_HEADER_BYTE1_ACU;
    msg.type = MSP_TYPE_RECEIVE;   // For ACU, we use '>' to indicate a response.
    msg.flag = 0;

    // Set the function code. (This can be updated from main if desired.)
    msg.function = MSP_FUNCTION_ID;

    acuBuildPayload(&msg);

    {
        UINT j;  // Declare loop index variable outside loop header.
        for (j = 0; j < sizeof(MSPMsg_t); j++)
        {
            txBufferACU[j] = ((const uint8_t *)&msg)[j];
        }
        txLengthACU = sizeof(MSPMsg_t);
    }

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF);
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_TXFF);
}

// -----------------------------------------------------------------------------
// uartMsgRxACU()
// Receives and decodes a 41-byte MSP message for ACU.
// -----------------------------------------------------------------------------
void uartMsgRxACU(void)
{
    if (newDataReceivedACU)
    {
        MSPMsg_t receivedMsg;
        UINT i;  // Declare loop index variable outside loop header.
        for (i = 0; i < sizeof(MSPMsg_t); i++)
        {
            ((uint8_t *)&receivedMsg)[i] = rxBufferACU[i];
        }

        // Validate header and type.
        if (receivedMsg.start == TX_MSG_HEADER_BYTE0_ACU &&
            receivedMsg.header == TX_MSG_HEADER_BYTE1_ACU &&
            receivedMsg.type == MSP_TYPE_RECEIVE)
        {
            // Validate CRC.
            UINT8 computed_crc = computeCRC_MSP(&receivedMsg);
            if (computed_crc == receivedMsg.crc)
            {
                acuDecodePayload(&receivedMsg);
            }
            else
            {
                // Handle CRC error.
            }
        }
        else
        {
            // Handle header or type error.
        }
        acuPacketCount++;
        newDataReceivedACU = false;
    }
}

// -----------------------------------------------------------------------------
// checkComOkACU()
// Returns true if the number of transmitted ACU packets is at least filter_protocol,
// then resets the counter.
// -----------------------------------------------------------------------------
bool checkComOkACU(uint16_t filter_protocol)
{
    if (acuPacketCount >= filter_protocol)
    {
        acuPacketCount = 0;
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
// ACU ISR Implementations
// -----------------------------------------------------------------------------
__interrupt void scibRxISR(void)
{
    UINT16 intStatus = SCI_getInterruptStatus(SCIB_BASE);

    if (intStatus & SCI_INT_RXFF)
    {
        static volatile UINT16 rxIndexACU_local = 0;
        while (SCI_getRxFIFOStatus(SCIB_BASE) != SCI_FIFO_RX0)
        {
            UINT16 received = SCI_readCharNonBlocking(SCIB_BASE);
            if (rxIndexACU_local < RX_MSG_LENGTH)
            {
                rxBufferACU[rxIndexACU_local++] = (UINT8)received;
            }
            else
            {
                rxIndexACU_local = 0;
            }
            if (rxIndexACU_local == RX_MSG_LENGTH)
            {
                uartMsgRxACU();
                newDataReceivedACU = true;
                rxIndexACU_local = 0;
            }
        }
    }

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

__interrupt void scibTxISR(void)
{
    UINT16 intStatus = SCI_getInterruptStatus(SCIB_BASE);

    if (intStatus & SCI_INT_TXFF)
    {
        UINT16 fifoFree = 16U - SCI_getTxFIFOStatus(SCIB_BASE);
        while ((fifoFree > 0) && (txIndexACU < txLengthACU))
        {
            SCI_writeCharNonBlocking(SCIB_BASE, txBufferACU[txIndexACU++]);
            fifoFree--;
        }
        if (txIndexACU >= txLengthACU)
        {
            SCI_disableInterrupt(SCIB_BASE, SCI_INT_TXFF);
            txIndexACU = 0;
            txLengthACU = 0;
        }
    }

    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
