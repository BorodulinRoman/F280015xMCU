/******************************************************************************
 * File:        uart_operation_master.c
 * Description: UART master implementation using SCI interrupt + FIFO
 * Author:      Roman Borodulin
 * Created:     20/01/2025
 ******************************************************************************/
#include "GenericTypeDefs.h"
#include "uart_operation_master.h"

// -----------------------------------------------------------------------------
// Global variables (from header):
// -----------------------------------------------------------------------------
volatile UINT8 rxBufferMaster[RX_M_MSG_LENGTH];
volatile bool newDataReceivedMaster = false;

// -----------------------------------------------------------------------------
// Local (static) variables to manage reception
// -----------------------------------------------------------------------------
static volatile UINT16 rxIndexMaster = 0;

// -----------------------------------------------------------------------------
// Local (static) variables to manage transmission
// -----------------------------------------------------------------------------
static volatile UINT8  txBufferMaster[TX_M_MSG_LENGTH];
static volatile UINT16 txIndexMaster = 0;
static volatile UINT16 txLengthMaster = 0;

// -----------------------------------------------------------------------------
// Local (static) message counter
//   This increments each time we send a message, and is stored in bytes 2,3.
// -----------------------------------------------------------------------------
static volatile UINT16 msgCounter = 0;

// -----------------------------------------------------------------------------
// Forward declarations of interrupt service routines
// -----------------------------------------------------------------------------
__interrupt void sciaRxISR(void);
__interrupt void sciaTxISR(void);

// =============================================================================
// Function:     uartConfigMaster
// Purpose:      Configures SCI-A for 115200, 8N1, FIFO mode, and enables
//               interrupts in the PIE.
// =============================================================================
void uartConfigMaster(void)
{
    //
    // 1) Configure GPIOs for SCIA
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    //
    // 2) Perform any needed reset or initialization of SCIA.
    //
    SCI_performSoftwareReset(SCIA_BASE);

    //
    // 3) Set configuration (baud rate = 115200, 8N1, etc.)
    //
    SCI_setConfig(SCIA_BASE,
                  DEVICE_LSPCLK_FREQ,
                  UART_BAUD_RATE,
                  (SCI_CONFIG_WLEN_8 |
                   SCI_CONFIG_STOP_ONE |
                   SCI_CONFIG_PAR_NONE));

    //
    // 4) Enable FIFO and set interrupt trigger levels
    //
    SCI_enableFIFO(SCIA_BASE);
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);

    //
    // 5) Clear any pending status
    //
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);

    //
    // 6) Enable SCI interrupt sources in the peripheral
    //
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXFF | SCI_INT_TXFF);

    //
    // 7) Register ISR handlers with the interrupt controller
    //
    Interrupt_register(INT_SCIA_RX, sciaRxISR);
    Interrupt_register(INT_SCIA_TX, sciaTxISR);

    //
    // 8) Set interrupt priority and enable interrupts in PIE
    //
    Interrupt_enable(INT_SCIA_RX);
    Interrupt_enable(INT_SCIA_TX);

    //
    // 9) Enable the SCI module
    //
    SCI_enableModule(SCIA_BASE);

    //
    // 10) Enable CPU interrupts (if not already done)
    //
    EINT;   // Enable global interrupt
    ERTM;   // Enable real-time interrupt
}

// =============================================================================
// Function:     uartMsgTxMaster
// Purpose:      Initiates the transmission of a 20-byte message. Bytes 2 and 3
//               store a 16-bit message counter (2=high, 3=low).
// =============================================================================
void uartMsgTxMaster(void)
{
    UINT16 i;

    // 1) Header
    txBufferMaster[0] = TX_M_MSG_HEADER_BYTE0;  // e.g. 0xE5
    txBufferMaster[1] = TX_M_MSG_HEADER_BYTE1;  // e.g. 0xED

    // 2) Message counter in bytes 2,3
    txBufferMaster[2] = (UINT8)((msgCounter >> 8) & 0xFF);
    txBufferMaster[3] = (UINT8)(msgCounter & 0xFF);
    msgCounter++;

    // 3) Fill bytes [4..17] with your data
    for (i = 4; i < (TX_M_MSG_LENGTH - 2); i++)
    {
        txBufferMaster[i] = (UINT8)i;  // or your data
    }

    // 4) Compute CRC over first 18 bytes (indexes 0..17)
    UINT16 crc = crcCalculationMaster(txBufferMaster, (TX_M_MSG_LENGTH - 2));

    // 5) Store CRC in last two bytes (little-endian or big-endian—depends on your protocol)
    txBufferMaster[TX_M_MSG_LENGTH - 2] = (UINT8)(crc & 0xFF);
    txBufferMaster[TX_M_MSG_LENGTH - 1] = (UINT8)((crc >> 8) & 0xFF);

    // 6) Set TX length and start transmission
    txIndexMaster  = 0;
    txLengthMaster = TX_M_MSG_LENGTH;

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXFF);

}

// =============================================================================
// Function:     uartMsgRxMaster
// Purpose:      If newDataReceivedMaster is set, we have a 20-byte msg in
//               rxBufferMaster. This is a placeholder for any post-processing.
// =============================================================================
void uartMsgRxMaster(void)
{
    if(newDataReceivedMaster)
    {
        // Process rxBufferMaster here as needed

        // Reset the flag
        newDataReceivedMaster = false;
    }
}

// =============================================================================
// Function:     crcCalculationMaster
// Purpose:      Example trivial CRC (sum of bytes). Replace as needed.
// =============================================================================
UINT16 crcCalculationMaster(volatile UINT8 *data, UINT length)
{
    UINT16 sum = 0;
    UINT i;
    for(i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return sum  ;
}

// =============================================================================
// Interrupt Service Routine:  sciaRxISR
// Purpose:                    Handle RX FIFO interrupt events (SCIA).
// =============================================================================
__interrupt void sciaRxISR(void)
{
    UINT16 intStatus = SCI_getInterruptStatus(SCIA_BASE);

    if(intStatus & SCI_INT_RXFF)
    {
        // While RX FIFO not empty, read data out
        while(SCI_getRxFIFOStatus(SCIA_BASE) != SCI_FIFO_RX0)
        {
            UINT16 received = SCI_readCharNonBlocking(SCIA_BASE);

            // Store in rxBufferMaster if space allows
            if(rxIndexMaster < RX_M_MSG_LENGTH)
            {
                rxBufferMaster[rxIndexMaster++] = (UINT8)received;
            }
            else
            {
                // If exceeded buffer, handle error or reset
                rxIndexMaster = 0;
            }

            // Check if we've received a full (20-byte) message
            if(rxIndexMaster == RX_M_MSG_LENGTH)
            {
                newDataReceivedMaster = true;
                rxIndexMaster = 0;
            }
        }
    }

    // Clear interrupt flags
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

// =============================================================================
// Interrupt Service Routine:  sciaTxISR
// Purpose:                    Handle TX FIFO interrupt events (SCIA).
// =============================================================================
__interrupt void sciaTxISR(void)
{
    UINT16 intStatus = SCI_getInterruptStatus(SCIA_BASE);

    if(intStatus & SCI_INT_TXFF)
    {
        // FIFO space available
        UINT16 fifoFree = 16U - SCI_getTxFIFOStatus(SCIA_BASE);

        // Fill TX FIFO while space remains and data is left to send
        while((fifoFree > 0) && (txIndexMaster < txLengthMaster))
        {
            SCI_writeCharNonBlocking(SCIA_BASE, txBufferMaster[txIndexMaster++]);
            fifoFree--;
        }

        // If we've finished sending the entire message
        if(txIndexMaster >= txLengthMaster)
        {
            // Disable TX interrupt
            SCI_disableInterrupt(SCIA_BASE, SCI_INT_TXFF);

            // Reset counters
            txIndexMaster = 0;
            txLengthMaster = 0;
        }
    }

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
