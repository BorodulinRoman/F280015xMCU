#ifndef COMMON_DEFINES_H_
#define COMMON_DEFINES_H_

#define RX_MSG_LENGTH 41
#define TX_MSG_LENGTH 41
#define SCI_FIFO_LEVEL_EMPTY  SCI_FIFO_RX0
#define UART_BAUD_RATE     115200


// Define the states
#define PBIT                        1
#define IDNTIF_RFM_FLIGHT_MODULE    2
#define IDNTIF_RFM_ACU              3
#define READY_FOR_SAFETY_PIN        4
#define ARM_DELEY                   5
#define WAIT_FOR_ARM                6
#define ARMING_HAND_SHAKE           7
#define ARMING                      8
#define ARMED                       9
#define DET                         10
#define DISARM                      11
#define FAIL_SAFE                   12
#define SEC_DET_ATTEMPT             13
#define DUD                         14


// ACU rx decoders
extern volatile UINT16 acu_header1;
extern volatile UINT16 acu_header2;
extern volatile UINT16 rfm_logic_1_ID;
extern volatile UINT16 rfm_logic_2_ID;
extern volatile UINT16 acu_logic_1_ID;
extern volatile UINT16 acu_logic_2_ID;
extern volatile UINT8  rfm_status_and_impact;
extern volatile UINT8  rfm_logic_sate;
extern volatile UINT16 acu_last_received_counter;
extern volatile UINT16 comm_counter;

// ACU TX decoders (use different names if these should be separate variables)
extern volatile UINT16 acu_header1_tx;
extern volatile UINT16 acu_header2_tx;
extern volatile UINT16 mc1_logic_1_ID;
extern volatile UINT16 mc1_logic_2_ID;
extern volatile UINT16 mc2_logic_1_ID;
extern volatile UINT16 mc2_logic_2_ID;
extern volatile UINT8  mc_logic_sate;
extern volatile UINT16 mc_last_received_counter;
extern volatile UINT16 comm_counter_tx;
extern volatile UINT8  capacitor_voltage;


// MSP Message parameters
#define MSP_START_CHAR         '$'
#define MSP_HEADER_CHAR        'X'
#define MSP_TYPE_REQUEST       '<'
#define MSP_DEFAULT_FLAG       0
#define MSP_FUNCTION_ID        0x0100   // Example function code
#define MSP_PAYLOAD_SIZE       32
// FUNCTIONS and state machine definitions remain unchanged
#define IDENTIFY_FUNC                               0x0084
#define IDENTIFY_RFM_ACU                            0x0084

// Total packet size: 1 + 1 + 1 + 1 + 2 + 2 + 32 + 1 = 41 bytes.
struct MSPMsg_t {
    uint8_t start;         // Must be '$'
    uint8_t header;        // Must be 'X'
    char    type;          // e.g. '<', '>', or '!'
    uint8_t flag;          // Usually 0
    uint16_t function;     // Function code (little-endian)
    uint16_t payloadSize;  // Should be MSP_PAYLOAD_SIZE (32)
    uint8_t payload[MSP_PAYLOAD_SIZE]; // Payload data
    uint8_t crc;           // 8-bit CRC computed over bytes 2..39
};

typedef struct MSPMsg_t MSPMsg_t;
#endif /* COMMON_DEFINES_H_ */
