#ifndef __CAN_H__
#define __CAN_H__

#include <kernel/kernel.h>
#include <config/node.h>

// Test settings
#define TEST_EN     0               // enable/disable Tests
#define TEST_RX_EN  0               // enable/disable Receiver Tests
#define TEST_TX_EN  0               // enable/disable Transmitter Tests

// Tx application settings
#define CAN_MESSAGE         2       // select can package from 0 to 3 (only used in non circulation mode)
#define CAN_DOACK           0       // acknowledge enable
#define CAN_TX_CIRC_MODE_EN 0       // circulation mode enable
#define CAN_TX_CIRC_NMSGS   4       // number of different messages

#define SERVER_NODE         0       // Server node. It receives CAN messages, performs some operation according to the received ID, and replies with the result.
//Possible server operations
// TODO: Define operation ID's according to the exercise description
#define OP_MUL  0x0a
#define OP_ADD  0x0
#define OP_SUB  0x0

// Return codes
#define OK               1
#define ERR_BUFFER_SIZE -1
#define ERR_BUSBUSY     -2
#define ERR_COLLISION   -3
#define ERR_NOACK       -4
#define ERR_BITSTUFF    -5
#define ERR_CRC         -6
#define ERR_MAX_RETRANS -7

// Error strings
#define ERR_TEXT_BUFFER_SIZE    "Error: Buffer size too small \n"
#define ERR_TEXT_BUSBUSY        "Error: Bus busy \n"
#define ERR_TEXT_COLLISION      "Error: Collision detected \n"
#define ERR_TEXT_NOACK          "Error: ACK not received \n"
#define ERR_TEXT_BITSTUFF       "Error: Bit-stuffing error \n"
#define ERR_TEXT_CRC            "Error: Message Corrupted \n"
#define ERR_TEXT_MAX_RETRANS    "Error: Maximum number of retransmissions reached. \n"

#define CRC_CAN 0xC599

#define CAN_RETRANSMISSION_TIME 1000000

// Port Configuration
#define CAN_PORT 2
#define CAN_TX_PIN 5
#define CAN_RX_PIN 2

#define TRIGGER_PORT 1
//#define TRIGGER_IN_PIN 4      // Port 1.4 as trigger
#define TRIGGER_IN_PIN 4        // Button 1 as trigger
#define TRIGGER_OUT_PIN 3

typedef struct{
    uint32_t baudrate;
    uint32_t bit_time;
} CANConfig_t;

void        setBit(uint8_t* array, uint16_t bitPos);
void        clearBit(uint8_t* array, uint16_t bitPos);
uint8_t     testBit(uint8_t* array, uint16_t bitPos);
uint16_t    writeBuffer(uint8_t* dst, uint16_t dst_msbBit, uint32_t value, uint8_t bitLen);
uint32_t    readBuffer(uint8_t* src, uint16_t src_msbBit, uint8_t bitLen);

void        CAN_init(void);
void        CAN_setBaudrate(uint32_t baudrate);
int8_t      CAN_sendBit(uint8_t bit, Time_t* time);
uint8_t     CAN_recvBit(Time_t* time);

void        CAN_shiftReg(uint32_t* status, uint32_t polynom, uint32_t msbMask, uint8_t nextBit);
uint32_t CAN_createCRC(uint8_t* src, uint16_t bitLen, uint32_t polynom);
int16_t     CAN_createFrame(int32_t ID, uint8_t IDE, uint8_t* payload, uint8_t DLC, uint8_t* dst, uint8_t dstLen);
int16_t     CAN_sendFrame(uint8_t* msg, uint8_t doACK);
int16_t     CAN_trySendFrame(uint8_t* msg, uint8_t doACK, uint8_t retries);

int8_t      CAN_receiveFrame(uint8_t* dst, uint8_t dstLen, Time_t time);
void        CAN_decodeFrame(uint8_t* dst);

#endif
