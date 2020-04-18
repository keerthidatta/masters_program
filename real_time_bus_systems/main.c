
#include <msp430f5529.h>

#include "config/node.h"
#include "lib/sysclk.h"
#include "kernel/kernel.h"
#include "lib/printfx.h"
#include "assert.h"

#include "can.h"
#include "can_tests.h"

#define CAN_BUFFER_SIZE 20
#define DEBOUNCING_TIME 500000
/*****************************************************************************************
*
*                                         Globals
*
*******************************************************************************************/
Time_t time;

/*****************************************************************************************
*
*                                         Events
*
*******************************************************************************************/
OS_DECLARE_EVENT(sendEv);
OS_DECLARE_EVENT(recvEv);

/*****************************************************************************************
*
*                                        Tests
*
*******************************************************************************************/
#if (TEST_EN == 1)

OS_DECLARE_TASK(testTask, 512, 128);
OS_TASKENTRY(testTask) {
    CAN_runTests(TEST_RX_EN, TEST_TX_EN);
    while(1);
}

#else

/*****************************************************************************************
*
*                                        Tx / Rx Tasks
*
*******************************************************************************************/

OS_DECLARE_TASK(sendTask,512,128);
OS_TASKENTRY(sendTask) {
    uint8_t msg_id = -1;
    uint8_t msg[CAN_TX_CIRC_NMSGS][11];
    uint8_t payload[] = { 0xEF, 0x1F };

    // Create CAN_TX_CIRC_NMSGS messages
    if (CAN_createFrame(OP_ADD, 0, payload, 2, msg[0], sizeof(msg[0])) < 0)
        printfx("Error creating CAN frame 0! \n");
    if (CAN_createFrame(OP_SUB, 0, payload, 2, msg[1], sizeof(msg[3])) < 0)
        printfx("Error creating CAN frame 3! \n");
    if (CAN_createFrame(OP_MUL, 1, payload, 2, msg[2], sizeof(msg[1])) < 0)
        printfx("Error creating CAN frame 1! \n");
    if (CAN_createFrame(0x800, 1, payload, 2, msg[3], sizeof(msg[2])) < 0)
        printfx("Error creating CAN frame 2! \n");

    while (1) {
        waitEvent(&sendEv);
        clearEvent(&sendEv);

#if (CAN_TX_CIRC_MODE_EN == 1)
        msg_id = (++msg_id)%CAN_TX_CIRC_NMSGS;
#else
        msg_id = CAN_MESSAGE;
#endif

        // Send message
        if (CAN_trySendFrame(msg[msg_id], CAN_DOACK, 0) < 0)
            printfx("Error sending message %d! \n", msg_id);
        else
            printfx("Message %d successfully delivered \n", msg_id);

        // Button debouncing
        busyDelay(DEBOUNCING_TIME);

        P2IFG = 0x00;
        P1IFG = 0x00;
        Pin_EnableInt(CAN_PORT, CAN_RX_PIN);
        Pin_EnableInt(TRIGGER_PORT, TRIGGER_IN_PIN);
    }
}

OS_DECLARE_TASK(recvTask,512,128);
OS_TASKENTRY(recvTask) {
    uint8_t can_buffer[CAN_BUFFER_SIZE];

    while (1) {
        memset(can_buffer, 0x00, CAN_BUFFER_SIZE);
        waitEvent(&recvEv);
        clearEvent(&recvEv);

        if (CAN_receiveFrame(can_buffer, CAN_BUFFER_SIZE, time) < 0) {
            printfx("Error receiving message! \n");
        } else
            CAN_decodeFrame(can_buffer);

        busyDelay(DEBOUNCING_TIME);
        P2IFG = 0x00;
        P1IFG = 0x00;
        Pin_EnableInt(CAN_PORT, CAN_RX_PIN);
        Pin_EnableInt(TRIGGER_PORT, TRIGGER_IN_PIN);
    }
}
#endif

/*****************************************************************************************
*
*                                   Interrupt Handlers
*
*******************************************************************************************/
// Interrupt handler for button P1.1
OS_DECLARE_IRQ_HANDLER(48, port1_isr);
void port1_isr(void) {
    Pin_DisableInt(TRIGGER_PORT, TRIGGER_IN_PIN);
    Pin_DisableInt(CAN_PORT, CAN_RX_PIN);

    // Set trigger
    Pin_High(TRIGGER_PORT,TRIGGER_OUT_PIN);
    Pin_Low(TRIGGER_PORT,TRIGGER_OUT_PIN);

    setEvent(&sendEv);
}

// Interrupt handler for CAN RX
OS_DECLARE_IRQ_HANDLER(43, port2_isr);
void port2_isr(void) {
    Pin_DisableInt(TRIGGER_PORT, TRIGGER_IN_PIN);
    Pin_DisableInt(CAN_PORT, CAN_RX_PIN);

    getCurrentTime(&time);

    setEvent(&recvEv);
}

/*****************************************************************************************
*
*                                   Main
*
*******************************************************************************************/
int main(void) {
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

#if (TEST_EN == 0)
    // Enable Interrupt for button P1.1 (Trigger)
    Pin_EnableInt(TRIGGER_PORT,TRIGGER_IN_PIN);
    Pin_FallingEdgeInt(TRIGGER_PORT,TRIGGER_IN_PIN);
    Pin_EnableResistor(TRIGGER_PORT,TRIGGER_IN_PIN);
    Pin_High(TRIGGER_PORT,1);

    // Trigger output
    Pin_OutputMode(TRIGGER_PORT, TRIGGER_OUT_PIN);
    Pin_SelectIO(TRIGGER_PORT, TRIGGER_OUT_PIN);
    Pin_Low(TRIGGER_PORT, TRIGGER_OUT_PIN);

    // Debug test output
    Pin_OutputMode(3, 6);
    Pin_SelectIO(3, 6);
#endif

    // run at 24MHz
    sysclk_setClocks(UCS_DCOCLK, UCS_DIVM, UCS_DIVSM, UCS_DIVA, UCS_DIVAN);
    UART_setConfig(UART_BAUDRATE, UART_DATABITS, UART_STOPBITS, UART_PARITY);

    CAN_init();

    os_run();
}
