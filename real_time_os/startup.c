/*
 * startup.c
 *
 *  Created on: 05-Apr-2017
 *      Author: Group_23
 */
#include<kernel/kernel.h>
#include<arch/sysclk.h>
#include<drv/uart.h>
#include<libs/printfx.h>
#include<msp430f5529.h>


#define UART_BAUDRATE 9600ul ///< the desired UART baudrate in bps
#define UART_DATABITS 0 ///< databits select: 0=8DB 1=7DB
#define UART_STOPBITS 0 ///< stopbits select: 0=1SB 1=2SB
#define UART_PARITY 0x00 ///< parity select: 0=none, 1=none, 2=odd, 3=even

static void __attribute__((noreturn)) os_run(void);

int main(void)
{
    clk_defaultInitClock();
    uart_setConfig(UART_BAUDRATE, UART_DATABITS, UART_STOPBITS, UART_PARITY);
    os_run();
}
static void __attribute__((noreturn)) os_run(void)
{
    os_initTasks();
    registerResources();
    os_initSystemTimer();
    os_dispatcher();
}

