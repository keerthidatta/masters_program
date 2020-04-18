/*
 * timer.c
 *
 *  Created on: 06-Apr-2017
 *      Author: Group_23
 */
#include <kernel/kernel.h>
#include <msp430f5529.h>
#include <msp430.h>

Time_t os_timeline;

OS_DECLARE_IRQ_HANDLER(52, os_timerISR);


Result_t os_initSystemTimer(void)
{
      TA0CCR1 |= 0xF000; //Set the value of compare register for soft overflow.
      TA0CCR0 |= 0xFFFF; //Set the value of CCR0
      TA0CCTL1 |= CCIE;//Setting CCR1 = Compare Mode and Enabling Interrupt(CCIE)
      TA0CTL |= TASSEL_2 + ID__8 + MC_1 + TAIE + TACLR ; //Configuring Timer_A to UP mode and setting SM_Clock
      TA0EX0 |= TAIDEX_2;
      TA0R = 0;
      os_timeline = 0;
    _EINT();

    return 1;
}


void accumulate_timeline(void)
{
  uint16_t overflow = (TA0CTL & TAIFG);
  if(TA0R > 0xFFF0)
  {
    while(TA0IV != 0xE);
    overflow = 1;
  }
  if(overflow)
  {
      os_timeline += 0xFFFF + 1;
  }
  TA0CCR0 = TA0R + 2;
  os_timeline += TA0CCR0 + 1;
  while(TA0IV != 0xE);
  TA0CCR0 = 0xFFFF;
}

//void __attribute__((interrupt)) __hwirq_52(void)

void os_timerISR(void)
{
    update_timer();

}
void update_timer(void){
    accumulate_timeline();
    os_schedule();

}
void getCurrentTime(Time_t *time)
{
    if(time == NULL)
        return;
    uint16_t interrupt_state = _get_interrupt_state();

    _DINT();
    *time = os_timeline + TA0R;
    if(interrupt_state)
    {
      _EINT();
    }
}

