#include <LPC17xx.h>
#include "mainheader.h"

void config_pwm2(void)
  {
		LPC_PINCON->PINSEL3|=(2<<8); // P1.20 salida PWM (PWM1.2)
		LPC_SC->PCONP|=(1<<6); //Se activa el pwm
		LPC_PWM1->MR0=Fpclk*Tpwm-1; //Se pone el n?mero del contador 
		LPC_PWM1->PCR|=(1<<10); //configurado el ENA2 (1.2)
		LPC_PWM1->MCR|=(1<<1); //reset if TC = MR0
		LPC_PWM1->TCR|=(1<<0)|(1<<3); //Counter Enable, PWM Mode enable
	}
void config_EINT1(void)
	{
		LPC_PINCON->PINSEL4|=(0x01 << 22); //asocia la interrupcion al pin del bot?n P2.11
		LPC_SC -> EXTMODE = (1 << 1); // falling edge
		LPC_SC -> EXTPOLAR = (1 << 1);
		NVIC->ISER[0] = (1 << 19);
		
	}