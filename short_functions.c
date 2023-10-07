#include "mainheader.h"


void set_servo(float Grados)
	{
    LPC_PWM1->MR2=(Fpclk*1e-3 + Fpclk*1e-3*Grados/180); //Var?a en funci?n de los grados el Duty cicle desde 100/15 % a 200/15 % 
		LPC_PWM1->LER|=(1<<2)|(1<<0); //Enable el Match 0 y el Match 2 (MR0 and MR2)
	}
