#include <LPC17xx.H>
#include "mainheader.h"

#define MIN 0
#define MAX 180


void set_servo(float grados)
	{
    LPC_PWM1->MR2=(Fpclk*1e-3 + Fpclk*1e-3*grados/180); //Var�a en funci�n de los grados el Duty cicle desde 100/15 % a 200/15 % 
		LPC_PWM1->LER|=(1<<2)|(1<<0); //Enable el Match 0 y el Match 2 (MR0 and MR2)
	}
	void EINT1_IRQHandler(){ //Esta función estará en el timer 0 IRQHandler
		static float grados1 = 180; //empieza en 180 grados
		static uint8_t direccion = 0; //direccion a la que gira el servo
		
		LPC_SC->EXTINT|=2;//limpiar la flag
		if (direccion){
			set_servo(grados1+=10);	 // incrementamos la posici�n del servo de 10 en 10�
		}
		else{
			set_servo(grados1-=10);	 // incrementamos la posici�n del servo de 10 en 10� a la inversa
		}
		if (grados1 == MIN || grados1 == MAX) direccion ^= 1; //si llega al minimo o maximo cambia la direccion
		
		//IR_measure();
		//display_measure();
		
	}
int main(void)
{
	config_EINT1();
	config_pwm2();
	
	while(1);
}