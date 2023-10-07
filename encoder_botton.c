#include "mainheader.h"

uint8_t grados = 180; //Hacer estas dos variables globales para todos los archivos
uint8_t estado = 0;

	void EINT1_IRQHandler(){ //Esta función estará en el timer 0 IRQHandler
		static uint8_t direccion = 0; //direccion a la que gira el servo
		
		LPC_SC->EXTINT|=2;//limpiar la flag
		if (direccion){
			set_servo(grados+=10);	 // incrementamos la posici?n del servo de 10 en 10?
		}
		else{
			set_servo(grados-=10);	 // incrementamos la posici?n del servo de 10 en 10? a la inversa
		}
		if (grados == MIN || grados == MAX) direccion ^= 1; //si llega al minimo o maximo cambia la direccion
		
		//IR_measure();
		//display_measure();
		
	}