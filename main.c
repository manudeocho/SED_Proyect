#include <LPC17xx.H>
#include "lcddriver.h"
#include <stdio.h>
#include <string.h>

//ENCODER:
//SW -> P2.10
//DT -> P1.23
//CLK -> P1.20
//SERVO:
//CTRL -> P1.18

#define F_cpu 100e6					// Defecto Keil (xtal=12Mhz)
#define Fpclk 25e6	// Fcpu/4 (defecto despu?s del reset)
#define Tpwm 20e-3	// Perido de la se?al PWM (15ms)

#define light_blue 0x07FF
#define orange 0xFB00
#define gris 0x39C5
#define light_green 0x0730

uint8_t MIN = 0;
uint8_t MAX = 180;

uint8_t mode = 0; // 0->Manual 1->Automatic
uint8_t estado = 0;
uint32_t grados = 0;

char buffer[25];
				
  

void config_EINT0(void){
		LPC_PINCON->PINSEL4|=(0x01 << 20); //asocia la interrupcion al pin del bot?n P2.10
		LPC_SC -> EXTMODE |= (1 << 0); // rising edge
		//LPC_SC -> EXTPOLAR |= (1 << 0); //flanco de subida
		NVIC->ISER[0] |= (1 << 18); //Enable interrupcion
		NVIC_SetPriority( EINT0_IRQn, 0);
	}

void config_EINT2(void){
		LPC_PINCON->PINSEL4|=(0x01 << 24); //asocia la interrupcion al pin del bot?n P2.12 (KEY2)
		LPC_SC -> EXTMODE |= (1 << 2); // rising edge
		NVIC->ISER[0] |= (1 << 20); //Enable interrupcion
		NVIC_SetPriority( EINT2_IRQn, 1);
	}
	
void config_pwm1(void){
		LPC_PINCON->PINSEL3|=(0x2<<4); // P1.18 salida PWM (PWM1.1)
		LPC_PINCON->PINMODE3|= (0<<4); 	// Habilita pullup
		LPC_SC->PCONP|=(1<<6); //Se activa el pwm
		LPC_PWM1->MR0=Fpclk*Tpwm-1; //Se pone el n?mero del contador 
		LPC_PWM1->PCR|=(1<<9); //configurado el ENA2 (1.1)
		LPC_PWM1->MCR|=(1<<1); //reset if TC = MR0
		LPC_PWM1->TCR|=(1<<0)|(1<<3); //Counter Enable, PWM Mode enable
	}

void config_encoder(void){
		LPC_SC->PCONP|=(1<<18); //Power on encoder
		LPC_PINCON->PINSEL3|=(1<<8)|(1<<14); //MCI0 and MCI1 modes enabled
		LPC_QEI->QEIMAXPOS = 4e9; //Pos max 4 mil millones
		//digital filter
	}

void config_TIMER1(){
		LPC_SC->PCONP |= 1 << 2; 					// Power up Timer1
		LPC_TIM1->MR0 = (Fpclk/2)-1;			// 25e6/2 para interrumpir cada medio segundo
		LPC_TIM1->MCR |= 1 << 0;					// Interrupt on Match 0 
		LPC_TIM1->MCR |= 1 << 1; 					// Reset on Match 0
		NVIC_EnableIRQ(TIMER1_IRQn); 
		LPC_TIM1->TCR |= 1 << 0; 					// Start timer
		NVIC_SetPriority( TIMER1_IRQn, 2);
	}

void config_TIMER3(){
		LPC_SC->PCONP |= 1 << 23; 				// Power up Timer3
		LPC_TIM3->MR0 = (Fpclk*0.4)-1;	// 
		LPC_TIM3->MCR |= 1 << 0;					// Interrupt on Match 0 
		LPC_TIM3->MCR |= 1 << 1; 					// Reset on Match 0
		//NVIC_EnableIRQ(TIMER3_IRQn); 
		LPC_TIM3->TCR |= 1 << 0; 					// Start timer
		NVIC_SetPriority( TIMER3_IRQn, 2);
	}

uint16_t get_IR_distance(){
		uint16_t Distancia = 0;
		return Distancia;
	}

uint16_t get_temperature(){
		uint16_t Temp = 0;
		return Temp;
	}


void display_numero(uint8_t Linea, char Texto, uint16_t color){
	
	sprintf(buffer,"                                                            ");
	drawString(10,Linea*16, buffer, BLACK, BLACK, MEDIUM); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
	sprintf(buffer,"%d",Texto);
	drawString(10,Linea*16, buffer, BLACK, color, MEDIUM); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
}

void display_texto(uint8_t Linea, char *Texto, uint16_t color){
	sprintf(buffer,"                                                            ");
	drawString(10,Linea*16, buffer, BLACK, BLACK, MEDIUM); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
	sprintf(buffer,"%s",Texto);
	drawString(10,Linea*16, buffer, BLACK, color, MEDIUM); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100

}

void display_borrar(uint8_t min, uint8_t max){
	
	uint8_t i;
	
	for(i=min;i<max+1;i++){
	sprintf(buffer,"                                                            ");
	drawString(10,16*i, buffer, BLACK, BLACK, MEDIUM); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
	}
}
void set_servo(float Grados){
	if(Grados < 180+1){
    LPC_PWM1->MR1=(Fpclk*0.5e-3 + Fpclk*(2.4-0.5)*1e-3*Grados/180); //Var?a en funci?n de los grados el Duty cicle desde 0.5/15 a 2.5/15
		LPC_PWM1->LER|=(1<<1)|(1<<0); //Enable el Match 0 y el Match 1 (MR0 and MR1)
		display_texto(16,"Degrees:",orange);
		display_numero(17, Grados, orange);
	}
	else{
		display_texto(16,"Error: not valid degree",RED);
		display_texto(17, "Push again please", orange);
	}
}	


void TIMER1_IRQHandler(){
		
		uint16_t distancia = 0;
		float temperature = 0;
		static uint8_t direccion = 0; // 0->Gira derecha  1->Gira izquierda
	
		LPC_TIM1->IR |= 1 << 0; // Borrar flag de interrupción
		
		if (estado == 3){

		}
		if(estado == 10)direccion = 0;
			
		if(estado == 12){ //Si modo autom?tico 
			
			if(direccion == 0){ //Si va a la derecha
				grados = grados + 10; //aumenta en 10 los grados
				set_servo(grados); //mueve el servo a esa posicion
			}
			else{
				grados = grados - 10;	//disminuye en 10 los grados
				set_servo(grados);	//mueve el servo a esa posicion
			}
			if(grados < MIN+1 | grados > MAX-1){ //si los grados est?n en las posiciones l?mite 
				direccion ^= 1; //cambia la direcci?n
			}
		}
		
		if(estado == 3 || estado == 12){
			distancia = get_IR_distance(); //saca la distancia
			temperature = get_temperature(); 
			display_texto(4, "Distance:", light_blue);
			display_numero(5, distancia, light_blue);
			display_texto(6, "Temperature:", light_blue);
			display_numero(7, temperature, light_blue);
		}
		else if(estado == 1);
		else{
			display_borrar(4,7);	
		}
	}
	
	void TIMER3_IRQHandler(){
		
		LPC_TIM3->IR |= 1 << 0; // Borrar flag de interrupción
		NVIC_DisableIRQ(TIMER3_IRQn); //Desable timer 3
		NVIC->ISER[0] |= (1 << 18); //Enable interrupcion del EINT0
		
	}

	void EINT0_IRQHandler(){ 
		
		
		LPC_SC->EXTINT|=1;//limpiar la flag
		
		NVIC_DisableIRQ(EINT0_IRQn); //Deshabilita EINT0
		NVIC->ISER[0] |= (1<<4); //Habilitar timer 3
		LPC_TIM3->TCR |= (1 << 1); // Reset timer
		LPC_TIM3->TCR &= ~(1 << 1); // Start timer
		
		switch(estado){
			
			case 1:
				display_borrar(0,10);
				estado = 2; //Estado en el que el encoder mide
				display_texto(10,"Welcome to manual mode!",light_green);
				display_texto(1,"Set the angle",light_blue);
				break; 
					
			case 2:
				//activar timer
				display_borrar(10,12);
				grados = 5*LPC_QEI->QEIPOS;
				set_servo(grados); //if click -> QUEIPOS= QUEIPOS +2
				LPC_SC->PCONP = LPC_SC->PCONP & 0xFFFBFFFF; //desactivar encoder
				estado = 3;
				display_texto(1,"Measures:",light_blue);
				display_texto(2,"Push to stop",light_blue);
				break;
			
			case 3:
				//desactivar timer
				LPC_SC->PCONP|=(1<<18); //activar encoder
				estado = 2;
				display_borrar(2,2);
				display_texto(1,"Set the angle",light_blue);
				break;
			
			case 10:
				MAX = 5*LPC_QEI->QEIPOS;
				display_texto(12,"Maximum angle:",YELLOW);
				display_numero(13,MAX,YELLOW);
				set_servo(MAX);
				estado = 11;
				display_borrar(0,2);
				display_borrar(10,11);
				display_texto(1,"Set minimum angle",light_blue);
				break;
			
			case 11: // 11->12 or 15
				MIN = 5*LPC_QEI->QEIPOS;
				display_texto(14,"Minimum angle:",YELLOW);
				display_numero(15,MIN,YELLOW);
				set_servo(MIN);
				grados = MIN;
				if(MIN>MAX || MIN==MAX){ //11->15
					estado = 10; //15->10
					display_texto(10,"Error: Min not less than max",RED);
					display_texto(1,"Set angle max again please",light_blue);
				}
				else{
					estado = 12; //11->12
					display_texto(1,"Measures:",light_blue);
					display_texto(1,"Push to stop everything",light_blue);
					display_texto(10,"KEY2 to set angles again",light_green);
				}
				break;
				
			case 12:
				//activar timer
				estado = 13;
				display_borrar(2,2);
				display_borrar(10,11);
				display_texto(1,"Push to continue",light_blue);
				break;
			
			case 13:
				//desactivar timer
				estado = 12;
				display_texto(1,"Measures:",light_blue);
				display_texto(1,"Push to stop everything",light_blue);
				display_texto(10,"KEY2 to set angles again",light_green);
				break;
			}
			display_texto(18,"Estado: ",gris);
			display_numero(19,estado,gris);
		
	}
	
	void EINT2_IRQHandler(){
		
		LPC_SC->EXTINT|=(1<<2);
		if(estado == 12){
			estado = 10;
			display_borrar(10,11);
			display_texto(1,"Decide maximum angle again",light_blue);
		}
	}
	
int main(void)
{
	//Config mode
	mode = 1^(((1<<11)&(LPC_GPIO2->FIOPIN))>>11); //if P2.11 is pushed -> mode = 1;
	
	lcdInitDisplay();
  fillScreen(BLACK);
	NVIC_SetPriorityGrouping(2);
	config_EINT0();
	config_EINT2();
	config_pwm1();
	config_encoder();
	config_TIMER1();
	config_TIMER3();
	if(mode == 0){
	estado = 1;
	display_texto(1,"Welcome to the proyect of",light_green);
	display_texto(2,"DIGITAL ELECTRONIC SYSTEMS",light_green);
	display_texto(4,"Push for manual mode",light_blue);
	display_texto(5,"Reset + KEY1 for automatic",light_blue);
	display_texto(6,"mode",light_blue);
	display_texto(8,"By Josilda Soarez and",orange);
	display_texto(9,"Manuel Sanchez",orange);
	display_texto(18,"Estado: ",gris);
	display_numero(19,estado,gris);
	}
	else{
	estado = 10;
	display_texto(1,"Set maximum angle",light_blue);
	display_texto(10,"Welcome to automatic mode!",light_green);
	display_texto(18,"Estado: ",gris);
	display_numero(19,estado,gris);
	}

	

	
	while(1);
}













/*
void random_comands(void){

//	drawChar(10, 100, 'A', RED, BLACK, SMALL);
//	drawChar(30, 100, 'A', RED, BLACK, MEDIUM);
//	drawChar(60, 100, 'A', RED, BLACK, LARGE);

	drawString(10, 00, "hello world!", CYAN, BLACK, SMALL);
//	drawString(10, 240, "hello world!", YELLOW, BLACK, MEDIUM);
//	drawString(10, 280, "hello world!", BLACK, WHITE, LARGE);
//	drawCircle(100, 50, 25, BLUE); //x0, y0, radio, color
//	fillCircle(100, 85, 25, MAGENTA);
//	fillRect(100, 63, 70, 20, YELLOW); //x0, y0, ancho, alto, color
	
	for(retardo=0;retardo<1000000;retardo++);

	for(contador=0;contador<10;contador++)
		{
			sprintf(buffer,"Contando... %d",contador);
			drawString(10,100, buffer, BLUE, YELLOW, MEDIUM); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100 
			for(retardo=0;retardo<10000000;retardo++);
		}
	}*/
