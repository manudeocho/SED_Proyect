#include <LPC17xx.H>
#include "lcddriver.h"
#include <stdio.h>
#include <string.h>
#include "uart.h"
#include <Math.h>

//ENCODER:
//SW -> P2.10
//DT -> P1.23
//CLK -> P1.20
//SERVO:
//CTRL -> P1.18

#define F_cpu 100e6					// Defecto Keil (xtal=12Mhz)
#define Fpclk 25e6	// Fcpu/4 (defecto despu?s del reset)
#define Tpwm 20e-3	// Perido de la se?al PWM (15ms)
#define F_pclk F_cpu/4 	// Defecto despues del reset

#define light_blue 0x07FF
#define orange 0xFB00
#define gris 0x39C5
#define light_green 0x0730
#define purple 0xF81F

uint8_t MIN = 0;
uint8_t MAX = 180;

uint8_t turn_res = 10;
uint8_t mode = 0; // 0->Manual 1->Automatic
uint8_t estado = 0;
uint8_t display_token[20] = {0};
float grados = 0;

#define pi 3.14159
#define F_out 1000  // 100 Hz	
#define N_muestras 32	// 32 muestras/ciclo
#define V_refp 3.3
uint16_t muestras[N_muestras];
uint16_t f_out = 100;

float distancia = 0;
float temperature = 0;

char buffer[30];		// Buffer de recepción de 30 caracteres
char *ptr_rx;			// puntero de recepción
char rx_completa;		// Flag de recepción de cadena que se activa a "1" al recibir la tecla return CR(ASCII=13)
char *ptr_tx;			// puntero de transmisión
char tx_completa;		// Flag de transmisión de cadena que se activa al transmitir el caracter null (fin de cadena)
char fin=0;
				
 
void config_ADC(void){
	LPC_SC->PCONP|= (1<<12);					// POwer ON
	LPC_PINCON->PINSEL1|= (1<<14);  	// ADC input= P0.23 (AD0.0)
	LPC_PINCON->PINMODE1|= (2<<14); 	// Deshabilita pullup/pulldown
	LPC_SC->PCLKSEL0|= (0x00<<8); 		// CCLK/4 (Fpclk despu?s del reset) (100 Mhz/4 = 25Mhz)
	LPC_ADC->ADCR= (0x01<<0)|		  	  // Canal 0
								 (0x01<<8)|		  	  // CLKDIV=1   (Fclk_ADC=25Mhz /(1+1)= 12.5Mhz)
								 (0x01<<21)|			 	// PDN=1
								 (7<<24);				    // Inicio de conversi?n con el Match 1 del Timer 1989
	LPC_ADC->ADINTEN= (1<<8);	// Hab. interrupci?n fin de conversi?n canal 0
	NVIC_EnableIRQ(ADC_IRQn);					// ? 
	NVIC_SetPriority(ADC_IRQn,1);			// ?      
}
void config_EINT0(void){
	
		LPC_PINCON->PINSEL4|=(0x01 << 20); //asocia la interrupcion al pin del bot?n P2.10
		LPC_SC -> EXTMODE |= (1 << 0); // rising edge
		//LPC_SC -> EXTPOLAR |= (1 << 0); //flanco de subida
		NVIC->ISER[0] |= (1 << 18); //Enable interrupcion
		NVIC_SetPriority( EINT0_IRQn, 0);

		//LPC_QEI->QEICON = 1; // Reset position counter
		//LPC_QEI->FILTER = (uint32_t)(40e3);
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
	}

void config_TIMER1(uint16_t IR_Period){
		LPC_SC->PCONP |= 1 << 2; 					// Power up Timer1
		LPC_TIM1->MR0 = (Fpclk*(IR_Period*0.001))-1;			// 25e6/2 para interrumpir cada IR_temp ms
		LPC_TIM1->MR1 = (Fpclk*(IR_Period*0.001))/2 -1;
		LPC_TIM1->MCR |= (1 << 0);				// Interrupt on Match 0 
		LPC_TIM1->MCR |= 1 << 1; 					// Reset on Match 0
		NVIC_EnableIRQ(TIMER1_IRQn); 
		LPC_TIM1->EMR = 0x00C0;   				// Toggle External Match 1
		LPC_TIM1->TCR |= 1 << 0; 					// Start timer
		NVIC_SetPriority( TIMER1_IRQn, 0); //2
	}

void config_TIMER3(){
		LPC_SC->PCONP |= 1 << 23; 				// Power up Timer3
		LPC_TIM3->MR0 = (Fpclk)-1;	// 
		LPC_TIM3->MCR |= 1 << 0;					// Interrupt on Match 0 
		LPC_TIM3->MCR |= 1 << 1; 					// Reset on Match 0
		NVIC_DisableIRQ(TIMER3_IRQn); 
		LPC_TIM3->TCR |= 1 << 0; 					// Start timer
		NVIC_SetPriority( TIMER3_IRQn, 2);
	}



void config_DAC(void)
{
	LPC_PINCON->PINSEL1|= (2<<20); 	 	// DAC output = P0.26 (AOUT)
	LPC_PINCON->PINMODE1|= (2<<20); 	// Deshabilita pullup/pulldown
	LPC_DAC->DACCTRL=0;								//  
}

void config_TIMER2(void)
{
	LPC_SC->PCONP|=(1<<22);						// 	Power ON
    LPC_TIM2->PR = 0x00;     	 				//  Prescaler =1
    LPC_TIM2->MCR = 0x03;							//  Reset TC on Match, e interrumpe!  
    LPC_TIM2->MR0 = (F_pclk/f_out/N_muestras)-1;  // Cuentas hasta el Match 
    LPC_TIM2->EMR = 0x00;   					//  No actúa sobre el HW
    LPC_TIM2->TCR = 0x01;							//  Start Timer
    NVIC_EnableIRQ(TIMER2_IRQn);			//  Habilita NVIC
	  NVIC_SetPriority(TIMER2_IRQn,1);  //  Nivel 1 prioridad 
		
}

void genera_muestras(uint16_t muestras_ciclo)
{
	uint16_t i;
	//señal senoidal
	for(i=0;i<muestras_ciclo;i++)
	muestras[i]=(uint32_t)(511+511*sin(2*pi*i/N_muestras)); // Ojo! el DAC es de 10bits
}

void TIMER2_IRQHandler(void)
{
static uint16_t indice_muestra;
	LPC_TIM2->IR|= (1<<0); 										// borrar flag
	LPC_DAC->DACR= muestras[indice_muestra++] << 6; // bit6..bit15 
	indice_muestra&= N_muestras-1;						// contador circular (Si N_muestras potencia de 2) 
	LPC_TIM2->MR0 = (F_pclk/f_out/N_muestras)-1;  // Cuentas hasta el Match 	
}	



void init_TIMER1(void)
{
	  LPC_SC->PCONP|=(1<<22);						// 	Power ON
    LPC_TIM2->PR = 0x00;     	 				//  Prescaler =1
    LPC_TIM2->MCR = 0x03;							//  Reset TC on Match, e interrumpe!  
    LPC_TIM2->MR0 = (F_pclk/f_out/N_muestras)-1;  // Cuentas hasta el Match 
    LPC_TIM2->EMR = 0x00;   					//  No actúa sobre el HW
    LPC_TIM2->TCR = 0x01;							//  Start Timer
    NVIC_EnableIRQ(TIMER2_IRQn);			//  Habilita NVIC
	  NVIC_SetPriority(TIMER2_IRQn,1);  //  Nivel 1 prioridad 
		
}



uint16_t get_temperature(){
		uint16_t Temp = 0;
		return Temp;
	}


void display_numero(uint8_t Linea, char Texto, uint16_t color){
	
	sprintf(buffer,"                                                            ");
	drawString(10,Linea*16, buffer, BLACK, BLACK, MEDIUM);
	sprintf(buffer,"%d",Texto);
	drawString(10,Linea*16, buffer, color, BLACK, MEDIUM);
}

void display_texto(uint8_t Linea, char *Texto, uint16_t color){
	sprintf(buffer,"                                                            ");
	drawString(10,Linea*16, buffer, BLACK, BLACK, MEDIUM);
	sprintf(buffer,"%s",Texto);
	drawString(10,Linea*16, buffer, color, BLACK, MEDIUM);

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
		display_token[5] = 1; //Distancia
		
	}
	else{
		display_token[6] = 1; //Angulo no válido
	}
}	


void TIMER1_IRQHandler(){
		
		float temperature = 0;
		static uint8_t direccion = 0; // 0->Gira derecha  1->Gira izquierda
	
		LPC_TIM1->IR |= 1 << 0; // Borrar flag de interrupci?n
		
		if (estado == 2){
			grados = ((float)turn_res / 2)*LPC_QEI->QEIPOS;
			set_servo(grados); //if click -> QUEIPOS= QUEIPOS +2
		}
		if(estado == 10)direccion = 0;
			
		if(estado == 12){ //Si modo autom?tico 
			
			if(direccion == 0){ //Si va a la derecha
				grados = grados + turn_res; //aumenta en 10 los grados
				set_servo(grados); //mueve el servo a esa posicion
			}
			else{
				grados = grados - turn_res;	//disminuye en 10 los grados
				set_servo(grados);	//mueve el servo a esa posicion
			}
			if(grados < MIN+1 | grados > MAX-1){ //si los grados est?n en las posiciones l?mite 
				direccion ^= 1; //cambia la direcci?n
			}
		}
		
		if(estado == 3 || estado == 12){
			temperature = get_temperature(); 
			display_token[4] = 1; //Distancia
			//uart0_fputs("Distance:\n");
			NVIC_EnableIRQ(TIMER2_IRQn);			//  Habilita NVIC
		}
		else if(estado == 1) NVIC_DisableIRQ(TIMER2_IRQn);			//  Habilita NVIC
		else{
			display_token[1] = 1;	
			NVIC_DisableIRQ(TIMER2_IRQn);			//  Habilita NVIC
		}
	}
	
	void TIMER3_IRQHandler(){
		
		LPC_TIM3->IR |= 1 << 0; // Borrar flag de interrupci?n
		NVIC_DisableIRQ(TIMER3_IRQn); //Desable timer 3
		NVIC->ISER[0] |= (1 << 18); //Enable interrupcion del EINT0
		
	}

	void EINT0_IRQHandler(){ 
		
		
		LPC_SC->EXTINT|=1;//limpiar la flag
		
		//NVIC_DisableIRQ(EINT0_IRQn); //Deshabilita EINT0
		//NVIC->ISER[0] |= (1<<4); //Habilitar timer 3
		//LPC_TIM3->TCR |= (1 << 1); // Reset timer
		//LPC_TIM3->TCR &= ~(1 << 1); // Start timer
		
		switch(estado){
			
			case 1:
				estado = 2; //Estado en el que el encoder mide
				display_token[2] = 1; //Set the angle + INFO
				break; 
					
			case 2:
				//activar timer
				display_borrar(10,12);
				LPC_SC->PCONP = LPC_SC->PCONP & 0xFFFBFFFF; //desactivar encoder
				estado = 3;
				display_token[3] = 1;//Measures
				break;
			
			case 3:
				//desactivar timer
				LPC_SC->PCONP|=(1<<18); //activar encoder
				estado = 2;
				display_token[7] = 1; //Set the angle
				break;
			
			case 10:
				MAX = ((float)turn_res / 2)*LPC_QEI->QEIPOS;
				set_servo(MAX);
				estado = 11;
				display_token[16] = 1; //Maximum
				display_token[11] = 1;
				break;
			
			case 11: // 11->12 or 15
				MIN = ((float)turn_res / 2)*LPC_QEI->QEIPOS;
				display_token[12] = 1; //Minimum
				set_servo(MIN);
				grados = MIN;
				if(MIN>MAX || MIN==MAX){ //11->15
					estado = 10; //15->10
					display_token[10] = 1; //Maximum again + ERROR MESAGE
				}
				else{
					estado = 12; //11->12
					display_token[15] = 1; //Measures
				}
				break;
				
			case 12:
				estado = 13;
				display_token[13] = 1; //Erase measures
				break;
			
			case 13:
				estado = 12;
				display_token[15] = 1; //Measures
				break;
			}
			display_token[18]=1; //State info!!!
		
	}
	
	void EINT2_IRQHandler(){
		
		LPC_SC->EXTINT|=(1<<2);
		if(estado == 12){
			estado = 10;
			display_token[9] = 1; //Maximum again
		}
	}
	
void ADC_IRQHandler(void)
{
	float voltios;
	float idistance;
	
	voltios= (float) ((LPC_ADC->ADGDR >>4)&0xFFF)*3.3/4095;	// se borra automat. el flag DONE al leer ADCGDR
	
	if (voltios <= 2.75 && voltios >= 2.5){
		idistance = 0.05 + ((voltios-2.5)/(2.75 -2.5))*(0.066-0.05);
	}
	else if (voltios < 2.5 && voltios >= 2){
		idistance = 0.03333 + ((voltios-2)/(2.5 -2))*(0.05-0.033);
	}
	else if (voltios < 2 && voltios >= 0.4){
		idistance = 0.025 + ((voltios-1.5)/(2 -1.5))*(0.033-0.025);
	}
	else{
		idistance = 1;
	}
	distancia = 1/idistance;
	if(distancia != 1)f_out = 220*pow(2, distancia / 120); 
}

uint16_t ask_user(){
	
	uint16_t IR_Period = 500;
	tx_cadena_UART0("1. xxg? (turning resolution in degrees as 'xx', followed by 'g' and finished by Enter, i.e.: 10g, 15g, 20g\n");
	do{
		if(rx_completa){					 	
			rx_completa=0; 				
			if ((strcmp (buffer, "\n10g\r") == 0) || (strcmp (buffer, "10g\r") == 0)){
					turn_res = 10; 
					fin=1;
			}
			else if ((strcmp (buffer, "\n15g\r") == 0) || (strcmp (buffer, "15g\r") == 0)){
				turn_res = 15; 
				fin=1;
}
			else if ((strcmp (buffer, "\n20g\r") == 0) || (strcmp (buffer, "20g\r") == 0)){
				turn_res = 20; 
				fin=1;}
			else tx_cadena_UART0("Wrong command.\n Please, text it again.\n\r");
			 }
		}while(fin==0);
	fin = 0;	
	sprintf(buffer, "%d degrees selected\n2. xxxms? (scanning period in miliseconds as 'xxx', followed by 'ms' and finished by Enter, i.e.: 200ms, 400ms, 600ms.\n",turn_res);
	tx_cadena_UART0(buffer); 

	do{
		if(rx_completa){					 	
			rx_completa=0; 				
			if (strcmp (buffer, "\n200ms\r") == 0){
				IR_Period = 200;
				fin=1;
			}
			else if (strcmp (buffer, "\n400ms\r") == 0){
				IR_Period = 400;
				fin=1;
}
			else if (strcmp (buffer, "\n600ms\r") == 0){
				IR_Period = 600;
				fin=1;}
			else tx_cadena_UART0("Wrong command.\n Please, text it again.\n\r");
			 }
		}while(fin==0);
	fin = 0;	

	sprintf(buffer, "%d ms selected\n3. h? (To insert new parameters and see this help again 'h' followed by Enter\n",IR_Period);
	tx_cadena_UART0(buffer); 

	return IR_Period;
}
int main(void)
{
	uint16_t IR_period = 500;
	uint8_t display_token_pos = 0;
	uint16_t i;
	
	//Config mode
	mode = 1^(((1<<11)&(LPC_GPIO2->FIOPIN))>>11); //if P2.11 is pushed -> mode = 1;
	
		LPC_GPIO1->FIODIR |= (1<<18);	 // P1.18 definido como salida  
		LPC_GPIO1->FIOCLR |= (1<<18);	 // P1.18 apagado 
		ptr_rx=buffer;	                // inicializa el puntero de recepción al comienzo del buffer
		uart0_init(9600);							 // configura la UART0 a 9600 baudios, 8 bits, 1 bit stop

	lcdInitDisplay();
  fillScreen(BLACK);
	tx_cadena_UART0("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	
	if(mode == 0){
	estado = 1;
	tx_cadena_UART0("Welcome!!!\nPush for manual mode\nReset + KEY1 for automatic\n\n\nBut first the program need some parameters:\n");
	display_texto(1,"Welcome to the proyect of",light_green);
	display_texto(2,"DIGITAL ELECTRONIC SYSTEMS",light_green);
	display_texto(4,"Push for manual mode",light_blue);
	display_texto(5,"Reset + KEY1 for automatic",light_blue);
	display_texto(6,"mode",light_blue);
	display_texto(8,"By Josilda Soares and",orange);
	display_texto(9,"Manuel Sanchez",orange);
	display_texto(18,"Estado: ",gris);
	display_numero(19,estado,gris);
	}
	else{
	estado = 10;
	tx_cadena_UART0("You are in Automatic Mode:\n ");				
	display_texto(1,"Set maximum angle",light_blue);
	display_texto(10,"Welcome to automatic mode!",light_green);
	display_texto(18,"Estado: ",gris);
	display_numero(19,estado,gris);
	}
	
	IR_period = ask_user();
	
	do{
		if(rx_completa){					 	// Comprabamos la llegada de una cadena por RXD
			rx_completa=0; 				// Borrar flag para otra recepción
			if (strcmp (buffer, "\nh\r") == 0){
				tx_cadena_UART0("Insert new parameters\n" ); 
				ask_user();
			}
			fin=1;
			 }
		}while(fin==0);
	fin = 0;	
		
	
	NVIC_SetPriorityGrouping(2);
	config_DAC();
	config_TIMER2();
	genera_muestras(N_muestras);
	config_DAC();
	config_ADC();
	config_EINT0();
	config_EINT2();
	config_pwm1();
	config_encoder();
	config_TIMER1(IR_period);
	//config_TIMER3();

	tx_cadena_UART0("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nConfig done... Iniciating program...\n\n\n"); 


	
while(1){
for(display_token_pos=0;display_token_pos<=20;display_token_pos++){
	if(display_token[display_token_pos] != 0) break;
}
switch(display_token_pos){
	case 1:
		display_borrar(4,9);
		display_token[display_token_pos] = 0;
		break;
	case 2:
		display_texto(10,"Welcome to manual mode!",light_green);
		display_texto(1,"Set the angle",light_blue);
		display_texto(2,"Then press the buttom",light_blue);
		tx_cadena_UART0("You are in Manual Mode:\n1. Turn the encoder (left-right) to position the measurement system.\n2. Push the encoder button to start/stop measurements.\n");
		display_token[display_token_pos] = 0;
		break;
	case 3:
		display_texto(1,"Measures:",light_blue);
		display_texto(2,"Push to stop",light_blue);
		display_token[display_token_pos] = 0;
		break;
	case 4:	
		display_texto(4, "Distance:", purple);
			//sprintf(buffer,"%s","Distance:");
			//drawString(85,4*16, buffer, orange, BLACK, MEDIUM); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
		display_numero(5, distancia, purple);
		if(distancia > 255){ display_texto(5, "Fuera de rango", purple);	}
			//sprintf(buffer,"      ");
			//drawString(55,5*16, buffer, BLACK, BLACK, 2); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
			//sprintf(buffer,"%f",distancia);
			//drawString(55,5*16, buffer, YELLOW, BLACK, 2); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
		display_texto(7, "Temperature:", purple);
			//sprintf(buffer,"%s","Temperature:");
			//drawString(75,7*16, buffer, orange, BLACK, MEDIUM); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
		display_numero(8, temperature, purple);
			//sprintf(buffer,"      ");
			//drawString(55,8*16, buffer, BLACK, BLACK, 2); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
			//sprintf(buffer,"%f",temperature);
			//drawString(55,8*16, buffer, YELLOW, BLACK, 2); //Coordenadas en pixels desde la esquina superior izda: x:10, y:100
				display_token[display_token_pos] = 0;
		//sprintf(buffer,"Distance: %f\nTemperature: %f",distancia,temperature);
				break;
	case 5:
		display_texto(16,"Degrees:",orange);
		display_numero(17, grados, orange);
			display_token[display_token_pos] = 0;
				break;
	case 6:
		display_texto(16,"Error: not valid degree",RED);
		display_texto(17, "Set another angle please", orange);
			display_token[display_token_pos] = 0;
				break;
	case 7:
		display_borrar(2,2);
		display_texto(1,"Set the angle",light_blue);
			display_token[display_token_pos] = 0;
				break;
		case 9:
		display_borrar(10,11);
		display_texto(1,"Decide maximum angle again",light_blue);
				display_token[display_token_pos] = 0;
				break;
	case 10:
		display_texto(10,"Error: Min not less than max",RED);
		display_texto(1,"Set angle max again please",light_blue);
			display_token[display_token_pos] = 0;
				break;
	case 11:
			display_borrar(0,2);
			display_borrar(10,11);
			display_texto(1,"Set minimum angle",light_blue);
			display_token[display_token_pos] = 0;
				break;
	case 12:
		display_texto(14,"Minimum angle:",YELLOW);
		display_numero(15,MIN,YELLOW);
			display_token[display_token_pos] = 0;
				break;
	case 13:
		display_borrar(2,2);
		display_borrar(10,11);
		display_texto(1,"Push to continue",light_blue);
			display_token[display_token_pos] = 0;
				break;
	case 15:
		display_texto(1,"Measures:",light_blue);
		display_texto(1,"Push to stop everything",light_blue);
		display_texto(10,"KEY2 to set angles again",light_green);
			display_token[display_token_pos] = 0;
				break;
	case 16:
		display_texto(12,"Maximum angle:",YELLOW);
		display_numero(13,MAX,YELLOW);
			display_token[display_token_pos] = 0;
				break;
	case 18:
		display_texto(18,"Estado: ",gris);
		display_numero(19,estado,gris);
			display_token[display_token_pos] = 0;
				break;

}
}
}
