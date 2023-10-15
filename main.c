#include <LPC17xx.H>

#define Fpclk 25e6	// Fcpu/4 (defecto despu?s del reset)
#define Tpwm 20e-3	// Perido de la se?al PWM (15ms)

uint8_t MIN = 0;
uint8_t MAX = 180;

uint8_t estado = 0;
uint32_t grados = 0;

void config_EINT0(void){
		LPC_PINCON->PINSEL4|=(0x01 << 20); //asocia la interrupcion al pin del bot?n P2.10
		LPC_SC -> EXTMODE |= (1 << 0); // rising edge
		//LPC_SC -> EXTPOLAR |= (1 << 0); //flanco de subida
		NVIC->ISER[0] |= (1 << 18); //Enable interrupcion
	}
	
void config_pwm1(void){
		LPC_PINCON->PINSEL3|=(2<<4); // P1.18 salida PWM (PWM1.1)
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

void config_Systick(){
	
	
	}

uint16_t get_IR_distance(){
		uint16_t Distancia = 0;
		return Distancia;
	}
	
void set_servo(float Grados){
    LPC_PWM1->MR2=(Fpclk*1e-3 + Fpclk*1e-3*Grados/180); //Var?a en funci?n de los grados el Duty cicle desde 100/15 % a 200/15 % 
		LPC_PWM1->LER|=(1<<2)|(1<<0); //Enable el Match 0 y el Match 2 (MR0 and MR2)
	}	
	
void SystickIRQHandler(){
		uint16_t distancia = 0;
		static uint8_t direccion = 0; // 0->Gira derecha  1->Gira izquierda
	
		if(estado == 12 & MAX>MIN){ //Si modo automático 
			grados = MIN;
			if(direccion == 0){ //Si va a la derecha
				grados = grados + 10; //aumenta en 10 los grados
				set_servo(grados); //mueve el servo a esa posicion
			}
			else{
				grados = grados - 10;	//disminuye en 10 los grados
				set_servo(grados);	//mueve el servo a esa posicion
			}
			if(grados < MIN+1 | grados > MAX-1){ //si los grados están en las posiciones límite 
				direccion ^= 1; //cambia la dirección
			}
		}
		
		distancia = get_IR_distance(); //saca la distancia
		//display(distancia);	//la pone en el display
	
	}

	void EINT0_IRQHandler(){ 

		uint8_t estado_siguiente = 0;
		LPC_SC->EXTINT|=1;//limpiar la flag
		
			if(estado == 2){ // 2->3
				//activar timer
				grados = 5*LPC_QEI->QEIPOS;
				set_servo(grados); //if click -> QUEIPOS= QUEIPOS +2
				//LPC_SC->PCONP = LPC_SC->PCONP & 0xFFFBFFFF; //desactivar encoder
				estado_siguiente = 3;
			}
			if(estado == 3){ // 3->2
				//desactivar timer
				LPC_SC->PCONP|=(1<<18); //activar encoder
				estado_siguiente = 2;
			}
			if(estado == 10){ // 10->11
				MAX = 5*LPC_QEI->QEIPOS;
				estado_siguiente = 11;
			}
			if(estado == 11){ // 11->12
				MIN = 5*LPC_QEI->QEIPOS;
				estado_siguiente = 12;
			}
			if(estado == 12){ // 12->13
				//activar timer
				estado_siguiente = 13;
			}
			if(estado == 13){ // 13->12
				//desactivar timer
				estado_siguiente = 12;
			}
			
			estado = estado_siguiente;
			estado_siguiente = 0;
		
	}
	
int main(void)
{
	//Config mode
	uint8_t mode = 0; // 0->Manual 1->Automatic
	mode = 1^(((1<<11)&(LPC_GPIO2->FIOPIN))>>11); //if P2.11 is pushed -> mode = 1;
	if(mode == 0){
		//config_manual();
		estado = 2; //Estado en el que el encoder mide
	}
	else{
		//config_automatic();
		estado = 10; //Estado en el que el encoder mide MAX
	}
	
	config_EINT0();
	config_pwm1();
	config_encoder();
	config_Systick();
	
	while(1);
}