// 58% DE CICLO DE TRABAJO COMO 0 PARA EL MOTOR

#include <stm32f103xb.h>
#include <stdlib.h>

unsigned short value=0;
unsigned int indexRX=0;
unsigned int indexTX=1;


int inUSARTdata;
int L;
int Ldata[10];
int Ldatai=0, Ldataj=0;
int USART1index=0;
int RXaux;

// Variables controlador
float kp=2; //3.9
float ki=0;
float kd=0;

float pastError;
int ref=150;
float out=0;
unsigned short refOffSet=100;

unsigned int Ti=0;


void wait_us(unsigned int time);

int main(void){

	//--Inicialización interface debug---------------------------------------------------------------------

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 					// AFIOEN = 1, activar reloj de modulo AFIO
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;						// SWJ_CFG = 010 JTAG disabled y SW-DP enable

	//--Inicializacion de reloj externo a 72 Mhz-----------------------------------------------------------

	FLASH->ACR |=FLASH_ACR_LATENCY_1;						// Ajustar Latencia de memoria Flash en 2 (Debido al ajuste de alta frecuencia)
	RCC->CR |= RCC_CR_HSEON; 								// HSE0N = 1, Habilitar reloj externo
    while(!(RCC->CR & RCC_CR_HSERDY));   					// HSERDY = 1 ?, Esperar a que el reloj externo se estabilice
    RCC->CFGR |= RCC_CFGR_PLLSRC; 							// PLLSRC=1, Seleccionar reloj externo como fuente de reloj de PLL
    RCC->CFGR |= RCC_CFGR_PLLMULL9;							// PLLMUL=7, Seleccionar 9 como factor PLL
    RCC->CR	  |= RCC_CR_PLLON;								// PLLON=1 Encender PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));						// PLLRDY=1 ? Esperar a que se ajuste el PLL
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; 						// PPRE1=0b100, Ajustar el prescaler de APB1, para obtener 36Mhz
    RCC->CFGR |= RCC_CFGR_SW_1;    							// SW=2, PLL seleccionado como reloj del sistema
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); 				// SWS=2? , Esperar a que el reloj del sistema se configure

	//--C13
    RCC->APB2ENR |=RCC_APB2ENR_IOPCEN;						// IOPCEN=1, Habilitar reloj del Puerto A
    GPIOC->CRH = 0;											// Limpiar el puerto C
    GPIOC->CRH &= GPIO_CRH_MODE13;							// MODE10=0x11, PC13 en modo de salida
    GPIOC->CRH |= ~GPIO_CRH_CNF13;							// CNF0=0x00, PC13 en modo de entrada analoga
    GPIOC->ODR |= GPIO_ODR_ODR13;							// Salida en uno
    //--Inicialización ADC1--------------------------------------------------------------------------------

	RCC->CFGR |= RCC_CFGR_ADCPRE_1;							// ADCPRE=2, Prescaler de reloj ADC = 6, ADCCLK= 12Mhz
	RCC->APB2ENR |=RCC_APB2ENR_ADC1EN;						// ADC1EN=1, Habilitar reloj del ADC1

	//  Inicilizacion de Pines GPIOA

	RCC->APB2ENR |=RCC_APB2ENR_IOPAEN;						// IOPAEN=1, Habilitar reloj del Puerto A
	GPIOA->CRH = 0;											// Limpiar el puerto A
	GPIOA->CRL &= ~GPIO_CRL_MODE0;							// MODE0=0, PA0 en modo de entrada
	GPIOA->CRL &= ~GPIO_CRL_CNF0;							// CNF0=0, PA0 en modo de entrada analoga

	// Configuracion

	ADC1->CR2 |= ADC_CR2_ADON;								// ADON=1, Encendido del conversor
	for(unsigned int i=0; i<12;i++);						// Espera de 2 ciclos de reloj de ADC (12 de sistema) para calibracion
	ADC1->CR2 |= ADC_CR2_CAL;								// CAL=1, Iniciar Calibracion
	while(ADC1->CR2 & ADC_CR2_CAL);							// CAL=0?, Esperar a que termine la calibracion
	ADC1->CR2 |= ADC_CR2_CONT;								// CONT=1, Activar modo de conversion continua
	ADC1->SQR1 &= ~ADC_SQR1_L;								// L=0, Seleccion de un solo canal para secuencia de conversion regular
	ADC1->SQR3 &= ~ADC_SQR3_SQ1;							// SQ1=0, Seleccion de primer(unico^) canal de secuencia de conversion regular
	ADC1->CR2 |= ADC_CR2_ADON;								// ADON=1, Iniciar conversion continua

	//--Inicializacion PWM (Timer 3)-----------------------------------------------------------------------
	RCC->APB1ENR |=RCC_APB1ENR_TIM3EN;						// Habilitar reloj de Timer 3

	// Ajuste de pines (pin B1-TIM3-Canal4)

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;						// Habilitar de puerto B
	GPIOB->CRL = 0;											// Limpiar el registro
	GPIOB->CRL |= GPIO_CRL_MODE1;							// MODE1=0b11, PB1 Modo salida 50 MHz
	GPIOB->CRL |= GPIO_CRL_CNF1_1;							// CNF1=0b10, PB1 Modo alternado	 push-pull


	// COnfiguracion
	TIM3->CCER |= TIM_CCER_CC4E;							// Habilitar canal 4
	TIM3->CR1 |= TIM_CR1_ARPE;								// Habilitar Auto recarga de valor precarga
	TIM3->CCMR2|=TIM_CCMR2_OC4M_2+TIM_CCMR2_OC4M_1;			// AJustar modo PWM1 en canal 4
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;							// Habilitar precarga del ciclo de trabajo  en canal 4, del registro sombra al real
	TIM3->PSC = 7;											// Ajustar prescaler en 4
	TIM3->ARR = 1024;										// Ajustar frecuencia f=Fmax/PSC/ARR-> f=8,789Khz
	TIM3->CCR4 = 512;										// Ajustar ciclo de trabajo al 50% en canal 4
	TIM3->EGR |= TIM_EGR_UG;								// Reinicializar contador y actualizar registros
	TIM3->CR1 |= TIM_CR1_CEN;								// Habilitar contador

	//--Inicialización UART1-------------------------------------------------------------------------------

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 					// USART1EN=1, Activación de reloj UART1
	USART1->CR1 |= USART_CR1_UE;       						// UE=1, Activación UART1
	USART1->CR1 &= ~USART_CR1_M;  							// M=0, Tamaño de palabra 8 bits
	USART1->CR2 &= ~USART_CR2_STOP;    						// STOP=0, Seleccionar el número de bits de parada

	//	Inicializacion de pines (GPIOA)

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;						// IOPAEN = 1, Habilitar reloj de GPIOA
	GPIOA->CRH |= GPIO_CRH_MODE9_0;    						// MODE9 = 01, PA9 como salida a 10 MHz
	GPIOA->CRH |= GPIO_CRH_CNF9_1;     						// CNF9 = 10, PA9 como salida push pull alternada (salida de periferico)
	GPIOA->CRH &= ~GPIO_CRH_MODE10;    						// MODE10 = 00, PA10 como entrada
	GPIOA->CRH |= GPIO_CRH_CNF10_1;    						// CNF10 = 10, PA10 como entrada push pull

	//	Baudios 9600, Baudios = 72MHz/16*USARTDIV, USARTDIV = 468,75

	USART1->BRR = (0x1D4C); 								// Parte entera y decimal del preescaler de Baudios
	USART1->CR1 |= USART_CR1_TE;							// TE=1, Habilitar transmisor
	USART1->CR1 |= USART_CR1_RE;							// RE=1, Habilitar receptor
	//USART1->CR1 |= USART_CR1_RXNEIE;						// RXNEIE=1, Habilitar interrupciones de recepcion

	//	Primera transmision (vacia)

	USART1->DR = (0x04);									// Datos TX UART1
	while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision


//-----------------------------------------------------------------------------------------------------


	TIM3->CCR4 =1023;
	//TIM3->CCR4 =1023;

	USART1->DR ='L';										// Datos TX UART1
	//TIM3->CCR4 =0;

	//NVIC_EnableIRQ(USART1_IRQn);							// Habilitar interrupcion de USART1
	while(1){

		if(USART1->SR & USART_SR_RXNE){						// Recepcion
			inUSARTdata=USART1->DR;
			switch(indexRX){
				case 0:
					if(inUSARTdata==0x01) indexRX++;
					break;
				case 1:
					if(inUSARTdata==0x01) indexRX++;
					break;
				case 2:
					RXaux=inUSARTdata<<8;
					indexRX++;
					break;
				case 3:
					L=RXaux+inUSARTdata;
					L=L;

					// Filtro
//					Ldata[Ldatai]=L;
//						L=0;
//					for(int i=0;i<10;i++){
//						L+=Ldata[i]/10;
//					}
//					Ldatai= Ldatai<10 ? Ldatai +1 : 0;

					indexRX=0;
					break;
				default:
					indexRX=0;
					break;
			}

		}

		if(USART1->SR & USART_SR_TC){						// Transmision
			switch(indexTX){
				case 0:
					USART1->DR ='L';
					indexTX++;
					break;
				case 1:
					USART1->DR ='M';
					indexTX++;
					break;
				case 2:
					USART1->DR =':';
					indexTX++;
					break;
				case 3:
					USART1->DR = (L/1000)+48;
					indexTX++;
					break;
				case 4:
					USART1->DR = ((L%1000)/100)+48;
					indexTX++;
					break;
				case 5:
					USART1->DR = (((L%1000)%100)/10)+48;
					indexTX++;
					break;
				case 6:
					USART1->DR = (((L%1000)%100)%10)+48;
					indexTX++;
					break;
				case 7:
					USART1->DR = 0x0A;
					indexTX++;
					break;
				case 8:
					USART1->DR = 0x0D;
					indexTX=0;
					break;
				default:
					indexTX=0;
					break;
			}
		}
		// Lectura ADC
		if(ADC1->SR & ADC_SR_EOC){							// EOC=1? Preguntar si ya termino la conversion
			value=ADC1->DR & ADC_DR_DATA;					// :DATA, Almacenar los datos de la conversion
			//ref=refOffSet+(value/16);						// Modificacionde referencia
		};

		// Muestreo 5ms
		if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk || Ti==0){
			SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;				// Enable=0, Deshabilitar el contador
			SysTick->LOAD =(10*9)-1;								// Contador
			SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;				// Enable=1, Habilitar contador
			if(Ti==1000){
				Ti=0;
				GPIOC->ODR ^= GPIO_ODR_ODR13;

				// Control PID
				int aux1= TIM3->CCR4;
				out= kp*(ref-L)+ ki*(ref-L+pastError)+kd*(ref-L-pastError);
				pastError+=(ref-L);
				if(out+aux1 >1023){
					TIM3->CCR4 = 1023;
				}else if(out+aux1 <0){
					TIM3->CCR4 = 0;
				}else{
					TIM3->CCR4 += out;
				}

// 				Control ON_OFF
//				if( abs(ref-(350-L))>0 ){
//					if(ref<(450-L)){
//						TIM3->CCR4 = 700;
//					}else{
//						TIM3->CCR4 = 1023;
//					}
//				}

			}else{
				Ti++;

			}

		}



	}
}


void wait_us(unsigned int time){ //455 valor maximo
	SysTick->LOAD =(time*9)-1; 								// Load=Time*8-1, ajustar valor del contador
	SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;				// Enable=1, Habilitar contador
	while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));	// COUNTFLAG=1?,Preguntar si ya se alcanzo 0
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;				// Enable=0, Deshabilitar el contador
}


//void USART1_IRQHandler(void){
//	if(USART1->SR & USART_SR_RXNE){
//		inUSARTdata=USART1->DR;								// Almacenar dato de entrada
//
//	}
//}

