// 58% DE CICLO DE TRABAJO COMO 0 PARA EL MOTOR

#include <stm32f103xb.h>
#include <stdlib.h>



//#define motorOut TIM3->CCR4
unsigned short value=0;
unsigned int indexUsartRX3=0;
unsigned int indexUsartTX1=1;


int inUSARTdata1;
int inUSARTdata3;
int L;
float sensorInputData;
int PWMzero=870;
int PWMmax=950;
float sensorInputDataOffSet;
float sensorInputDataArray[10];		
unsigned int sensorInputDataArrayPointeri=0, sensorInputDataArrayPointerj=0;
unsigned int USART1index=0;
unsigned int sendDataUartFlag=0;
int sensorInputDataTemp;

// Variables controlador
float kp=0.3; 												// Constante proporcionAL
float ki=0;													// Constante Integral
float kd=0;													// Constante derivativa
float kn=0;													// Constante del filtro derivativo
int finiteIntegratorWindowsEnable=0;						// Selector de tipo de ventana de integracion de Ki 
int finiteDerivativeWindowsEnable=0;						// Selector de tipo de ventana de integracion de Kd 
float errorArray[500];										// Arreglo de error
float derivativeArray[500];									// Arreglo del derivativo
float derivative=0;											// Componnte derivativa
int errorArrayPointer=0;									
int errorArrayPointerMax=499;
int derivativeArrayPointer=0;
int derivativeArrayPointerMax=499;
float error=0;
float integratorSum=0;
float derivativeSum=0;
unsigned int Ts=50;



// P1: 40  kp:0.7 ref:100
// P2: 80  kp:0.2 ref:270
// P3: 120 kp:0.3 ref:260
// P4: 160 kp:0.3 ref:300
// P5: 200 kp:0.3 ref:350
// P6: 240 kp:0.3 ref:380
// P7: 280 kp:0.3 ref:420

float pastError=0;
int ref=160;
int newRef=200;
float out=0;
unsigned short refOffSet=100;

unsigned int Ti=0;
unsigned int initLoopCounter=0;

void controlLoop(void);
void initializationLoop(void);
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

	//--PC13
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


	// Configuracion
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

	USART1->DR = (0x04);									// Primera transmision basura
	while(!(USART1->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision
	

	//--Inicialización UART3-------------------------------------------------------------------------------

	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; 					// USART3EN=1, Activación de reloj UART3
	USART3->CR1 |= USART_CR1_UE;       						// UE=1, Activación UART3
	USART3->CR1 &= ~USART_CR1_M;  							// M=0, Tamaño de palabra 8 bits
	USART3->CR2 &= ~USART_CR2_STOP;    						// STOP=0, Seleccionar el número de bits de parada

	//	Inicializacion de pines (GPIOA)

	GPIOB->CRH  = 0;
	GPIOB->CRH |= GPIO_CRH_MODE10_0;   						// MODE10 = 01, PB10 como salida a 10 MHz
	GPIOB->CRH |= GPIO_CRH_CNF10_1;    						// CNF10 = 10, PB10 como salida push pull alternada (salida de periferico)
	GPIOB->CRH &= ~GPIO_CRH_MODE11;    						// MODE11 = 00, PB11 como entrada
	GPIOB->CRH |= GPIO_CRH_CNF11_1;    						// CNF11 = 10, PB11 como entrada push pull

	//	Baudios 9600, Baudios = 36MHz/16*USARTDIV, USARTDIV = 234,375

	USART3->BRR = (0x0EA6); 								// Parte entera y decimal del preescaler de Baudios
	USART3->CR1 |= USART_CR1_TE;							// TE=1, Habilitar transmisor
	USART3->CR1 |= USART_CR1_RE;							// RE=1, Habilitar receptor
	//USART1->CR1 |= USART_CR1_RXNEIE;						// RXNEIE=1, Habilitar interrupciones de recepcion

	//	Primera transmision (vacia)

	USART3->DR = (0x04);									// Primera transmision basura
	while(!(USART3->SR & USART_SR_TC)); 					// TC=1? Esperar a que se complete la transmision

	// Inicializacion boton de ajuste referencia


	GPIOB->CRH &= ~GPIO_CRH_MODE12;							// MODE12=0b00, PB12 en modo de entrada
	GPIOB->CRH |= GPIO_CRH_CNF12_1;							// CNF12=0b10, PB12 en modo de de Pull-up, Pull-down
	GPIOB->ODR |= GPIO_ODR_ODR12;							// Activar resistencia de Pull-Up
//-----------------------------------------------------------------------------------------------------

	TIM3->CCR4 =PWMzero;									// Iniciar con el ventilador apagado
	TIM3->CCR4 =PWMmax;
	TIM3->CCR4 =PWMzero;
	
	//NVIC_EnableIRQ(USART1_IRQn);							// Habilitar interrupcion de USART1
	
	// Inicilizacion del arreglo del integrador:
	for(int i=0;i<errorArrayPointerMax;i++){
		errorArray[i]=0;
	}
	while(1){
		if(initLoopCounter<50){
			initializationLoop();
		}else if(initLoopCounter==50){
			sensorInputDataOffSet=sensorInputData;
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR =0x0A;
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR =0x0D;
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR ='S';
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR ='T';
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR ='A';
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR ='R';
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR ='T';
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR =0x0A;
			while(!(USART1->SR & USART_SR_TC));
			USART1->DR =0x0D;

			controlLoop();
			//TIM3->CCR4 =PWMmax;
			initLoopCounter++;
		}else{
			controlLoop();
		}

	}
}


void wait_us(unsigned int time){ //455 valor maximo
	SysTick->LOAD =(time*9)-1; 								// Load=Time*8-1, ajustar valor del contador
	SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;				// Enable=1, Habilitar contador
	while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));	// COUNTFLAG=1?,Preguntar si ya se alcanzo 0
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;				// Enable=0, Deshabilitar el contador
}


void initializationLoop(void){
	// Recepcion de datos UART-sensor
	if(USART3->SR & USART_SR_RXNE){						
		inUSARTdata3=USART3->DR;
		switch(indexUsartRX3){
			case 0:
				if(inUSARTdata3==0xF0){
					indexUsartRX3++;
				}else{
					indexUsartRX3=0;
				}
				break;
			case 1:
				if(inUSARTdata3==0xF0){
					indexUsartRX3++;
				}else{
					indexUsartRX3=0;
				}
				
				break;
			case 2:
				if(inUSARTdata3==0xF0){
					indexUsartRX3++;
				}else{
					indexUsartRX3=0;
				}
				break;
			case 3:
				sensorInputDataTemp=0;
				sensorInputDataTemp=inUSARTdata3<<8;
				indexUsartRX3++;
				break;
			case 4:
				sensorInputDataTemp+=inUSARTdata3;

				// Filtro
				sensorInputDataArray[sensorInputDataArrayPointeri]=sensorInputDataTemp;
				sensorInputDataArrayPointeri = (sensorInputDataArrayPointeri<9) ? sensorInputDataArrayPointeri +1 : 0; 
				initLoopCounter++;
				sensorInputData=0;
				for(int i=0;i<10;i++){
					sensorInputData+=sensorInputDataArray[i];
				}
				sensorInputData=sensorInputData/10;

				indexUsartRX3=0;
				break;
			default:
				indexUsartRX3=0;
				break;
		}

	}

	// Transmision de datos UART -UI

	if(USART1->SR & USART_SR_TC){						
		switch(indexUsartTX1){
			case 0:
				USART1->DR ='L';
				indexUsartTX1++;
				break;
			case 1:
				USART1->DR = ((int)sensorInputData/1000)+48;
				indexUsartTX1++;
				break;
			case 2:
				USART1->DR = (((int)sensorInputData%1000)/100)+48;
				indexUsartTX1++;
				break;
			case 3:
				USART1->DR = ((((int)sensorInputData%1000)%100)/10)+48;
				indexUsartTX1++;
				break;
			case 4:
				USART1->DR = ((((int)sensorInputData%1000)%100)%10)+48;
				indexUsartTX1++;
				break;
			case 5:
				USART1->DR = 0x0A;
				indexUsartTX1++;
				break;
			case 6:
				USART1->DR = 0x0D;
				indexUsartTX1=0;
				break;
			default:
				indexUsartTX1=0;
				break;
		}
	}
	
}
void controlLoop(void){

	// Verificacion de referencia
	if( !(GPIOB->IDR & GPIO_IDR_IDR12) ){
		ref=newRef;
	}

	// Recepcion de datos UART-sensor
	if(USART3->SR & USART_SR_RXNE){						
		inUSARTdata3=USART3->DR;
		switch(indexUsartRX3){
			case 0:
				if(inUSARTdata3==0xF0) indexUsartRX3++;
				break;
			case 1:
				if(inUSARTdata3==0xF0) indexUsartRX3++;
				break;
			case 2:
				if(inUSARTdata3==0xF0) indexUsartRX3++;
				break;
			case 3:
				sensorInputDataTemp=0;
				sensorInputDataTemp=inUSARTdata3<<8;
				indexUsartRX3++;
				break;
			case 4:
				sensorInputDataTemp+=inUSARTdata3;

				// Filtro
				sensorInputDataArray[sensorInputDataArrayPointeri]=sensorInputDataOffSet-sensorInputDataTemp;
				sensorInputDataArrayPointeri = (sensorInputDataArrayPointeri<9) ? sensorInputDataArrayPointeri +1 : 0; 
				initLoopCounter++;
				sensorInputData=0;
				for(int i=0;i<10;i++){
					sensorInputData+=sensorInputDataArray[i];
				}
				sensorInputData=sensorInputData/10;

				indexUsartRX3=0;
				break;
			default:
				indexUsartRX3=0;
				break;
		}

	}
	// Recepcion de datos UART-UI
	if(USART1->SR & USART_SR_RXNE){						
		inUSARTdata1=USART1->DR;
		

	}
	// Transmision de datos UART-UI
	if(sendDataUartFlag==1){
		if(ref==newRef){
			if(USART1->SR & USART_SR_TC){						
				switch(indexUsartTX1){
					case 0:
						USART1->DR ='P';
						indexUsartTX1++;
						break;
					case 1:
						USART1->DR ='M';
						indexUsartTX1++;
						break;
				    case 2:
						USART1->DR =' ';
						indexUsartTX1++;
						break;
					case 3:
						USART1->DR = ((int)sensorInputData/1000)+48;
						indexUsartTX1++;
						break;
					case 4:
						USART1->DR = (((int)sensorInputData%1000)/100)+48;
						indexUsartTX1++;
						break;
					case 5:
						USART1->DR = ((((int)sensorInputData%1000)%100)/10)+48;
						indexUsartTX1++;
						break;
					case 6:
						USART1->DR = ((((int)sensorInputData%1000)%100)%10)+48;
						indexUsartTX1++;
						break;
					case 7:
						USART1->DR =' ';
						indexUsartTX1++;
						break;					
					case 8:
						USART1->DR = (((int)out)/1000)+48;
						indexUsartTX1++;
						break;
					case 9:
						USART1->DR = ((((int)out)%1000)/100)+48;
						indexUsartTX1++;
						break;
					case 10:
						USART1->DR = (((((int)out)%1000)%100)/10)+48;
						indexUsartTX1++;
						break;
					case 11:
						USART1->DR = (((((int)out)%1000)%100)%10)+48;
						indexUsartTX1++;
						break;
					case 12:
						USART1->DR = 0x0A;
						indexUsartTX1++;
						break;
					case 13:
						USART1->DR = 0x0D;
						indexUsartTX1=0;
						sendDataUartFlag=0;
						break;
					default:
						indexUsartTX1=0;
						break;
				}
			}
		}else{

			if(USART1->SR & USART_SR_TC){						
				switch(indexUsartTX1){
					case 0:
						USART1->DR ='P';
						indexUsartTX1++;
						break;
					case 1:
						USART1->DR ='R';
						indexUsartTX1++;
						break;
					case 2:
						USART1->DR =' ';
						indexUsartTX1++;
						break;
					case 3:
						USART1->DR = ((int)sensorInputData/1000)+48;
						indexUsartTX1++;
						break;
					case 4:
						USART1->DR = (((int)sensorInputData%1000)/100)+48;
						indexUsartTX1++;
						break;
					case 5:
						USART1->DR = ((((int)sensorInputData%1000)%100)/10)+48;
						indexUsartTX1++;
						break;
					case 6:
						USART1->DR = ((((int)sensorInputData%1000)%100)%10)+48;
						indexUsartTX1++;
						break;
					case 7:
						USART1->DR =' ';
						indexUsartTX1++;
						break;					
					case 8:
						USART1->DR = (((int)out)/1000)+48;
						indexUsartTX1++;
						break;
					case 9:
						USART1->DR = ((((int)out)%1000)/100)+48;
						indexUsartTX1++;
						break;
					case 10:
						USART1->DR = (((((int)out)%1000)%100)/10)+48;
						indexUsartTX1++;
						break;
					case 11:
						USART1->DR = (((((int)out)%1000)%100)%10)+48;
						indexUsartTX1++;
						break;
					case 12:
						USART1->DR = 0x0A;
						indexUsartTX1++;
						break;
					case 13:
						USART1->DR = 0x0D;
						indexUsartTX1=0;
						sendDataUartFlag=0;
						break;
					default:
						indexUsartTX1=0;
						break;
				}
			}

		}
	}
	// Lectura ADC
	if(ADC1->SR & ADC_SR_EOC){							// EOC=1? Preguntar si ya termino la conversion
		value=ADC1->DR & ADC_DR_DATA;					// :DATA, Almacenar los datos de la conversion
		//ref=refOffSet+(value/16);						// Modificacionde referencia
	};

	// Muestreo 
	if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk || Ti==0){
		SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;				// Enable=0, Deshabilitar el contador
		SysTick->LOAD =(10*9)-1;								// Contador
		SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;				// Enable=1, Habilitar contador
		if(Ti==Ts*100){
			Ti=0;
			GPIOC->ODR ^= GPIO_ODR_ODR13;

			// Control PID

			
			error=ref-sensorInputData;
			
			// Integrador:
			integratorSum+=error;
			if(finiteIntegratorWindowsEnable){
				if(errorArrayPointer<errorArrayPointerMax){
					errorArrayPointer++;
				}else{
					errorArrayPointer=0;
				}
				integratorSum=integratorSum-errorArray[errorArrayPointer];
				errorArray[errorArrayPointer]=error;
				integratorSum+=errorArray[errorArrayPointer];
			}else{
				integratorSum+=error;
			}

			// Derivador:

			if(finiteDerivativeWindowsEnable){
				if(derivativeArrayPointer<derivativeArrayPointerMax){
					derivativeArrayPointer++;
				}else{
					derivativeArrayPointer=0;
				}
				derivative=kn*((error*kd)-((Ts/1000)*derivativeSum));
				derivativeSum=derivativeSum-derivativeArray[derivativeArrayPointer];
				
				derivativeArray[derivativeArrayPointer]=derivative;
				derivativeSum+=derivativeArray[derivativeArrayPointer];
			}else{
				derivative=kd*(1000/Ts)*(error-pastError);
				pastError=error; 
			}
			// salida 
			out= kp*(error)+ (ki*(Ts/1000)*integratorSum)+ derivative;
			
			pastError=error;

			if(out >(PWMmax-PWMzero)){
				out = PWMmax-PWMzero;
			}else if(out<0){
				out = 0;
			}

			// Controlador ON-OFF
			/*
			error=ref-sensorInputData;

			if(error<5){
				out=2;				
			}else if(error>5){
				out=58;
			}
			*/

			TIM3->CCR4 =PWMzero+ out;

			// Habilitar mandar datos por UART

			sendDataUartFlag=1;
		}else{
			Ti++;
		}
	}
}

//void USART1_IRQHandler(void){
//	if(USART1->SR & USART_SR_RXNE){
//		inUSARTdata3=USART1->DR;								// Almacenar dato de entrada
//
//	}
//}

