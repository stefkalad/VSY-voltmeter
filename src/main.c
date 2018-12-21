
#include <stdint.h>
#include <stm32f303xe_DIFF.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

#include "tiny_printf.h"


//TIMER SETTINGS

#define INPUT_PIN  		 0	//PA0
#define LED_PIN_INNER 	 5	//PA5
#define MUX_PIN_A0  	 9	//PA9
#define MUX_PIN_A1  	 1	//PA1




#define DELAY_FEEDBACK  50 //(ms)
#define DELAY_SETUP 	3  //(ms)
#define DELAY_T1		20 //(ms)


//MEASURING CONSTANTS
#define VOLTAGE_REF		2400 //(mV)
#define KM_default 		VOLTAGE_REF/DELAY_T1 * 1000
#define ZEROERR_default 0

#define TIS_ns 	125U
#define IS_LOOP  5U


void rcc_init(void);
void usart_init(void);
void gpio_init(void);
void time2_init(void);

int zeroError_us = ZEROERR_default;
int KM = KM_default;


int _write(int file, char *data, int len)
{

   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      return -1;
   }
   for (int i = 0; i< len; i++){
	   while ((USART2->ISR & USART_ISR_TXE) == 0);
	   USART2->TDR = (uint16_t) *(data + i);
   }

   // return # of bytes written - as best we can tell
   //how to return false????
   return len;
}




void pinON(int pinNumber){
	GPIOA-> BSRR |= (0x1 << pinNumber);
}
void pinOFF(int pinNumber){
	GPIOA-> BSRR |= (0x1 << (pinNumber + 16));
}

void pinMuxON(int pinNumber){
	if (pinNumber ==  MUX_PIN_A0 ){
		//set OC3M to 101 force 1
		TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
		TIM2->CCMR2 |=  (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
	}

	else if (pinNumber ==  MUX_PIN_A1 ){
		//set OC2M to 101 force 1
		TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
		TIM2->CCMR1 |=  (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
	}

}
void pinMuxOFF(int pinNumber){
	if (pinNumber ==  MUX_PIN_A0 ){
		//set OC1M to 100 force 0
		TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
		TIM2->CCMR2 |=  (TIM_CCMR2_OC3M_2);
	}

	else if (pinNumber ==  MUX_PIN_A1){
		//set OC2M to 100 force 0
		TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
		TIM2->CCMR1 |=  (TIM_CCMR1_OC2M_2);
	}
}

char getChar(){

	 while ((USART2->ISR & USART_ISR_RXNE) == 0);
	 return USART2->RDR;

}
void putChar(char c){
	 while ((USART2->ISR & USART_ISR_TXE) == 0);
	 USART2->TDR = (uint16_t) c;

}

void delay(int cycles){
	cycles *= 1000;
	while (cycles-- > 0);
}
void delay_ms2(unsigned int ms){

	//clock run one one us
	ms = 65535 - ms * 1000;
	TIM2->CNT = ms;
	TIM2->SR &= ~TIM_SR_UIF;
	TIM2->CR1 |= TIM_CR1_CEN;

	while (!(TIM2->SR & TIM_SR_UIF));


}


unsigned int calculateVx(unsigned int measuredTx_us){

	unsigned int measuredVoltage_nV = (measuredTx_us - zeroError_us) * KM;
	unsigned int measuredVoltage_mV = measuredVoltage_nV/1000000;
	return measuredVoltage_mV;

}

unsigned int measureVoltage(int source){

	//STEP 01 CONNECT FEEDBACK
	// -let integrator to go near Up
	pinMuxON(MUX_PIN_A0);
	pinMuxON(MUX_PIN_A1);
	delay(DELAY_FEEDBACK);


	//STEP 02 CONNECT REFERENCE
	// -let integrator to go to + voltages
	pinMuxON(MUX_PIN_A0);
	pinMuxOFF(MUX_PIN_A1);
	delay(DELAY_SETUP);


	//STEP 03 CONNECT SOURCE/GROUND
	//-let integrator to go to - voltages
	//wait until it cross zero

	//set timer to count T1 ms
	TIM2->CNT = 65535 - (int)DELAY_T1 * 1000;
	TIM2->SR &= ~TIM_SR_UIF;
	TIM2->CR1 |= TIM_CR1_CEN;

	//select the input and set the output compare
	if (source == 0){
		pinMuxOFF(MUX_PIN_A0);
		pinMuxON(MUX_PIN_A1);

		//set OC3M to 011 to toggle go to  A0=1
		TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
		TIM2->CCMR2 |= (TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1);

		//set OC1M to 011 to toggle go to  A1=0
		TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
		TIM2->CCMR1 |= (TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1);

	}
	else if (source == 1){
		pinMuxOFF(MUX_PIN_A0);
		pinMuxOFF(MUX_PIN_A1);

		//set OC3M to 011 to toggle go to  A0=1
		TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
		TIM2->CCMR2 |= (TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1);
	}


	//wait until UIF is set -- indicates the counter overflow
	while (!(TIM2->SR & TIM_SR_UIF));

	//wait until the input gets negative (counter is stopped by HW)
	while (GPIOA->IDR & (0x1 << INPUT_PIN)); //wait until get 0
	TIM2->CR1 &= ~TIM_CR1_CEN;
	unsigned int tx_us = TIM2->CNT;
	printf("DEBUG: %u measured KM %u\r\n",tx_us, KM);

	//STEP 05 RETURN TX
	return tx_us;


}

void measure(){
	printf("MEASURMENT PROCESS IS RUNNING....\r\n");
	unsigned int vx = calculateVx(measureVoltage(1));
	printf("Measured voltage is: %u mV\r\n\n", vx);
}

void calibrate(){
	printf("CALIBRATION PROCESS IS RUNNING...\r\n");

	printf("Measuring adittive error...\r\n");
	unsigned int formerZeroError_us = zeroError_us;
	unsigned int tempZeroError_us = measureVoltage(0);
	printf("Additive constant, former %u us, now %u us (%u in mV)\r\n", formerZeroError_us, tempZeroError_us, calculateVx(tempZeroError_us));
	zeroError_us = tempZeroError_us;

	printf("Measuring multiplicative error...\r\n");
	printf("Known voltage in mV: ");
	char inputBuffer[5];
	int counter = 0;
	int vxReal = 0;
	char input;
	do{
		if (vxReal == -1) printf("\r\nBad input format!\r\n");
		do{
			input = getChar();
			putChar(input);
			inputBuffer[counter] = input;
			counter++;
		}while (input != '\r' && input != 'n' && counter < 5);

		inputBuffer[counter -1] = '\0';

	}while((vxReal = ts_atoi(inputBuffer)) == -1);

	unsigned int formerKM = KM;
	unsigned int vx = calculateVx(measureVoltage(1));

	KM = (KM * vxReal)/(vx/1000);
	printf("Multiplicative constant KM, former %u, now %u\r\n\n", formerKM, KM);

}

void sample(){
	printf("SAMPLING PROCESS IS RUNNING...\r\n");
	printf("Number of samples (<10): ");
	char inputBuffer[2];
	int samples = 0;
	do{
		if (samples == -1) printf("Bad input format!\r\n");
		inputBuffer[0] = getChar();
		putChar(inputBuffer[0]);
		printf("\r\n");
	}while((samples = ts_atoi(inputBuffer)) == -1);

	unsigned int vxSum = 0;
	for (int i = 0; i < samples; i++){
		vxSum += calculateVx(measureVoltage(1));
	}
	vxSum /= samples;

	printf("Measured voltage of %u samples is: %u mV\r\n\n", samples, vxSum);

}

int main(void)
{


	/* Reconfigure system clock */
	rcc_init();

	/* Initialize all used peripherals */
	gpio_init();
	usart_init();
	time2_init();


	printf("-------------------VOLTMETER -------------------\r\n");
	printf("To start measurment please type M to terminal!\r\n\n");


	while (1) {
		printf("INPUT: ");
		char select = getChar();

		switch (select){
			case 'm':
			case 'M':
				printf("%c\r\n", select);
				measure();
				break;
			case 'c':
			case 'C':
				printf("%c\r\n", select);
				calibrate();
				break;
			case 's':
			case 'S':
				printf("%c\r\n", select);
				sample();
				break;
			case '\n':
			case '\r':
				printf("\r\n");
				break;
			default:
				printf("WRONG INPUT: %c \r\n PRESS C for CALIBRATION, PRESS M for MEASURMENT, S for SAMPLING...\r\n", select);
				break;

		}

	}
}

/**
 * Procedure for initializing the RCC system.
 * The HSE is setup as the main system clock.
 */
void rcc_init(void)
{
	/* Select the HSE Bypass and the HSE clock source. */
	RCC->CR |= RCC_CR_HSEBYP;
	/* Switch on the HSE clock. */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait until the HSE clock is ready. */
	while ((RCC->CR & RCC_CR_HSERDY) == 0);

	/* Switch the main clock to the HSE clock source. */
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;

	/* Wait until the clock source is switched to HSE. */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);

	/* Optionally turn off the HSI oscillator to save the power. */
	RCC->CR &= ~RCC_CR_HSION;
}

/**
 * Procedure for initializig the GPIOA.
 * All the pins used by peripherals which requires special settings are
 * configured here in advance.
 */
void gpio_init(void)
{

	/* Switch on the clock for GPIOA. We are going to use PA2 for TX (AF7) and PA3
	 * for RX (AF7) */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;


	//Define MUX pins and Led pin
	GPIOA-> MODER |= (1 << LED_PIN_INNER * 2);
	//GPIOA-> MODER |= (0x1 << MUX_PIN_A0 * 2);
	//GPIOA-> MODER |= (0x1 << MUX_PIN_A1 * 2);


	//Define mux pins
	GPIOA-> MODER |= (2 << MUX_PIN_A0 * 2);
	GPIOA-> MODER |= (2 << MUX_PIN_A1 * 2);
	GPIOA-> AFR[1] |= (10<< (MUX_PIN_A0-8) * 4); 	//AF1 -TIM2_CH1 for mux pin
	GPIOA-> AFR[0] |= (1 << MUX_PIN_A1 * 4); 		//AF1 -TIM2_CH2 for mux pin
	GPIOA-> OSPEEDR |= (3 << MUX_PIN_A0 * 2);
	GPIOA-> OSPEEDR |= (3 << MUX_PIN_A1 * 2);


	//Define input pin
	GPIOA-> MODER |= (2 << INPUT_PIN * 2);
	GPIOA-> AFR[0] |= (1 << INPUT_PIN * 4); //AF10 -TIM2_CH3 for input pin
	GPIOA-> PUPDR |= (2 << INPUT_PIN *2);


	//Define USART pins
	/* Set the corresponding GPIO pins to alternate function mode */
	GPIOA->MODER |= (2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos);
	/* Select the alternate function to AF7 */
	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFRL2_Pos) | (7 << GPIO_AFRL_AFRL3_Pos);


}

void time2_init(void)
{
	/* General purpose TIM2 is used
	 * TIM2CH1 is used with INPUT_PIN in triggered gated mode - AF1
	 * TIM2CH2 is used with MUX_PIN_A0 in output compare mode - AF10
	 * TIM2CH3 is used with MUX_PIN_A1 in output compare mode - AF1
	 */
	//enable TIM2 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//set TIM2 prescaler
	TIM2->PSC = 7;
	//set TIM2 CNT overflow
	TIM2->ARR = 65535;
	//set input trigger: SMS - Slave gated mode and TS to Filtered Timer Input
	TIM2->SMCR = 0x55;


	//enable mux pins TIM2_CH2 and TIM2_CH3 in output capture compare mode
	TIM2->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3E);

	//set the compare level
	TIM2-> CCR2 = 0;
	TIM2-> CCR3 = 0;

	//clear output compare mode bits (it should be cleared by default)
	TIM2->CCMR1 &= (~TIM_CCMR1_OC2CE & ~TIM_CCMR1_OC2PE & ~TIM_CCMR1_OC2FE); //clear bits
	TIM2->CCMR2 &= (~TIM_CCMR2_OC3CE & ~TIM_CCMR2_OC3PE & ~TIM_CCMR2_OC3FE); //clear bits


	//TIM2->EGR |= TIM_EGR_UG;
	//TIM2->DIER = TIM_DIER_UIE;





}

/**
 * Procedure for initializing the USART2
 */
void usart_init(void)
{
	/* Enable the clock source for the USART2 peripheral in RCC */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	/* The default configuration state of the USART is:
	 * databits = 8
	 * parity = none
	 * start bits = 1
	 * stop bits = 1
	 * oversampling = 16
	 */
	/* Configure the USART frame eg. the parity, type of the parity etc. */
	//USART2->CR1 |= USART_CR1_PCE | USART_CR1_PS;

	/* Configure the baudrate. In case of oversampling by 16 the baudrate is:
	 * B = 2 * f / r, where r is the value of baudrate register BRR and f is clock
	 * frequency of the USART2 peripheral, which is in that case 8MHz.
	 */
	USART2->BRR = 833U;

	/* Now we can enable the USART2 peripheral and both, the transmitter and
	 * the receiver.
	 */
	USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}
