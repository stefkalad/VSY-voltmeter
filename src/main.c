
#include <stdint.h>
#include  <errno.h>
#include <stm32f303xe_DIFF.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO




#define INPUT_PIN  		 6
#define LED_PIN_INNER 	 5
#define MUX_PIN_A0  	 0
#define MUX_PIN_A1  	 1
#define LED_PIN_RESTART  4



#define DELAY_FEEDBACK  50 //(ms)
#define DELAY_SETUP 	1  //(ms)
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
      errno = EBADF;
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

char getChar(){

	 while ((USART2->ISR & USART_ISR_RXNE) == 0);
	 return USART2->RDR;

}

void putChar(char c){
	 while ((USART2->ISR & USART_ISR_TXE) == 0);
	 USART2->TDR = (uint16_t) c;

}

void delay(int cycles){
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
void delay_ms(unsigned int ms){

	//uint32_t cnt = ms * 8000;
	uint32_t cnt = ms * 1000;

	TIM2->CNT = 0u;
	while (TIM2->CNT < cnt);
	unsigned int temp = TIM2->CNT;
		printf("DEBUG: CNT %u \r\n", temp);

}



unsigned int calculateVx(unsigned int measuredTx_us){

	unsigned int measuredVoltage_nV = (measuredTx_us - zeroError_us) * KM;
	unsigned int measuredVoltage_mV = measuredVoltage_nV/1000000;
	return measuredVoltage_mV;

}

unsigned int measureVoltage(int source){

	//STEP 01 CONNECT FEEDBACK
	// -let integrator to go near Up
	pinON(MUX_PIN_A0);
	pinON(MUX_PIN_A1);
	delay_ms2(DELAY_FEEDBACK);
	TIM2->CR1 &= ~TIM_CR1_CEN;


	//STEP 02 CONNECT REFERENCE
	// -let integrator to go to - voltages
	pinON(MUX_PIN_A0);
	pinOFF(MUX_PIN_A1);
	delay_ms2(DELAY_SETUP);
	TIM2->CR1 &= ~TIM_CR1_CEN;


	//STEP 03 CONNECT SOURCE/GROUND
	//-let integrator to go to - voltages

	//SELECT THE INPUT
	if (source == 0){
		pinOFF(MUX_PIN_A0);
		pinON(MUX_PIN_A1);
	}
	else if (source == 1){
		pinOFF(MUX_PIN_A0);
		pinOFF(MUX_PIN_A1);
	}
	//wait until it cross zero - becomes 1
	while (!(GPIOA->IDR & (0x1 << INPUT_PIN)));
	delay_ms2(DELAY_T1);


	//STEP 04 CONNECT REFERENCE
	// -let integrator to go from negative to positive and measure time
	pinON(MUX_PIN_A0);
	pinOFF(MUX_PIN_A1);


	TIM2->CNT = 0u;
	while (GPIOA->IDR & (0x1 << INPUT_PIN));
	uint32_t tx_us = TIM2->CNT;
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

	}while((vxReal = myAtoi(inputBuffer)) == -1);

	unsigned int formerKM = KM;
	unsigned int vx = calculateVx(measureVoltage(1));

	KM = (KM * vxReal)/vx;
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
	}while((samples = myAtoi(inputBuffer)) == -1);

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
	for (int i = 0; i < 0; i++){
		pinON(LED_PIN_INNER);
		delay_ms(1000);
		pinOFF(LED_PIN_INNER);
		delay_ms(1000);
	}
	printf("-------------------WELCOME TO VOLTMETER INTERFACE-------------------\r\n");
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
	while ((RCC->CR & RCC_CR_HSERDY) == 0)
		;

	/* Switch the main clock to the HSE clock source. */
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;

	/* Wait until the clock source is switched to HSE. */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE)
		;

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
	GPIOA-> MODER |= (0x1 << LED_PIN_INNER * 2);
	GPIOA-> MODER |= (0x1 << MUX_PIN_A0 * 2);
	GPIOA-> MODER |= (0x1 << MUX_PIN_A1 * 2);

	GPIOA-> PUPDR |= (0x2 << INPUT_PIN *2);

	//Define timer pins
	//GPIOA->MODER |= (2 << GPIO_MODER_MODER5_Pos);
	/* Select the alternate function to AF1 -> TIM2_CH1 */
	//GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFRL5_Pos);


	//Define USART pins
	/* Set the corresponding GPIO pins to alternate function mode */
	GPIOA->MODER |= (2 << GPIO_MODER_MODER2_Pos) | (2 << GPIO_MODER_MODER3_Pos);
	/* Select the alternate function to AF7 */
	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFRL2_Pos) | (7 << GPIO_AFRL_AFRL3_Pos);


}

void time2_init(void)
{
	/* Enable the clock for the timer. Note that the timer is located at APB1
	 * domain, but there is an internal multiplier x2 which is active when the
	 * APB1 prescaler is higher than x1. Thus the TIM2 clock is 72MHz in our
	 * case */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure the channel 1 into PWM mode 1 */

	//TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
	/* Enable the channel 1 output */
	//TIM2->CCER |= TIM_CCER_CC1E;


	/* Set the timer auto-reload value to 10000 -> the timer period will be 1Hz */
	//TIM2->ARR = (uint32_t)0xFFFFFFFFU;

	/* Set the channel 1 capture/compare to 5000 -> dutycycle is 0.5 */
	//TIM2->CCR1 = TIMER_CCR1;
	TIM2->PSC = 7;
	TIM2->ARR = 65535;
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->SR &= ~TIM_SR_UIF; //clear uif flag




}

/**
 * Procedure for initializing the USART2.
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
