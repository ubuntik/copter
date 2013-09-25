/**
******************************************************************************
* @file		src/main.c based on SysTick/main.c
* @author	asubbotina, alchul
* @version	V0.0.2
* @date		28-March-2013
* @brief	Main program body
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_usart.h"
#include <strings.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MLEN (24)
#define MDIF (2)
#define DPLEN (6)
#define D0LEN (6)
#define D1LEN (12)
#define DDIF (2)
#define MAX_STRLEN 16	/* Lenght of string for getting measured value from MAXONAR */
#define MAX_GAS 83
#define MIN_GAS 0
#define dt 0.1		/* 0.1 sec */

/* Private macro -------------------------------------------------------------*/
#define MCHECK(x) (abs(MLEN-(x))<=MDIF)
#define DPCHECK(x) (abs(DPLEN-(x))<=DDIF)
#define D0CHECK(x) (abs(D0LEN+(x))<=DDIF)
#define D1CHECK(x) (abs(D1LEN+(x))<=DDIF)

/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

static __IO int TimingDelay;
static __IO int AddDelay=0;

static __IO uint8_t current_bit;
static __IO uint8_t prev_bit;
static __IO uint32_t input_sig = 0;
static __IO uint8_t new_sig = 0;
static __IO uint32_t signal0 = 0;
static __IO int32_t distance1 = 0;
//static __IO double P = 0.1;
//static __IO double D = 0.2;
//static __IO double I = 0.083;
static __IO double P = 0.1;
static __IO double D = 0.2;
static __IO double I = 0.04;
static __IO req_height = 100;	/* centimeters */
volatile char rcv_str[MAX_STRLEN+1];

double W = 0;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);

/**
* @brief	emulation of starting signal
* @param	None
* @retval	None
*/
uint8_t synctau = 24;
void emul_start_sig()
{
	uint8_t i = 0;

//	STM_EVAL_LEDToggle(LED4);
	GPIO_ToggleBits(GPIOB, GPIO_Pin_1);
	Delay(30);

	for (i = 0; i < 3; i++) {
		Delay(synctau);
		GPIO_ToggleBits(GPIOB, GPIO_Pin_1);
//		STM_EVAL_LEDToggle(LED4);
	}
}

/**
* @brief	emulation of finishing signal
* @param	None
* @retval	None
*/
void emul_finish_sig()
{
	Delay(60);
}

/**
* @brief	emulation of 1 bit in signal
* @param	None
* @retval	None
*/
void emul_one()
{
	uint8_t i = 0;
	Delay(6);
//	STM_EVAL_LEDToggle(LED4);
	GPIO_ToggleBits(GPIOB, GPIO_Pin_1);

	Delay(12);
//	STM_EVAL_LEDToggle(LED4);
	GPIO_ToggleBits(GPIOB, GPIO_Pin_1);
}

/**
* @brief	emulation of 0 bit in signal
* @param	None
* @retval	None
*/
void emul_nill()
{
		Delay(6);
//		STM_EVAL_LEDToggle(LED4);
		GPIO_ToggleBits(GPIOB, GPIO_Pin_1);

		Delay(6);
//		STM_EVAL_LEDToggle(LED4);
		GPIO_ToggleBits(GPIOB, GPIO_Pin_1);
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

uint32_t calc_chksum(uint32_t signal)
{
    uint32_t sum = 0;
    uint32_t i;
    uint32_t signal0 = signal & 0xfffffff0;

    for(i = 0; i < 4; i++) {
        sum += (signal0 >> i * 8) & 0xff;
    }
    return ((sum & 0xf) + ((sum & 0xf0)>>4)) & 0xf ;
}

uint8_t get_chksum(uint32_t signal)
{
	uint8_t sum = (signal & 0x0000000f);
	return sum;
}

int is_true_chksum (uint32_t signal)
{
	uint8_t crnt_sum = calc_chksum(signal);
	uint8_t orig_sum = get_chksum(signal);
	return (crnt_sum == orig_sum) ? 1 : 0;
}

uint32_t set_chksum(uint8_t sum, uint32_t signal)
{
	uint32_t new_input_sig = 0;
	new_input_sig = (signal & 0xfffffff0);
	new_input_sig |= sum;
	return new_input_sig;
}

uint8_t get_gas(uint32_t signal)
{
	uint8_t gas_signal = (signal & 0x7f000000) >> 24;
	return gas_signal;
}

uint32_t set_gas(uint8_t gas, uint32_t signal)
{
	uint32_t new_input_sig = 0;
	new_input_sig = (signal & 0x80ffffff);
	new_input_sig |= (uint32_t)gas << 24;
	return new_input_sig;
}

double get_P(uint32_t signal)
{
	double P_signal = ((double)((signal & 0x00007f00) >> 8)) / 10;
	return P_signal;
}

double get_D(uint32_t signal)
{
	double D_inc = (signal & 0x80000000) >> 31;
	double D_dec = (signal & 0x00008000) >> 15;
	double D_value = D + D_inc / 10 - D_dec / 10;
	return D_value;
}

uint32_t set_light(uint8_t value, uint32_t signal)
{
	uint32_t new_input_sig = 0;
	new_input_sig = (signal & 0x7fffffff);
	new_input_sig |= (uint32_t)value << 31;
	return new_input_sig;
}

uint32_t set_mode(uint8_t value, uint32_t signal)
{
	uint32_t new_input_sig = 0;
	new_input_sig = (signal & 0xffff7fff);
	new_input_sig |= (uint32_t)value << 15;
	return new_input_sig;
}

uint32_t set_trimmer(uint8_t value, uint32_t signal)
{
	uint32_t new_input_sig = 0;
	new_input_sig = (signal & 0xffff80ff);
	new_input_sig |= (uint32_t)value << 8;
	return new_input_sig;
}

/* Private functions ---------------------------------------------------------*/

#if 0
void Init_LEDs()
{
	/* Initialize Leds mounted on STM32F4-Discovery board */
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);

	/* Turn on LED4 and LED5 */
	STM_EVAL_LEDOn(LED4);
	/* set in 0 on starting of signal */
	STM_EVAL_LEDOn(LED5);
}
#endif

void Init_GPIOA()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Configure PA0 (button) into input mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Init_GPIOB()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Configure PB2 into input mode */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PB5 into output mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PB1 into output mode (for MINI-M4) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// for MINI-M4
	GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	// Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	// this activates the pullup resistors on the IO pins
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
}

void Init_USART()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStructure.USART_BaudRate = 9600;
	// we want the data frame size to be 8 bits (standard)
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// we want 1 stop bit (standard)
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// we don't want a parity bit (standard)
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// we don't want flow control (standard)
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// we want to enable the transmitter and the receiver
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	// again all the properties are passed to the USART_Init function
	// which takes care of all the bit setting
	USART_Init(USART1, &USART_InitStructure);


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	// enable the USART1 receive interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	// the USART1 interrupts are globally enabled
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// the properties are passed to the NVIC_Init function
	// which takes care of the low level stuff
	NVIC_Init(&NVIC_InitStructure);

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);

	// just send a message to indicate that it works
	USART_puts(USART1, "Init complete! Hello World!\r\n");
}

__IO uint32_t ctime=0;
uint32_t signal = 0;
uint32_t signal00 = 0;
uint32_t mask;
uint8_t framecnt = 0;
uint8_t crnt_gas = 0;
double h_sum = 0;
double h_crnt = 0;
double h_prev = 0;
uint16_t dist_arr[2000];
long long time_arr[2000];
double W_arr[2000];
unsigned int iter = 0;
long long time = 0;

int main(void)
{
//	Init_LEDs();
	Init_GPIOA();
	Init_GPIOB();
	Init_USART();

	if (SysTick_Config(SystemCoreClock / 74000)) {
		/* Capture error */
		while (1);
	}

	/* encode signal */
	TimingDelay = 0;
	h_prev = (double)distance1 * 2.54;
	while(1) {
		while(ctime < 121 * 6);
		ctime = 0;
		TimingDelay = 0;
		if(new_sig) {
			new_sig=0;
			framecnt=16;
		}
		if(framecnt==0) continue;
		framecnt--;
		signal0 = input_sig;
		if (is_true_chksum(signal0)) {
			signal = signal0;
			signal00 = signal0;
		}
		h_crnt = (double)distance1 * 2.54;

		if (fabs(h_crnt - h_prev) > 20)
			h_crnt = h_prev;

		crnt_gas = get_gas(signal);

		req_height = (crnt_gas < 40) ? 0 : 100;
		h_sum += (req_height - h_crnt) * dt;

		if (crnt_gas == 0) {
			W = 0;
		} else {
			W = -P * (h_crnt - req_height) - D * (h_crnt - h_prev) / dt + I * h_sum;
// + crnt_gas * (125.0 / 85.0) + 10;
		}

		if (W > MAX_GAS)
			W = MAX_GAS;
		if (W < MIN_GAS)
			W = MIN_GAS;
		dist_arr[iter] = distance1;
		time_arr[iter] = time;
		W_arr[iter++] = W;
		if (iter >= 2000 )
			iter = 0;

		signal = set_gas((uint8_t)W, signal);
		signal = set_chksum(calc_chksum(signal), signal);

		h_prev = h_crnt;

		emul_start_sig();
		for (mask = 1 << 31; mask > 0; mask = mask >> 1) {
			(signal & mask) ? emul_one() : emul_nill();
		}
		emul_finish_sig();
	}
}

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t

		if((t != '\r') && (cnt < MAX_STRLEN) ){
			rcv_str[cnt] = t;
			cnt++;
		} else {
			if (cnt == 4) {
				distance1 = (int)(rcv_str[1] - '0') * 100 + (int)(rcv_str[2] - '0') * 10 + (int)(rcv_str[3] - '0');
			}
			cnt = 0;
		}
	}
}

/**
* @brief	Inserts a delay time.
* @param	nTime: specifies the delay time length, in milliseconds.
* @retval	None
*/
void Delay(__IO uint32_t nTime)
{
	AddDelay = nTime*10;
	//TimingDelay = nTime;
	while(AddDelay != 0);

	while(TimingDelay > 0);
}

/**
* @brief	Decrements the TimingDelay variable.
*		Decode signal
* @param	None
* @retval	None
*/
	static __IO int bit_25 = 0;
void TimingDelay_Decrement(void)
{
	static __IO short is_first_bit = 1;
	static __IO uint32_t buff;
	static __IO char sign;
	static __IO int count = 0;
	static __IO int scount = 0;		 // signed count
	static __IO int scount_prev = 0;
	static __IO int scount_prev_prev = 0;
	static __IO int fdata = 0;		// after marker, start data
	static __IO int dsign = 0;		// positive or negative value of signal
	static __IO int bit = 0;		// 1 or 0
	static __IO int bitok = 0;		// new bit accepted
	static __IO int endframe = 0;		// end data
	static __IO int bitn = 0;		// number of bit
	static __IO int i = 0;
	static __IO int subcounter = 10;

	time++;
	if (AddDelay != 0) {
		TimingDelay += AddDelay;
		AddDelay=0;
	}
	//if (TimingDelay != 0x00) {
		TimingDelay--;
	//}

//	bit_25 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2);
	bit_25 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
	GPIO_WriteBit(GPIOB, GPIO_Pin_5, (bit_25) ? (Bit_RESET) : (Bit_SET));

	subcounter--;
	if(subcounter>0) return;
	subcounter=10;

	ctime++;

	current_bit = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
//	GPIO_WriteBit(GPIOD, GPIO_Pin_12, (current_bit) ? (Bit_SET) : (Bit_RESET));

	if (is_first_bit) {
		prev_bit = current_bit;
		is_first_bit = 0;
		return;
	}

	if (prev_bit == current_bit) {
		count++;
		return;
	}
	scount_prev_prev = scount_prev;
	scount_prev = scount;
	scount = (prev_bit == 1) ? count: -count;
	if (MCHECK(-scount_prev_prev) && MCHECK(scount_prev) && MCHECK(-scount)) {
		fdata=1;
		dsign=1;
		bitn=0;
	} else if(fdata) {
		if(dsign) {
			if(DPCHECK(scount)) {
				dsign=0;
			} else {
				fdata=0;
				endframe=1;
			}
		} else {
			if(D0CHECK(scount)) {
				bit=0;
				bitok=1;
				dsign=1;
				bitn++;
			} else if (D1CHECK(scount)) {
				bit=1;
				bitok=1;
				dsign=1;
				bitn++;
			} else {
				fdata=0;
			}
		}
	}

	if(bitok){
		if(bitn == 1) buff = 0;
		if(bit) {
			buff = buff | (1 << (32 - bitn));
		}
		bitok = 0;
	}
	if(endframe) {
		endframe=0;
		if (bitn == 32) {
			input_sig = buff;
			new_sig = 1;
		}
	}
	prev_bit = current_bit;
	count = 0;
}

#ifdef	USE_FULL_ASSERT

/**
* @brief	Reports the name of the source file and the source line number
*				 where the assert_param error has occurred.
* @param	file: pointer to the source file name
* @param	line: assert_param error line source number
* @retval	None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* Infinite loop */
	while (1) {}
}
#endif

