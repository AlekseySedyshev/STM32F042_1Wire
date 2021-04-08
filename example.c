#include "stm32f0xx.h"        // Device header
#include "onewire.h" 

extern uint8_t OW_SEND_REQ[];
extern uint8_t OW_GET_DATA[];
extern uint8_t ow_rxdata[12];
extern OW_str dsstr;

extern uint8_t  OW_SEND_REQ[];
extern uint8_t  OW_GET_DATA[];	

extern uint8_t DS_Rx_Flag,DS_Tx_Flag;

float temp_out;
float vbat_out;

uint32_t vbat_adc=0;

uint8_t flag_start=0;
uint8_t flag_measure=0;
uint8_t flag_pause=0;
uint16_t flag_counter=0;

uint8_t i;
uint8_t word_flag=0,adc_flag=0;

uint16_t result,res;

//---------------Timer settings-------------
uint16_t TimingDelay,led_count,ms1000;
uint8_t ms200=0,msec200=0,sec_tic=0;


void SysTick_Handler(void)						{
		if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk ) {
			if (TimingDelay			!=0x00) TimingDelay--;
			if (!led_count) {led_count=500; GPIOB->ODR ^=1;}
			if (DS_Tx_Flag) {flag_counter++;}
			if (flag_counter == 750) {flag_measure=1;flag_counter=0;DS_Tx_Flag=0;}
								
			if (!ms1000) {ms1000=1000;sec_tic=1;}
			led_count--;ms1000--;
  		SysTick->CTRL &=(~SysTick_CTRL_COUNTFLAG_Msk); 		
		}
}
 
void delay_ms (uint16_t DelTime) 			{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}


void initial (void)										{// Initial 
	//--------------SysTick------------------
SysTick->LOAD =(8000-1);  //HSI 8 MHz - 1 msek
SysTick->VAL =(8000-1);   //HSI 8 MHz - 1 msek	
SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;	
NVIC_EnableIRQ(SysTick_IRQn);
__enable_irq ();		
//-------------------GPIOB-Blinking Led		
RCC->AHBENR  	|= RCC_AHBENR_GPIOBEN; 																					//
GPIOB->MODER 	|= GPIO_MODER_MODER0_0;																					//Pb0-Out 


int main(void) 												{	//main
//------------------Initial parameters-----------------------------------
initial();
OW_Init();

//------------------Main Loop--------------------------------------------
while (1)  												{// Main loop
	
if (sec_tic) 			{ OW_SendReq(&dsstr); sec_tic=0; }	//Every Sec operation

if (flag_measure) { OW_GetTemp(&dsstr);	flag_measure=0;}
	
if (DS_Rx_Flag)	 	{ DS_Rx_Flag=0; OW_restByte(ow_rxdata, &dsstr);	
										temp_out = OW_temp(ow_rxdata);
									}





} // end - main loop 
} // end - Main  
	
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
