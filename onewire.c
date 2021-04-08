#include "onewire.h"

#define OW_USART 				USART2
#define OW_DMA_CH_RX 		DMA1_Channel5
#define OW_DMA_CH_TX 		DMA1_Channel4
#define OW_DMA_FLAG			(DMA1->ISR & DMA_ISR_TCIF4)
#define RESET_DMA_FLAG 	DMA1->IFCR |=DMA_IFCR_CTCIF4


#define OW_0		0x00
#define OW_1		0xff
#define OW_R_1	0xff

uint8_t DS_Rx_Flag=0,DS_Tx_Flag=0;

//uint8_t ow_buf[8];		// 1-wire buffer
uint8_t ow_rxdata[12];
OW_str dsstr;

static const char  OW_SEND_REQ[]	= {OW_SKIP_ROM,OW_CONVERT_TEMPERATURE};
static const char  OW_GET_DATA[]	= {OW_SKIP_ROM,OW_READ_SCRATCHPAD,
													OW_READ_SLOT,OW_READ_SLOT,OW_READ_SLOT,OW_READ_SLOT,	//	8 bytes ScratchData
													OW_READ_SLOT,OW_READ_SLOT,OW_READ_SLOT,OW_READ_SLOT,	//
													OW_READ_SLOT};																				// 	byte 9 - CRC	


void OW_toBitsBuf(uint8_t q, const char *ow_byte, OW_str *ows) 			{	// converter byte into  8 bytes (bits) for usart transfer ow_byte - byte for convert, ow_bits - buffer pointer, (8 byte buffer)
	uint8_t i,j;
	uint8_t temp;
	ows->bqnt=q*8;
for(j=0;j <q;j++) {	
	temp=ow_byte[j];
	for (i = 0; i < 8; i++) {
		if (temp & 0x01) {ows->bit_buf[j*8+i] = OW_1;} 
			else {ows->bit_buf[j*8+i] = OW_0;}
			temp = temp >> 1;
	}
}
}


void OW_restByte(uint8_t *ow_buf, OW_str *ows) 											{	// Function - restore bits buffer to byte buffer
	uint8_t j, i;
	
for (j=0;j<ows->bqnt;j++) {
	for (i = 0; i < 8; i++) {	
		ow_buf[j] = ow_buf[j] >> 1;
		if (ows->bit_buf[j*8+i] == OW_R_1) {ow_buf[j] |= 0x80;}

	}
	}
}

uint8_t OW_Init(void) 																							{ //Init Usart and DMA
	
	RCC->AHBENR 		|=	RCC_AHBENR_GPIOAEN;
	GPIOA->MODER 		|= 	GPIO_MODER_MODER2_1; 								//PA2 - BiDirect TX_RX
	GPIOA->OTYPER 	|=	GPIO_OTYPER_OT_2;										//Open drain
			
	GPIOA->AFR[0]	|=	1<<GPIO_AFRL_AFRL2_Pos;							//AFR 1 for PA2
	RCC->APB1ENR |=RCC_APB1ENR_USART2EN;

	OW_USART->CR1 &= 	(~USART_CR1_UE);
	OW_USART->BRR	 = 	CPU_FREQ/115200;
	OW_USART->CR3 |=	USART_CR3_HDSEL;	//Half Duplex Mode
	OW_USART->CR1 |= 	USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

	NVIC_EnableIRQ (USART2_IRQn); 
	NVIC_SetPriority(USART2_IRQn,0x01);
	
	RCC->AHBENR |=RCC_AHBENR_DMA1EN;
	return OW_OK;
}													
													
void USART2_IRQHandler(void) 																				{	//USART2 IRQ HANDLER
	if (USART2->ISR & USART_ISR_RXNE)	{}// Got Data in RX - Buffer 
	
	if (USART1->ISR & USART_ISR_IDLE)											{// TimeOut - message receidved
		
		DMA1_Channel4->CCR 	&= (~DMA_CCR_EN);	// TX Channel Off
		DMA1_Channel5->CCR 	&= (~DMA_CCR_EN);	// RX Channel Off
		USART2->CR1 &= (~USART_CR1_IDLEIE );
		//USART1->ICR |=USART_ICR_RTOCF;				// Clear Flag
		DS_Rx_Flag=1;
		
}
	
	if (USART1->ISR & USART_ISR_ORE)		{
			USART1->ICR |=USART_ICR_ORECF;		// Reset OverRun
	}	
	
	
}													
													
													
void DMA1_Channel4_5_IRQHandler(void) 															{	//DMA1_Channel4_5  IRQ HANDLER

	if (DMA1->ISR & DMA_ISR_TCIF4) 						{	// Tx Transfer complete
		DMA1_Channel4->CCR 	&= (~DMA_CCR_EN);		//DISABLE DMA TX Channel
		DMA1->IFCR |=DMA_IFCR_CTCIF4;						//Clear TC flag
		DS_Tx_Flag=1;	
	}
	
}
uint8_t owCRC(uint8_t *mas, uint8_t start, uint8_t Len) 						{	// CRC - function, Count CRC8 - *mas - array pointer, Lenght - Len 
  uint8_t i, dat, crc, fb;
  crc = 0; Len=Len+start;
  do {
    dat = mas[start];
    for (i = 0; i < 8; i++) { 
      fb = crc ^ dat;
      fb &= 1;
      crc >>= 1;
      dat >>= 1;
      if (fb == 1) crc ^= 0x8c; // Polinom
    }
    start++;
  } while (start < Len ); // Byte counter in array
  return crc;
}

uint8_t OW_Reset(void) 																							{	//RESET function (check online DS18B20)

	uint8_t ow_presence;
	
	OW_USART->CR1 &= (~USART_CR1_UE);
	OW_USART->BRR = CPU_FREQ/9600;
	OW_USART->CR1 |= 	 USART_CR1_UE ;

	OW_USART->ICR |=USART_ICR_TCCF; 
	OW_USART->TDR = OW_SEARCH_ROM;
	while ((OW_USART->ISR & USART_ISR_TC) == 0) {	}

	ow_presence = OW_USART->RDR;

	OW_USART->CR1 	&= 	(~USART_CR1_UE); 
	OW_USART->BRR  	 = 	CPU_FREQ/115200;
	OW_USART->CR1 	|= 	 USART_CR1_UE;
	
	if (ow_presence != ONEWIRE_NOBODY) {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}



uint8_t OW_SendReq(OW_str *owstr) 																	{	// Send request

	if (OW_Reset() == OW_NO_DEVICE) {	return OW_NO_DEVICE;}
	OW_toBitsBuf(2,OW_SEND_REQ, owstr);
		
		OW_USART->RQR |= USART_RQR_TXFRQ;		// Flush Tx reg
		OW_USART->ICR |=USART_ICR_TCCF;			// Clear USART TC Flag
		DMA1->IFCR |=DMA_IFCR_CTCIF4;				// Clear DMA TC Flag
// Write channel
		OW_DMA_CH_TX->CPAR =(uint32_t)&OW_USART->TDR; 						//TX-DMA for DS18B20
		OW_DMA_CH_TX->CMAR =(uint32_t)owstr->bit_buf;
		OW_DMA_CH_TX->CNDTR = owstr->bqnt;
		OW_DMA_CH_TX->CCR 	|= DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_DIR | DMA_CCR_TCIE;
		
		NVIC_EnableIRQ(DMA1_Channel4_5_IRQn); 
		NVIC_SetPriority(DMA1_Channel4_5_IRQn,2);
	
// Send start
		OW_USART->CR3 |=	USART_CR3_DMAT;
		OW_DMA_CH_TX->CCR 	|= DMA_CCR_EN;

	return OW_OK;
}

uint8_t OW_GetTemp(OW_str *owstr) 																	{	// Get temperature

	if (OW_Reset() == OW_NO_DEVICE) {	return OW_NO_DEVICE;}
	OW_toBitsBuf(11,OW_GET_DATA, owstr);
	
	//Flush USART RX/TX and clear TC flag
		OW_USART->RQR |=USART_RQR_RXFRQ | USART_RQR_TXFRQ;
	// Write channel
		OW_DMA_CH_TX->CPAR =(uint32_t)&OW_USART->TDR; 						//TX-DMA for DS18B20
		OW_DMA_CH_TX->CMAR =(uint32_t)owstr->bit_buf;
		OW_DMA_CH_TX->CNDTR = owstr->bqnt;
		OW_DMA_CH_TX->CCR 	|= DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_DIR ;
// Read	channel
		OW_DMA_CH_RX->CPAR =(uint32_t)&OW_USART->RDR; 						//RX-DMA for DS18B20
		OW_DMA_CH_RX->CMAR =(uint32_t)owstr->bit_buf;
		OW_DMA_CH_RX->CNDTR = owstr->bqnt;
		OW_DMA_CH_RX->CCR 	|= DMA_CCR_MINC | DMA_CCR_PL_0;
		
		OW_USART->ICR |=USART_ICR_IDLECF;
		OW_USART->CR1 |= 	USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_UE;
		OW_USART->CR3 |=	USART_CR3_DMAR | USART_CR3_DMAT;
		
		//NVIC_EnableIRQ (USART2_IRQn); 
		//NVIC_SetPriority(USART2_IRQn,0x01);
		
		OW_DMA_CH_RX->CCR 	|= DMA_CCR_EN;
		OW_DMA_CH_TX->CCR 	|= DMA_CCR_EN;

	return OW_OK;
}

float OW_temp(uint8_t *buff ) 																			{
	uint16_t tmp=0;
	float out;
	tmp=owCRC(buff,2, 8);
	if (buff[10]==tmp){
			tmp=0;
			tmp= ((buff[3] << 8) | buff[2]);
			if (buff[3] & 0x80) { tmp=~tmp; tmp++; out=(float)(tmp*(-1))/(float)16;}
				else {out= (float)tmp/(float)16;}
		}
return out;
}
