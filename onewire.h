
#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#include "stm32f0xx.h" 
#include <stdint.h> 

#define CPU_FREQ 										8000000u

// first parameter OW_Send function

#define OW_SEND_RESET								1
#define OW_NO_RESET									2

// статус возврата функций
#define OW_OK												1
#define OW_ERROR										2
#define OW_NO_DEVICE								3

#define OW_NO_READ									0xff
#define OW_READ_SLOT								0xff


#define ONEWIRE_NOBODY 					0xF0u 	//Answer - nobody on Line

//==========ROM COMMAND================
#define OW_SEARCH_ROM						0xF0u 	//CMD Search ROM
#define OW_READ_ROM 						0x33u  //CMD Read Rom
#define OW_MATCH_ROM 						0x55u	//CMD Match Rom (CMD providing master communicate with chosen ROM)
#define OW_SKIP_ROM 						0xCCu  //CMD Skip ROM
#define OW_ALARM_SEARCH					0xECu  //CMD ALARM_SEARCH

//==========FUNKTION COMMAND===========
#define OW_CONVERT_TEMPERATURE 	0x44u  //CMD Get temperature
#define OW_WRITE_SCRATCHPAD 		0x4Eu  //CMD Write Scratchpad (write data into ROM) 
#define OW_READ_SCRATCHPAD 			0xBEu  //CMD GET Scratchpad(read datafrom ROM)
#define OW_COPY_SCRATCHPAD 			0x48u	//CMD Copy Scratchpad from EEPROM into RAM
#define OW_RECALL_E2 						0xB8u	//CMD Recall Alarm Trigger (Th and Tl from EEPROM)
#define OW_RECALL_POWER					0xB4u	//CMD Read Power Supply


#define DS18B20 								0x28u  //DS18B20 series code 
#define DS18S20 								0x10u  //DS18S20 series code

typedef struct {
uint8_t bit_buf[90];
uint8_t bqnt;
} OW_str;




void USART2_IRQHandler(void);
void DMA1_Channel4_5_IRQHandler(void);

uint8_t OW_Init(void);
uint8_t owCRC(uint8_t *mas,uint8_t start, uint8_t Len);
void 		OW_toBitsBuf(uint8_t q, const char *ow_byte, OW_str *ows);
void 		OW_restByte(uint8_t *ow_buf, OW_str *ows); 
uint8_t OW_SendReq(OW_str *owstr);
uint8_t OW_GetTemp(OW_str *owstr);

float OW_temp(uint8_t *buff );

#endif /* ONEWIRE_H_ */
