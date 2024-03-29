/* --COPYRIGHT--,
 * Copyright (c)2020, TAPIT Co.,Ltd.
 * https://tapit.vn
 *
 **************************_TAPIT_EC15_EC21_STM32_************************
 *  Description:	Using UARTx to communicate with ModuleSIM EC15/EC21.
 *					USART2 in default,
 *					EC15/EC21 pin	- STM pin in default:
 *						TX_Pin 		- 	A3
 *  					RX_Pin 		-	A2
 *  Version:  		1.0
 *  Author: 		Hang Tran
 *  Release: 		May 8, 2020
 *  Built with STM32CubeIDE Version: 1.3.1
 *************************************************************************
 */
#define SIM_DEBUG 0

#define F4 4
#define F3 3
#define F1 1

/*Change your MCU line here*/
//#define _STM32CHIP_ F4
#define _STM32CHIP_ F3
//#define _STM32CHIP_ F1

#if _STM32CHIP_ == F4
	#include "stm32f4xx_hal.h"
#elif _STM32CHIP_ == F3
	#include "stm32f3xx_hal.h"
#else
	#include "stm32f1xx_hal.h"
#endif

#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

/*Change used UART here*/
//extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart3;
//extern UART_HandleTypeDef huart4;
//extern UART_HandleTypeDef huart5;
//extern UART_HandleTypeDef huart6;

typedef enum {
	RET_OK	 	= 0,
	RET_FAIL 	= 1,
	RET_TIMEOUT = 2,
} STATUS;

enum {
	ascii   	= 0,
	binary	 	= 1,
};

enum {
	active	 	= 0,
	passive   	= 1,
};

/*Change used UART here*/
#if (SIM_DEBUG == 1)
#define LOG_UART 	huart1
#endif
#define SIM_UART 	huart2
#define SIM_USART	USART2

#define FTP_MAX_DOWNLOADLEN		1000L
#define MAX_RECVBUF_LEN			1000L
#define MAX_SEND_TIME			(MAX_RECVBUF_LEN*10000) / 11520

#define APN			"UNINET"
#define APN_USER	""
#define APN_PASS	""

STATUS Sim_send(char *command, uint16_t len);
STATUS Sim_recv(uint32_t timeout);
STATUS Sim_checkResponseWith(char* userdata, uint8_t maxretry, uint32_t timeout);
STATUS Sim_checkRespIncludingNULL(char* userdata, uint8_t maxretry, uint32_t timeout);
STATUS Sim_checkOK(void);
uint8_t Sim_getSignalQuality(void);
STATUS Sim_queryCardStatus(void);
STATUS Sim_configInternet(char* apn, char* user, char* pass);
STATUS Sim_connectInternet(void);
STATUS Sim_disconnectInternet(void);
uint16_t Sim_getErrorCode(char* response, char* delim);
STATUS Sim_initSMS(void);

uint32_t strstrFromStart(char* userdata, uint32_t endpos);
uint32_t strstrFromEnd(char* userdata, uint32_t startpos);
void subString(char* maindata, char* subdata, uint32_t startpos, uint32_t endpos);
void deleteBuffer(char* buf, uint32_t len);
