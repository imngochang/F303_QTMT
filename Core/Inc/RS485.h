#include "stdint.h"
#include "stdbool.h"
#include "stm32f3xx_hal.h"

typedef enum
{
	Read_CoilStatus 				= 0x01,
	Read_InputStatus 				= 0x02,
	Read_HoldingRegister 		=	0x03,
	Read_InputRegister 			= 0x04,
	Force_SingleCoil 				=	0x05,
	Preset_SingleRegister		= 0x06,
	Read_ExceptionStatus 		= 0x07,
	Fetch_CommEventCounter	= 0x0B,
	Fetch_CommEventLog			= 0x0C,
	Force_MultipleCoils			= 0x0F,
	Preset_MultipleRegisters= 0x10,
	Report_SlaveID					= 0x11,
	Read_GeneralReference		= 0x14,
	Write_GeneralReference	= 0x15,
	MaskWrite_4XRegister		= 0x16,
	ReadWrite_4XRegisters		= 0x17,
	Read_FIFOQueue					= 0x24

} RS485_Function;

typedef enum
{
  RECEIVE= 0u,
  TRANSMIT
} RS485_PinState;

// Query struct
typedef struct
{
	uint8_t slaveAddress;
	RS485_Function mbFunction;
	uint16_t regAddress;
	uint16_t regCount;
	uint16_t crc;
}RS485Query_t;

// Hold Register struct
typedef struct
{
	uint8_t length;
	uint8_t slaveAddress;
	RS485_Function Function;
	uint16_t dataReg[3];
	uint8_t CoilStatus[20];
}RS485Data_t;

/* 
	Select transmiting or receiving mode
	-	status: TRANSMIT -  Transmiting mode
				RECEIVE  -	Receiving mode
*/
void RS485_EnablePin(RS485_PinState status);

/* 
	Calculate CRC of Modbus, with polynomial value = 0xA001
	-	rs485_data  : the array contains the query or response
	-	in_dx		: the length of array ( not include the CRC )
*/
uint16_t crcCalculation(uint8_t* rs485_data, int in_dx);

/* 
	Transmit query 
	-	huart  			: the UART used to communicate Modbus
	-	p_RS485Query	: Query struct
*/
void RS485_transmit(RS485Query_t p_RS485Query);

/* 
	Compare and get value of Hold register
	-	RS485_SentStruct : the query struct 
	-	p_RS485Data		 : the array contain the response
	-	length			 : length of array
*/
uint8_t RS485_analysis(RS485Data_t* m_rs485data, RS485Query_t RS485_SentStruct, uint8_t* p_RS485Data, uint8_t length);

/* 
	Compare and get value of Hold register
	-	RS485_SentStruct : the query struct 
	-	p_RS485Data		 : the array contain the response
	-	length			 : length of array
*/
void RS485_ForceSingleCoil(uint8_t SlaveAddr, uint16_t Coil, uint8_t State);

