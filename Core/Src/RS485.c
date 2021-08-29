#include "RS485.h"

#define rs485_uart huart2
extern UART_HandleTypeDef rs485_uart;

void RS485_EnablePin(RS485_PinState status)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, status?GPIO_PIN_SET:GPIO_PIN_RESET);
}
uint16_t crcCalculation(uint8_t* rs485_data, int in_dx)
{
	int i,j;
	uint16_t crc=0xFFFF;
	uint16_t crctemp;
	for(i=0;i<in_dx;i++)
  {
		crc = rs485_data[i]^crc;
		for(j=1;j<9;j++)
		{
			crctemp = crc/2;
			if(crc-crctemp*2==1)
			{
				crc=crctemp ^ 0xA001;
			}
			else
			{
				crc=crctemp;
			}
		}
	}
	return crc;
//  crctemp1 = crc/256;
//  crctemp2 = crc*256;
//  crc= crctemp1 + crctemp2;
//	crc1 =crc>>8;
//	crc2=(uint8_t)crc;
}

void RS485_transmit(RS485Query_t p_RS485Query)
{
	uint8_t queryData[10];
	uint8_t ind=0;
	queryData[ind++] = p_RS485Query.slaveAddress;
	queryData[ind++] = p_RS485Query.mbFunction;
	if	((p_RS485Query.mbFunction == Read_CoilStatus) 		||(p_RS485Query.mbFunction == Read_InputStatus)
		|| (p_RS485Query.mbFunction == Read_HoldingRegister)||(p_RS485Query.mbFunction == Read_InputRegister)
		|| (p_RS485Query.mbFunction == Force_SingleCoil)		||(p_RS485Query.mbFunction == Preset_SingleRegister))
	{
		queryData[ind++] = (uint8_t)(p_RS485Query.regAddress>>8);
		queryData[ind++] = (uint8_t)(p_RS485Query.regAddress&0xFF);
		queryData[ind++] = (uint8_t)(p_RS485Query.regCount>>8);
		queryData[ind++] = (uint8_t)(p_RS485Query.regCount&0xFF);
	}
	else if((p_RS485Query.mbFunction == Read_ExceptionStatus) || (p_RS485Query.mbFunction == Fetch_CommEventCounter)
		||	(p_RS485Query.mbFunction == Fetch_CommEventLog))
	{
		
	}	
	p_RS485Query.crc = crcCalculation(queryData,ind);
	queryData[ind++] = (uint8_t)(p_RS485Query.crc&0xFF);
	queryData[ind++] = (uint8_t)(p_RS485Query.crc>>8);	
	RS485_EnablePin(TRANSMIT);
	HAL_UART_Transmit(&rs485_uart, (uint8_t*)queryData,ind,1000);
	RS485_EnablePin(RECEIVE);
	
}

void RS485_ForceSingleCoil(uint8_t SlaveAddr, uint16_t Coil, uint8_t State)
{
	uint8_t queryData[10];
	uint8_t ind=0;
	queryData[ind++] = SlaveAddr;
	queryData[ind++] = Force_SingleCoil;
	queryData[ind++] = (uint8_t)(Coil>>8);
	queryData[ind++] = (uint8_t)(Coil&0xFF);
	if(State == 1) queryData[ind++] = 0xFF;
	else queryData[ind++] = 0x00;
	queryData[ind++] = 0x00;
	RS485_EnablePin(TRANSMIT);
	HAL_UART_Transmit(&rs485_uart, (uint8_t*)queryData,ind,1000);
	RS485_EnablePin(RECEIVE);
	
}

uint8_t RS485_analysis(RS485Data_t* m_rs485data, RS485Query_t RS485_SentStruct, uint8_t* p_RS485Data, uint8_t length)
{
	if((length > 3)&&(p_RS485Data[0] == RS485_SentStruct.slaveAddress))
	{
		uint8_t lengthData = 0;
		uint16_t crc = 0;
		m_rs485data->Function = (RS485_Function)p_RS485Data[1];
		if(m_rs485data->Function == RS485_SentStruct.mbFunction)
		{
			m_rs485data->slaveAddress = p_RS485Data[0];
			if((p_RS485Data[1]&0x80) == 0x80)
			{
				//return wrong function code
				return 0;
			}
			else
			{
				switch (p_RS485Data[1])
				{
					case 0x01:	//Read Coil Status
						lengthData = p_RS485Data[2];
						m_rs485data->length = lengthData;
						crc =  (uint16_t)p_RS485Data[3+m_rs485data->length]|((uint16_t)p_RS485Data[4+m_rs485data->length]<<8); 
						if(crc == crcCalculation((uint8_t*)p_RS485Data,3+lengthData))
						{
							for(uint8_t i=0; i<m_rs485data->length;i++)
							{
								m_rs485data->CoilStatus[i] = p_RS485Data[3+i];
							}
							return 1;
						}
						else return 0;
					case 0x02: //Read Input Status
						lengthData = p_RS485Data[2];
						m_rs485data->length = lengthData;
						crc =  (uint16_t)p_RS485Data[3+m_rs485data->length]|((uint16_t)p_RS485Data[4+m_rs485data->length]<<8); 
						if(crc == crcCalculation((uint8_t*)p_RS485Data,3+lengthData))
						{
							for(uint8_t i=0; i<m_rs485data->length;i++)
							{
								m_rs485data->CoilStatus[i] = p_RS485Data[3+i];
							}
							return 1;
						}
						else return 0;
					case 0x03:	//Read Holding Register
						lengthData = p_RS485Data[2];
						m_rs485data->length = lengthData/2;
						crc =  (uint16_t)p_RS485Data[3+lengthData]|((uint16_t)p_RS485Data[4+lengthData]<<8); 
						if(crc == crcCalculation((uint8_t*)p_RS485Data,3+lengthData))
						{
							for(uint8_t i=0; i<m_rs485data->length;i++)
							{
								m_rs485data->dataReg[i] = (uint16_t)(p_RS485Data[3+i*2]<<8)|p_RS485Data[4+i*2];	//Vi tri bat dau du lieu se la 3
							}
							return 1;
						}
						else return 0;
					case 0x04:	//Read Input Register
						lengthData = p_RS485Data[2];
						m_rs485data->length = lengthData/2;
						crc =  (uint16_t)p_RS485Data[3+lengthData]|((uint16_t)p_RS485Data[4+lengthData]<<8); 
						if(crc == crcCalculation((uint8_t*)p_RS485Data,3+lengthData))
						{
							for(uint8_t i=0; i<m_rs485data->length;i++)
							{
								m_rs485data->dataReg[i] = (uint16_t)(p_RS485Data[3+i*2]<<8)|p_RS485Data[4+i*2];	//Vi tri bat dau du lieu se la 3
							}
							return 1;
						}
						else return 0;
					case 0x05:
						
						break;
					case 0x06:
						
						break;
					case 0x07:
						crc =  (uint16_t)p_RS485Data[3]|((uint16_t)p_RS485Data[4]<<8);
						if(crc == crcCalculation((uint8_t*)p_RS485Data,3+lengthData))
						{
							m_rs485data->CoilStatus[0] = p_RS485Data[2];
							return 1;
						}
						else return 0;
					case 0x0B:
						break;
					case 0x0C:
						break;
					case 0x0F:
						break;
					case 0x10:
						break;
					case 0x11:
						break;
					case 0x14:
						break;
					case 0x15:
						break;
					case 0x16:
						break;
					case 0x17:
						break;
					case 0x18:
						break;
				}
			}
		}
		else return 0;
	}
	return 0;
}



