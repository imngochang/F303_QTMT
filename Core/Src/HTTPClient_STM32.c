#include "HTTPClient_STM32.h"

uint8_t HTTP_DataToGet[MAX_RECVBUF_LEN] = {0};
uint8_t FTP_DataToSend[MAX_RECVBUF_LEN/2] = "";


/**
  * @brief  Configure contextID and responseheader parameters for HTTP
  * @param  None.
  * @retval RET_FAIL if configuration failed.
  * 		RET_OK if configuration successful.
  */
STATUS HTTP_configParams(void)
{
	Sim_send("AT+CGDCONT=cid,\"ip\",\"APN\"\r",strlen("AT+CGDCONT=cid,\"ip\",\"APN\"\r"));
	ret = Sim_checkResponseWith("OK", 2, 5000);
	__NOP();
	if(ret == RET_OK)
	{
		Sim_send("AT+CGACT=1,cid\r",strlen("AT+CGACT=1,cid\r"));
		ret = Sim_checkResponseWith("OK", 2, 5000);
		__NOP();
		if(ret == RET_OK)
		{
			Sim_send("AT+HTTPINIT\r",strlen("AT+HTTPINIT\r"));
			ret = Sim_checkResponseWith("OK", 2, 5000);
			__NOP();
			if(ret == RET_FAIL)
			{
				Sim_send("AT+HTTPTERM\r",strlen("AT+HTTPTERM\r"));
				ret = Sim_checkResponseWith("OK", 2, 5000);
				__NOP();
				if(ret == RET_OK)
				{
					Sim_send("AT+HTTPINIT\r",strlen("AT+HTTPINIT\r"));
					ret = Sim_checkResponseWith("OK", 2, 5000);
					__NOP();
				}
			}
		}
	}
	return ret;
}

/**
  * @brief  Send HTTP request using GET method.
  * @param  url is URL to access.
  * @retval RET_OK if Server responds to code 200.
  * 		RET_FAIL if otherwise.
  */
STATUS HTTP_sendGETRequest(char* url)
{
	char cmd[100] = {0};
	sprintf(cmd,"AT+HTTPPARA=\"URL\",\"%s\"\r",url);
	Sim_send(cmd,strlen(cmd));
	ret = Sim_checkResponseWith("OK", 2, 5000);
	__NOP();
	if(ret == RET_OK)
	{
		Sim_send("AT+HTTPACTION=0\r",16);
		ret = Sim_checkResponseWith("+HTTPACTION: 0,200", 2, 30000);
		__NOP();
		if(ret == RET_OK)
		{
			Sim_send("AT+HTTPHEAD\r",12);
			ret = Sim_checkResponseWith("200 OK", 3, 60000);
			__NOP();
			if(ret == RET_OK)
			{
				__NOP();
			}
		}
	}
	return ret;
}

/**
  * @brief  Read the HTTP response from the server after sending the request.
  * @param  datatoget is used to store response data from the server.
  * @retval RET_OK if read success.
  * 		RET_FAIL if read fail.
  */
STATUS HTTP_readGETResponse(char* datatoget)
{
	deleteBuffer(datatoget, strlen(datatoget));
	Sim_send("AT+HTTPHEAD\r",strlen("AT+HTTPHEAD\r"));
	ret = Sim_checkResponseWith("\r\nOK\r\n", 4, 60000);
	__NOP();
	if(ret == RET_OK)
	{
		char* ptr = NULL;
		ptr = strstr((char*)Sim_Rxdata,"HTTP/");
		if(ptr != NULL)
		{
			char* ptr1 = NULL;
			ptr1 = strstr((char*)Sim_Rxdata,"\r\nOK");
			uint16_t startpos = ptr - (char*)Sim_Rxdata;
			uint16_t endpos = ptr1 - (char*)Sim_Rxdata;
			subString((char*)Sim_Rxdata, datatoget, startpos, endpos);
		}
		else
		{
			ret = RET_FAIL;
		}
	}
	return ret;
}
