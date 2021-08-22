/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HTTPClient_STM32.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* User define */
#define STATION 			30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*=========SIM===========*/
extern uint8_t Sim_Rxbyte[1];
uint8_t Sim_RSSI = 0;
bool Sim_SleepMode = false;
volatile bool Sim_isRISignal = false;
/*=========SERVER===========*/
extern uint8_t HTTP_DataToGet[MAX_RECVBUF_LEN];
char HTTPServer_URL[500] = {0};
uint8_t configStatus = 0;
char URL1[30]      = "update.php";
char Host1[30]     = "kttvttb.tapit.vn";
char URL2[30]      = "update.php";
char Host2[30]     = "kttvttb.tapit.vn";
/*=========RTC===========*/
RTC_TimeTypeDef currentTime = {0};
RTC_DateTypeDef currentDate = {0};
RTC_AlarmTypeDef userAlarm = {0};
volatile bool isRTCStartCounting = false;
uint8_t RTC_ALARM = 5;
char* Months[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
struct timestamp
{
	char timezone[50];
	uint8_t date;
	uint8_t mon;
	uint16_t year;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
} TimeStamp;

volatile uint16_t rtc_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
STATUS Sim_connectHTTP(char* host, char* url, char* sentdata, char* recvdata);
void Sim_extractTimeStamp(char* data);
void Sim_getHostnUrl(char* data);
void Sim_enterSLEEPMode(void);
void Sim_exitSLEEPMode(void);
void Sim_resetSIM(void);
void Sim_restartControl(void);
void Sim_sendMess(char* phonenumber, char* content);
bool RTC_initAlarm(uint8_t hours, uint8_t minutes, uint8_t seconds);
void RTC_saveDateTime(void);
void RTC_readDateTime(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SIM_RI_Pin)
	{
		Sim_isRISignal = true;
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	//Thuc hien lay thoi gian hien tai cua RTC
	HAL_RTC_GetTime(hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &currentDate, RTC_FORMAT_BIN);
	//Thuc hien cai dat thoi gian Alarm
	RTC_initAlarm(currentTime.Hours, currentTime.Minutes, 0);
	rtc_count++;
	if((userAlarm.AlarmTime.Hours == 0) && (userAlarm.AlarmTime.Minutes == 0))
	{
		RTC_saveDateTime();
		isRTCStartCounting = true;
	}
	HAL_RTC_SetAlarm_IT(hrtc, &userAlarm, RTC_FORMAT_BIN);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  	HAL_Delay(1000);
  	RTC_readDateTime();
    HAL_UART_Receive_IT(&SIM_UART, Sim_Rxbyte, 1);
    Sim_resetSIM();
    /* First Connection */
	char StationStart[25]  = {0};
	char StationConfig[25] = {0};
	sprintf(StationStart,"station=%d&status=%s",STATION,"START");
	sprintf(StationConfig,"station=%d&status=%s",STATION,"CONFIG");
	Sim_exitSLEEPMode();
	Sim_checkOK();
	HAL_Delay(3000);
    if(Sim_checkOK() == RET_OK)
    {
		Sim_RSSI = Sim_getSignalQuality();
		if(Sim_RSSI != 99) //99 <=> error
		{
			if(Sim_initSMS() == RET_OK)
			{
				if(HTTP_configParams() == RET_OK)
				{
					__NOP();
					Sim_connectHTTP(Host1, "status.php", StationStart, (char*)HTTP_DataToGet);
				}
			}
		}
    }
    __NOP();
    Sim_enterSLEEPMode();
    Sim_isRISignal = false; //After reset, RISignal will raise
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(Sim_isRISignal == true)
	  {
		  Sim_restartControl();
		  Sim_isRISignal = false;
	  }

	  if(configStatus == 1)
	  {
//	  	Sim_connectHTTP(StationStart, HostConfig, URLConfig, PortConfig);
	  	configStatus = 0;
	  }

	  if(isRTCStartCounting)
	  {
		  HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
		  if((currentTime.Hours == 0)&&(currentTime.Minutes > 4))
		  {
			  NVIC_SystemReset();
		  }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F5)
  {
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 9;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 10;
  sDate.Year = 21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 9;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 10;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  //Khoi tao cac gia tri ban dau cho cac bien
  userAlarm = sAlarm;
  currentTime = sTime;
  currentDate = sDate;
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F5);
  }
  else
  {
	  HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
	  /**Enable the Alarm A */
	  userAlarm.AlarmTime.Hours = currentTime.Hours;
	  userAlarm.AlarmTime.Minutes = currentTime.Minutes;
	  userAlarm.AlarmTime.Seconds = 0;
	  userAlarm.AlarmTime.SubSeconds = 0;
	  userAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  userAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  userAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	  userAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	  userAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	  userAlarm.AlarmDateWeekDay = 1;
	  userAlarm.Alarm = RTC_ALARM_A;
	  RTC_initAlarm(userAlarm.AlarmTime.Hours,userAlarm.AlarmTime.Minutes,0);
  }
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SIM_RESET_Pin|SIM_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SIM_RESET_Pin SIM_PWR_Pin */
  GPIO_InitStruct.Pin = SIM_RESET_Pin|SIM_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SIM_DTR_Pin */
  GPIO_InitStruct.Pin = SIM_DTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SIM_DTR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SIM_RI_Pin */
  GPIO_InitStruct.Pin = SIM_RI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SIM_RI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_EN_Pin */
  GPIO_InitStruct.Pin = RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
STATUS Sim_connectHTTP(char* host, char* url, char* sentdata, char* recvdata)
{
	Sim_exitSLEEPMode();
	deleteBuffer(HTTPServer_URL, strlen(HTTPServer_URL));
	sprintf(HTTPServer_URL,"http://%s/%s?%s",host,url,sentdata);
	STATUS m_ret = RET_FAIL;
	for(uint8_t i = 0; i < 3; i++)
	{
		if(HTTP_sendGETRequest(HTTPServer_URL) == RET_OK)
		{
			__NOP();
			if(HTTP_readGETResponse(recvdata) == RET_OK)
			{
				__NOP();
				if(strstr(recvdata,"200 OK") != NULL)
				{
					Sim_extractTimeStamp(recvdata);
					if(strstr(recvdata,"||CC") != NULL)
					{
						Sim_getHostnUrl(recvdata);
						m_ret = RET_OK;
					}
					else if(strstr(recvdata,"||00") != NULL)
					{
						configStatus = 1;
						m_ret = RET_OK;
					}
					else if(strstr(recvdata,"||11") != NULL)
					{
						char* tok = NULL;
						tok = strtok(recvdata,"|");
						tok = strtok(NULL,"<");
						int8_t rtc_alarm = atoi(tok);
						if((rtc_alarm >= 0) && (rtc_alarm <= 60))
						{
							if((rtc_alarm % 5) == 0)
							{
								if(rtc_alarm != RTC_ALARM)
								{
									RTC_ALARM = rtc_alarm;
//									Flash_WriteIntType(MODE_RTC_ADDR, RTC_ALARM);
//									HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//									RTC_initAlarm(sTime.Hours, sTime.Minutes, 0);
								}
							}
						}
						m_ret = RET_OK;
					}
				}
			}
		}
		Sim_disconnectInternet();
		if(m_ret == RET_OK)
		{
			break;
		}
		else
		{
			HAL_Delay(2000);
		}
	}
	return m_ret;
}

void Sim_extractTimeStamp(char* data)
{
	int i = 0;
	char* ptr1 = strstr(data,"Date:");
	char* ptr2 = strstr(data," GMT");
	if((ptr1 != NULL) & (ptr2 != NULL))
	{
		while(ptr1 != ptr2)
		{
			TimeStamp.timezone[i++] = *ptr1;
			ptr1++;
		}
		char* tok = strtok(TimeStamp.timezone," ");
		tok = strtok(NULL," ");
		tok = strtok(NULL," ");
		TimeStamp.date = atoi(tok);
		tok = strtok(NULL," ");
		for(int i = 0; i < 12; i++)
		{
			if(strstr(tok,Months[i]) != NULL)
			{
				TimeStamp.mon = i + 1;
				break;
			}
		}
		tok = strtok(NULL," ");
		TimeStamp.year = atoi(tok);
		tok = strtok(NULL,":");
		TimeStamp.hour = atoi(tok);
		tok = strtok(NULL,":");
		TimeStamp.min = atoi(tok);
		tok = strtok(NULL,":");
		TimeStamp.sec = atoi(tok);
//		//update RTC Time
//		RTC_UserTimeUpdate();
	}
}

void Sim_getHostnUrl(char* data)
{
	memset(Host1,0,strlen(Host1));
	memset(Host2,0,strlen(Host2));
	memset(URL1,0,strlen(URL1));
	memset(URL2,0,strlen(URL2));
	char* token;
	token = strtok(data,"|");
	token = strtok(NULL,":");
	strcpy(Host1,token);
	token = strtok(NULL,"/");
	token = strtok(NULL,"<");
	strcpy(URL1,token);
	token = strtok(NULL,"|");
	token = strtok(NULL,":");
	strcpy(Host2,token);
	token = strtok(NULL,"/");
	token = strtok(NULL,"<");
	strcpy(URL2,token);
}

void Sim_enterSLEEPMode(void)
{
	STATUS m_ret;
	for(uint8_t i = 0; i < 2; i++)
	{
		Sim_send("AT+CSCLK=1\r", strlen("AT+CSCLK=1\r"));
		m_ret = Sim_checkResponseWith("OK", 2, 5000);
		__NOP();
		if(m_ret == RET_OK)
		{
			//DTR high
			HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_SET);
			HAL_Delay(1000);
			break;
		}
	}
	if(Sim_checkOK() == RET_FAIL)
	{
		Sim_SleepMode = true;
	}
	else
	{
		Sim_SleepMode = false;
	}
}

void Sim_exitSLEEPMode(void)
{
	Sim_SleepMode = true; //Sim van con o che do sleep mode
	//DTR low
	HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	if(Sim_checkOK() == RET_OK)
	{
		Sim_SleepMode = false;
	}
}

void Sim_resetSIM(void)
{
	//DTR high
	HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	//DTR low
	HAL_GPIO_WritePin(SIM_DTR_GPIO_Port, SIM_DTR_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	if(Sim_checkOK() == RET_OK) //SIM response
	{
		/*turn off*/
		//pull down PWRKEY
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_SET);
		//wait at least 2.5s
		HAL_Delay(3000);
		//release PWRKEY
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_RESET);
		HAL_Delay(5000);
		HAL_Delay(5000);
		/* turn on power*/
		//pull down PWRKEY
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_SET);
		//wait at least 0.5s
		HAL_Delay(3000);
		//release PWRKEY
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_RESET);
		HAL_Delay(5000);
		HAL_Delay(5000);
		HAL_Delay(5000);
		HAL_Delay(5000);
	}
	else //SIM not response
	{
		/* turn on or off power*/
		//pull down PWRKEY
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_SET);
		//wait at least 2.5s
		HAL_Delay(3000);
		//release PWRKEY
		HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_RESET);
		HAL_Delay(5000);
		HAL_Delay(5000);
		HAL_Delay(5000);
		HAL_Delay(5000);
		Sim_exitSLEEPMode();
		Sim_checkOK();
		HAL_Delay(1000);
		if(Sim_checkOK() == RET_OK) //SIM ON
		{
			__NOP();
		}
		else //SIM OFF
		{
			/* turn on power*/
			//pull down PWRKEY
			HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_SET);
			//wait at least 2.5s
			HAL_Delay(3000);
			//release PWRKEY
			HAL_GPIO_WritePin(SIM_PWR_GPIO_Port, SIM_PWR_Pin, GPIO_PIN_RESET);
			HAL_Delay(5000);
			HAL_Delay(5000);
			HAL_Delay(5000);
			HAL_Delay(5000);
		}
	}
}

void Sim_restartControl(void)
{
	bool isResetSystem = false;
	char Sim_PhoneNumb[15] = {0};
	STATUS m_ret;
	m_ret = Sim_checkResponseWith("+CMTI:", 3, 5000);
	__NOP();
	if(m_ret == RET_OK)
	{
		char Sim_Cmd[100] = {0};
		Sim_exitSLEEPMode();
		for(uint8_t i = 1; i < 11; i++)
		{
			sprintf(Sim_Cmd,"AT+CMGR=%d\r",i);
			//Doc noi dung tin nhan
			Sim_send(Sim_Cmd, strlen(Sim_Cmd));
			m_ret = Sim_checkResponseWith("RESET", 3, 3000);
			char* tok = NULL;
			tok = strtok((char*)Sim_Rxdata,"\"");
			if(tok != NULL)
			{
				while(tok != NULL)
				{
					tok = strtok(NULL,"\"");
					if(strstr(tok,"+84") != NULL)
					{
						memset(Sim_PhoneNumb,0,strlen(Sim_PhoneNumb));
						strcpy(Sim_PhoneNumb,tok);
					}
					else if(strstr(tok,"RESET") != NULL)
					{
						Sim_sendMess(Sim_PhoneNumb, "Reset he thong thanh cong");
						isResetSystem = true;
						break; //break while
					}
				}
				if(isResetSystem == true)
				{
					break; //break for
				}
			}
		}
		Sim_send("AT+CMGD=1,4\r",strlen("AT+CMGD=1,4\r"));
		m_ret = Sim_checkResponseWith("OK", 2, 5000);
		__NOP();
		if(isResetSystem == true)
		{
			NVIC_SystemReset();
		}
	}
}

void Sim_sendMess(char* phonenumber, char* content)
{
	char Sim_Cmd1[40] = {0};
	sprintf(Sim_Cmd1,"AT+CMGS=\"%s\"\r",phonenumber);
	char Sim_Cmd2[40] = {0};
	sprintf(Sim_Cmd2,"%s\x1A",content);
	Sim_send(Sim_Cmd1, strlen(Sim_Cmd1));
	if(Sim_checkResponseWith(">",2,5000) == RET_OK)
	{
		Sim_send(Sim_Cmd2, strlen(Sim_Cmd2));
	}
}

bool RTC_initAlarm(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	minutes = (minutes/RTC_ALARM)*RTC_ALARM + RTC_ALARM;
	if(minutes > 59)
	{
		if(hours+1 > 23)
		{
			hours = (hours+1)%24;
		}
		else hours = hours+1;
	}
	else
	{
		hours = hours;
	}
	minutes = minutes%60;
	seconds = seconds%60;
	userAlarm.AlarmTime.Hours = hours;
	userAlarm.AlarmTime.Minutes = minutes;
	userAlarm.AlarmTime.Seconds = 0;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &userAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
		return false;
	}
	return true;
}

void RTC_saveDateTime(void)
{
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
	uint32_t dateToStore;
	memcpy(&dateToStore, &currentDate, 4);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2,(dateToStore >> 16));
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3,(dateToStore & 0xffff));
}

void RTC_readDateTime(void)
{
	uint32_t dateToRead;
	RTC_DateTypeDef tempDate = {0};
	dateToRead  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
	dateToRead |= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2) << 16;
	memcpy(&tempDate, &dateToRead, sizeof(uint32_t));
	HAL_RTC_SetDate(&hrtc, &tempDate, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/