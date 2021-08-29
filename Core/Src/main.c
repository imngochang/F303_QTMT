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
#include "FLASH_STM32.h"
#include "RS485.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* User define */
#define STATION 			30
#define PWRMODE				0
#define MIN_DATA_ADDR 		0x08011400 //Page 69
#define MAX_DATA_ADDR 		0x0801F800 //Page 126
#define MAX_PAGE_ADDR		0x0801FBF0 //End of page 126
#define MIN_PAGE_ADDR		0x080117F0 //End of page 69
#define RD_ADDR				0x08011000 //Page 68
#define WR_ADDR				0x08010C00 //Page 67
#define PG_ADDR				0x08010800 //Page 66
#define MODE_RTC_ADDR 		0x08010400 //Page 65
/* ADC */
#define VOLPOWER 3.30
#define NSAMPLE 50
//#define CALCULATE_VIN(x)			(VOLREF*x*97)/(4095*22) 		//Vin
#define CALCULATE_VIN(x)			(VOLPOWER*x*57)/(4095*10)		//Vsolar
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc4;

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
char HostConfig[30]     = "kttvttb.tapit.vn";
char URLConfig[30]      = "status.php";
/*=========RTC===========*/
RTC_TimeTypeDef currentTime = {0};
RTC_DateTypeDef currentDate = {0};
RTC_AlarmTypeDef userAlarm = {0};
volatile bool isRTCStartCounting = false;
volatile bool isOnTimeToSendReq = false;
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
/*========FLASH==========*/
uint32_t WRITE_DATA_ADDR = MIN_DATA_ADDR;
uint32_t READ_DATA_ADDR = MIN_DATA_ADDR;
uint32_t PAGE_ADDR = MAX_PAGE_ADDR;
char Flash_DataToWrite[200] = {0};
char Flash_DataToRead[200] = {0};
char Flash_Host[50],Flash_URL[50],Flash_Data[200] = {0};
uint16_t Flash_WriteDataLen, Flash_ReadDataLen = 0;
bool Flash_isReadData = false;
/*=======SENSOR==========*/
char MainData[200] = {0};
float levelWater = 0;
char sensorStatus[6] = "OK";
volatile uint16_t rainRaw = 0;
float levelArray[6] = {0};
/*=========PWR===========*/
typedef enum
{
  OFF = 0u,
  ON
} Pwr_State;
/*=========ADC===========*/
uint16_t ADC_value[4] = {0};
float Voltage = 0;
uint8_t ADC_Count = 0;
uint16_t ADC_ArrayVal[NSAMPLE] = {0};
float ADC_Avr = 0;
float Sensor_Current = 0;
float Sensor_CalibCurrent = 0;
uint16_t Sensor_ADCval[4] = {0};
uint16_t Sensor_Value[10] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC4_Init(void);
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
void RTC_updateUserTime(void);
void RTC_updateUserDate(RTC_DateTypeDef* datetime);
bool RTC_isLeapYear(uint16_t y);
void Flash_init(void);
void Flash_writeData(const char* host, const char* url, const char* data);
void Flash_extractData(char* data);
float RS485_readUltrasonicSensor(uint8_t Modbus_Addr);
float RS485_readLiquidLevelTransmitter(uint8_t Modbus_Addr);
void PWR_ctrlPwrSensor(Pwr_State status);
void PWR_getPwrSupply(uint8_t channel);
void insertionSort(uint16_t arr[], int n) ;
void insertionSort_2(float arr[], int n);
float Sensor_getADCValue(uint8_t channel);
float ADC_map(float x, float in_min, float in_max, float out_min, float out_max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	for(uint32_t i = 0; i < 100000; i++);
	if(GPIO_Pin == SIM_RI_Pin)
	{
		Sim_isRISignal = true;
	}
	if(GPIO_Pin == IN_1_Pin)
	{
		rainRaw++;
		long j = 0;
		while((HAL_GPIO_ReadPin(IN_1_GPIO_Port,IN_1_Pin) == GPIO_PIN_SET) && (j < 1000000)){j++;};
	}
//	if(GPIO_Pin == IN_2_Pin)
//	{
//		while(HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin) == GPIO_PIN_SET);
//		IN2_cnt++;
//	}
//	if(GPIO_Pin == IN_3_Pin)
//	{
//		while(HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin) == GPIO_PIN_SET);
//		IN3_cnt++;
//	}
//	if(GPIO_Pin == IN_4_Pin)
//	{
//		while(HAL_GPIO_ReadPin(IN_4_GPIO_Port, IN_4_Pin) == GPIO_PIN_SET);
//		IN4_cnt++;
//	}
	for(uint32_t i = 0; i < 100000; i++);
	EXTI->PR |= GPIO_Pin;
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	isOnTimeToSendReq = true;
	//Thuc hien lay thoi gian hien tai cua RTC
	HAL_RTC_GetTime(hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &currentDate, RTC_FORMAT_BIN);
	rtc_count++;
	if((userAlarm.AlarmTime.Hours == 0) && (userAlarm.AlarmTime.Minutes == 0))
	{
		isRTCStartCounting = true;
	}
	//Thuc hien cai dat thoi gian Alarm
	RTC_initAlarm(currentTime.Hours, currentTime.Minutes, 0);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC2_Init();
  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */
	#if (PWRMODE == 1)
	PWR_ctrlPwrSensor(ON);
	#endif
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
  	HAL_Delay(1000);
  	Flash_init();
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
					Sim_connectHTTP(HostConfig, URLConfig, StationStart, (char*)HTTP_DataToGet);
				}
			}
		}
    }
    __NOP();
    Sim_enterSLEEPMode();
    Sim_isRISignal = false; //After reset, RISignal will raise
	#if (PWRMODE == 0)
	PWR_ctrlPwrSensor(OFF);
	#endif
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
		  Sim_connectHTTP(HostConfig, URLConfig, StationConfig, (char*)HTTP_DataToGet);
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

	  if(isOnTimeToSendReq)
	  {
		  #if (PWRMODE == 0)
		  PWR_ctrlPwrSensor(ON);
		  #endif
		  Sim_exitSLEEPMode();
		  //Kiem tra tin hieu Sim
		  Sim_RSSI = Sim_getSignalQuality();
		  //Doc dien ap
		  PWR_getPwrSupply(0);
		  HAL_Delay(10000);
		  HAL_Delay(10000);
		  //Doc du lieu cam bien
		  /*-------RS485-------*/
		  float rs485_arr[5] = {0};
		  for(uint8_t i = 0; i < 5; i++)
		  {
		  	rs485_arr[i] = RS485_readLiquidLevelTransmitter(0x01);
		  }
		  insertionSort_2(rs485_arr,5);
		  if(rs485_arr[4] == -32767)
		  {
			  levelWater = 0;
			  //bao trang thai cam bien
			  memset(sensorStatus,0,strlen(sensorStatus));
			  strcpy(sensorStatus,"ERROR");
		  }
		  else
		  {
			  levelWater = rs485_arr[2];
			  if(levelWater < 0 || levelWater > 10) levelWater = 0;
			  memset(sensorStatus,0,strlen(sensorStatus));
			  strcpy(sensorStatus,"OK");
		  }
		  /*-------420mA-------*/
		  float adc_arr[5] = {0};
		  for(uint8_t i = 0; i < 5; i++)
		  {
			  adc_arr[i] = Sensor_getADCValue(2);
		  }
		  insertionSort_2(adc_arr,5);
		  Sensor_CalibCurrent = adc_arr[2];
		  if((Sensor_CalibCurrent > 3.9) && (Sensor_CalibCurrent < 4.01))
		  {
		  	Sensor_CalibCurrent = 4.00;
		  }
		  else if((Sensor_CalibCurrent > 20.01) && (Sensor_CalibCurrent < 21))
		  {
		  	Sensor_CalibCurrent = 20.00;
		  }
		  levelWater = ADC_map(Sensor_CalibCurrent, 4.00, 20.00, 0.0, 15); //donvi: met
		  if((levelWater < 0.0) || (levelWater > 16.0))
		  {
			  levelWater = 0;
			  memset(sensorStatus,0,strlen(sensorStatus));
			  strcpy(sensorStatus,"ERROR");
		  }
		  else
		  {
			  memset(sensorStatus,0,strlen(sensorStatus));
			  strcpy(sensorStatus,"OK");
		  }
		  //Tong hop du lieu
		  strcpy(MainData,"TEST DATA");
		  //Reset du lieu
		  rainRaw = 0;
		  #if (PWRMODE == 0)
		  PWR_ctrlPwrSensor(OFF);
		  #endif
		  //Gui du lieu len Server
		  ret = Sim_connectHTTP(Host1, URL1, MainData, (char*)HTTP_DataToGet);
		  if(ret != RET_OK)
		  {
			  Flash_writeData(Host1, NULL, MainData);
			  Flash_isReadData = false;
		  }
		  else
		  {
			  Flash_isReadData = true;
		  }
		  //Reset du lieu
		  memset(MainData,0,strlen(MainData));
		  Sim_enterSLEEPMode();
		  isOnTimeToSendReq = false;
	  }
	  else
	  {
		  if(Flash_isReadData == true)
		  {
			  if(READ_DATA_ADDR != WRITE_DATA_ADDR)
			  {
				  memset(Flash_DataToRead, 0, strlen(Flash_DataToRead));
				  READ_DATA_ADDR = Flash_ReadIntType(RD_ADDR);
				  WRITE_DATA_ADDR = Flash_ReadIntType(WR_ADDR);
				  PAGE_ADDR = Flash_ReadIntType(PG_ADDR);
				  Flash_ReadCharType(Flash_DataToRead, READ_DATA_ADDR, FLASH_TYPEPROGRAM_HALFWORD);
				  Flash_ReadDataLen = strlen(Flash_DataToRead);
				  if(Flash_ReadDataLen != 0)
				  {
					  Flash_extractData(Flash_DataToRead);
				  }
				  ret = Sim_connectHTTP(Flash_Host, Flash_URL, Flash_Data, (char*)HTTP_DataToGet);
				  if(ret == RET_OK)
				  {
					  READ_DATA_ADDR += Flash_ReadDataLen*2+2;
					  if(READ_DATA_ADDR == MAX_PAGE_ADDR)
					  {
						  READ_DATA_ADDR = MIN_DATA_ADDR;
					  }
					  Flash_Unlock();
					  Flash_Erase(RD_ADDR);
					  Flash_WriteIntType(RD_ADDR, READ_DATA_ADDR,FLASH_TYPEPROGRAM_WORD);
					  Flash_WriteIntType(WR_ADDR, WRITE_DATA_ADDR,FLASH_TYPEPROGRAM_WORD);
					  Flash_WriteIntType(PG_ADDR,PAGE_ADDR,FLASH_TYPEPROGRAM_WORD);
					  Flash_Lock();
				  }
				  else //gui that bai -> khong cho phep doc bo nho Flash
				  {
					  Flash_isReadData = false;
				  }
			  }
			  else //gui het du lieu Flash -> dua ve Sleepmode
			  {
				  if(Sim_SleepMode == false)
				  {
					  Sim_enterSLEEPMode();
				  }
			  }
		  }
		  else //khong cho phep doc du lieu Flash -> dua ve Sleepmode
		  {
			  if(Sim_SleepMode == false)
			  {
				  Sim_enterSLEEPMode();
			  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */
  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 2;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SIM_RESET_Pin|SIM_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIM_DTR_Pin|CTR_PWR_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SIM_RESET_Pin SIM_PWR_Pin */
  GPIO_InitStruct.Pin = SIM_RESET_Pin|SIM_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SIM_DTR_Pin CTR_PWR_SS_Pin */
  GPIO_InitStruct.Pin = SIM_DTR_Pin|CTR_PWR_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SIM_RI_Pin */
  GPIO_InitStruct.Pin = SIM_RI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SIM_RI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_EN_Pin */
  GPIO_InitStruct.Pin = RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_1_Pin IN_2_Pin */
  GPIO_InitStruct.Pin = IN_1_Pin|IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_3_Pin */
  GPIO_InitStruct.Pin = IN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_4_Pin */
  GPIO_InitStruct.Pin = IN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_4_GPIO_Port, &GPIO_InitStruct);

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
									Flash_WriteIntType(MODE_RTC_ADDR, RTC_ALARM, FLASH_TYPEPROGRAM_HALFWORD);
									HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
									HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
									RTC_initAlarm(currentTime.Hours, currentTime.Minutes, 0);
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
		//update RTC Time
		RTC_updateUserTime();
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

void RTC_updateUserTime(void)
{
	bool isCorrectTime = false;
	RTC_TimeTypeDef m_Time = currentTime;
	RTC_DateTypeDef m_Date = currentDate;
	TimeStamp.year -= 2000;
	TimeStamp.hour += 7;
	if(TimeStamp.year < 99)
	{
		if((TimeStamp.mon > 0) && (TimeStamp.mon < 13))
		{
			if((TimeStamp.date > 0) && (TimeStamp.date < 32))
			{
				if((TimeStamp.hour > 6) && (TimeStamp.hour < 31)) //7->30
				{
					if(TimeStamp.min < 60)
					{
						if(TimeStamp.sec < 60)
						{
							isCorrectTime = true;
						}
					}
				}
			}
		}
	}
	if(isCorrectTime)
	{
		//cap nhat ngay thang cho DateToUpdate
		m_Date.Date = TimeStamp.date;
		m_Date.Month = TimeStamp.mon;
		m_Date.Year = TimeStamp.year;
		if(TimeStamp.hour > 23)
		{
			TimeStamp.hour = TimeStamp.hour - 24;
			RTC_updateUserDate(&m_Date);
		}
		else
		{
			HAL_RTC_SetDate(&hrtc, &m_Date, RTC_FORMAT_BIN);
		}
		m_Time.Hours = TimeStamp.hour;
		m_Time.Minutes = TimeStamp.min;
		m_Time.Seconds = TimeStamp.sec;
		HAL_RTC_SetTime(&hrtc, &m_Time, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &m_Date, RTC_FORMAT_BIN);
		RTC_initAlarm(m_Time.Hours, m_Time.Minutes, 0);
	}
}

void RTC_updateUserDate(RTC_DateTypeDef* datetime)
{
	if ((datetime->Month == 1U) || (datetime->Month == 3U) || (datetime->Month == 5U) || (datetime->Month == 7U) || \
        (datetime->Month == 8U) || (datetime->Month == 10U) || (datetime->Month == 12U))
	{
		if (datetime->Date < 31U)
		{
			datetime->Date++;
		}
		/* Date structure member: day = 31 */
		else
		{
			if (datetime->Month != 12U)
			{
				datetime->Month++;
				datetime->Date = 1U;
			}
			/* Date structure member: day = 31 & month =12 */
			else
			{
				datetime->Month = 1U;
				datetime->Date = 1U;
				datetime->Year++;
			}
		}
	}
	else if ((datetime->Month == 4U) || (datetime->Month == 6U) || (datetime->Month == 9U) || (datetime->Month == 11U))
	{
		if (datetime->Date < 30U)
		{
			datetime->Date++;
		}
		/* Date structure member: day = 30 */
		else
		{
			datetime->Month++;
			datetime->Date = 1U;
		}
	}
	else if (datetime->Month == 2U)
	{
		if (datetime->Date < 28U)
		{
			datetime->Date++;
		}
		else if (datetime->Date == 28U)
		{
			/* Leap year */
			if (RTC_isLeapYear(datetime->Year))
			{
				datetime->Date++;
			}
			else
			{
				datetime->Month++;
				datetime->Date = 1U;
			}
		}
		else if (datetime->Date == 29U)
		{
			datetime->Month++;
			datetime->Date = 1U;
		}
	}
	HAL_RTC_SetDate(&hrtc, datetime, RTC_FORMAT_BIN);
}

bool RTC_isLeapYear(uint16_t y)
{
  if((y + 2000) % 4 != 0)
  {
    return false;
  }
  else if((y + 2000) % 100 != 0)
  {
    return true;
  }
  else if((y + 2000) % 400 != 0)
  {
    return false;
  }
  else return true;
}

void Flash_init(void)
{
	/* Flash init */
	if(Flash_ReadIntType(WR_ADDR) == 0xFFFFFFFF)
	{
		Flash_WriteIntType(WR_ADDR, WRITE_DATA_ADDR, FLASH_TYPEPROGRAM_HALFWORD);
	}
	if(Flash_ReadIntType(RD_ADDR) == 0xFFFFFFFF)
	{
		Flash_WriteIntType(RD_ADDR, READ_DATA_ADDR, FLASH_TYPEPROGRAM_HALFWORD);
	}
	if(Flash_ReadIntType(PG_ADDR) == 0xFFFFFFFF)
	{
		Flash_WriteIntType(PG_ADDR, PAGE_ADDR, FLASH_TYPEPROGRAM_HALFWORD);
	}
	WRITE_DATA_ADDR = Flash_ReadIntType(WR_ADDR);
	READ_DATA_ADDR = Flash_ReadIntType(RD_ADDR);
	PAGE_ADDR = Flash_ReadIntType(PG_ADDR);
	if(Flash_ReadIntType(MODE_RTC_ADDR) == 0xFFFFFFFF)
	{
		Flash_WriteIntType(MODE_RTC_ADDR, RTC_ALARM, FLASH_TYPEPROGRAM_HALFWORD);
	}
	RTC_ALARM = Flash_ReadIntType(MODE_RTC_ADDR);
	RTC_initAlarm(userAlarm.AlarmTime.Hours,userAlarm.AlarmTime.Minutes,0);
}

void Flash_writeData(const char* host, const char* url, const char* data)
{
	memset(Flash_DataToWrite,0,strlen(Flash_DataToWrite));
	if((host != NULL) && (url != NULL))
	{
		sprintf(Flash_DataToWrite,"%s|%s|%s",host,url,data);
	}
	else
	{
		if(strstr(host,Host1) != NULL)
		{
			sprintf(Flash_DataToWrite,"%s||1",data);
		}
		else
		{
			sprintf(Flash_DataToWrite,"%s||2",data);
		}
	}
	Flash_Unlock();
	Flash_WriteDataLen = strlen(Flash_DataToWrite);
	if((WRITE_DATA_ADDR + Flash_WriteDataLen*2) >= PAGE_ADDR)
	{
		if(PAGE_ADDR == MAX_PAGE_ADDR)
		{
			PAGE_ADDR = MIN_PAGE_ADDR;
			WRITE_DATA_ADDR = MIN_DATA_ADDR;
			Flash_Erase(WRITE_DATA_ADDR);
		}
		else
		{
			PAGE_ADDR += 0x800;
		}
		if(READ_DATA_ADDR > WRITE_DATA_ADDR)
		{
			READ_DATA_ADDR = PAGE_ADDR + 0x10 - 0x800;
		}
		Flash_Unlock();
		Flash_Erase(PAGE_ADDR);
	}
	Flash_WriteCharType(WRITE_DATA_ADDR, Flash_DataToWrite, FLASH_TYPEPROGRAM_HALFWORD);
	WRITE_DATA_ADDR += Flash_WriteDataLen*2+2;
	Flash_Erase(WR_ADDR);
	//Luu PAGE_ADDR, WRITE_DATA_ADDR, READ_DATA_ADDR vao bo nho Flash
	Flash_WriteIntType(RD_ADDR, READ_DATA_ADDR, FLASH_TYPEPROGRAM_WORD);
	Flash_WriteIntType(WR_ADDR, WRITE_DATA_ADDR, FLASH_TYPEPROGRAM_WORD);
	Flash_WriteIntType(PG_ADDR, PAGE_ADDR, FLASH_TYPEPROGRAM_WORD);
	Flash_Lock();
}

void Flash_extractData(char* data)
{
	memset(Flash_Host,0,strlen(Flash_Host));
	memset(Flash_URL,0,strlen(Flash_URL));
	memset(Flash_Data,0,strlen(Flash_Data));
	if(strstr(data,"||") == NULL)
	{
		char* token;
		token = strtok(data,"|");
		strcpy(Flash_Host,token);
		token = strtok(NULL,"|");
		strcpy(Flash_URL,token);
		token = strtok(NULL,"|");
		strcpy(Flash_Data,token);
	}
	else
	{
		char* token;
		token = strtok(data,"|");
		strcpy(Flash_Data,token);
		token = strtok(NULL,"|");
		token = strtok(NULL,"|");
		if(atoi(token) == 1)
		{
			strcpy(Flash_Host,Host1);
			strcpy(Flash_URL,URL1);
		}
		else
		{
			strcpy(Flash_Host,Host2);
			strcpy(Flash_URL,URL2);
		}
	}
}

float RS485_readUltrasonicSensor(uint8_t Modbus_Addr)
{
	// Change address 01 - 02	: 01 10 00 19 00 02 40 00 00 00 08 C3
	// Read data				: 02 03 00 01 00 01 D5 F9
	// Response					:	02 03 04 40 3F AF 58 90 F5 -> 40 3F AF 58 is data type float
	uint8_t ReceivedBuff[10] ={0};
	union u {
		uint8_t c[4];
		float f;
	} value;
	int32_t rawValue;
	RS485Query_t m_RS485Query=
	{
		.slaveAddress = Modbus_Addr,
		.mbFunction 	= Read_HoldingRegister,
		.regAddress 	= 0x01,
		.regCount		= 0x01
	};
	for(uint8_t j=0;j<3;j++)
	{
		RS485_transmit(m_RS485Query);
		if(HAL_UART_Receive(&huart3,(uint8_t*)ReceivedBuff, 9, 1000) == HAL_OK)
		{
			RS485Data_t UltrasonicSensor;
			if(RS485_analysis(&UltrasonicSensor, m_RS485Query, (uint8_t*)ReceivedBuff, 9) == 1)
			{
				__NOP();
				rawValue = (((((int32_t)UltrasonicSensor.dataReg[0]))<<16)|UltrasonicSensor.dataReg[1]);
				value.c[0] = (uint8_t)((rawValue>>0)&0xFF);
				value.c[1] = (uint8_t)((rawValue>>8)&0xFF);
				value.c[2] = (uint8_t)((rawValue>>16)&0xFF);
				value.c[3] = (uint8_t)((rawValue>>24)&0xFF);
//				sensorValue = (float)rawValue;
				return value.f;
			}
			else
			{
				__NOP();
			}
		}
		else
		{
			__NOP();
		}
		HAL_Delay(200);
	}
	return -32767;
}

// don vi met
float RS485_readLiquidLevelTransmitter(uint8_t Modbus_Addr)
{
	#if (IWDOG == 1)
	HAL_IWDG_Refresh(&hiwdg);
	#endif
	uint8_t ReceivedBuff[10] ={0};
	uint16_t sensorValue=0;
	RS485Query_t m_RS485Query=
	{
		.slaveAddress = Modbus_Addr,
		.mbFunction 	= Read_HoldingRegister,
		.regAddress 	= 0x04,
		.regCount		= 0x01
	};
	for(uint8_t j=0;j<3;j++)
	{
		RS485_transmit(m_RS485Query);
		if(HAL_UART_Receive(&huart3,(uint8_t*)ReceivedBuff, 7, 1000) == HAL_OK)
		{
			RS485Data_t LiquidLevelTransmitter;
			if(RS485_analysis(&LiquidLevelTransmitter, m_RS485Query, (uint8_t*)ReceivedBuff, 7) == 1)
			{
				sensorValue = LiquidLevelTransmitter.dataReg[0];
				HAL_Delay(200);
				return (float)(sensorValue)/100;
			}
			else
			{
				__NOP();
			}
		}
		else
		{
			__NOP();
		}
		HAL_Delay(200);
	}
	return -32767;
}

float Sensor_getADCValue(uint8_t channel)
{
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)Sensor_ADCval, 2);
	HAL_Delay(50);
	// Lay NSAMPLE mau
	for(uint8_t i=0; i<NSAMPLE; i++)
	{
		ADC_ArrayVal[i] = Sensor_ADCval[channel];
		HAL_Delay(2);
	}
	//Sap xep theo gia tri tang dan
	insertionSort(ADC_ArrayVal, NSAMPLE);
	// Tinh trung binh gia tri khoang giua
	ADC_Count = ADC_Avr = 0;
	for(uint8_t i = NSAMPLE/4+1; i < NSAMPLE*3/4+1; i++)
	{
		ADC_Avr += ADC_ArrayVal[i];
		ADC_Count++;
	}
	ADC_Avr /= ADC_Count;
	Sensor_Current = (ADC_Avr*VOLPOWER/4095)*10;
	ADC_Count = ADC_Avr = 0;
	for(uint8_t i=0; i<NSAMPLE; i++)
	{
		ADC_ArrayVal[i] = 0;
	}
	HAL_ADC_Stop_DMA(&hadc2);
	HAL_Delay(10);
	if(channel == 0)
	{
		Sensor_CalibCurrent = -0.00008*Sensor_Current*Sensor_Current + \
													1.0024*Sensor_Current + 0.0137;
	}
	else if(channel == 1)
	{
		Sensor_CalibCurrent = -0.0001*Sensor_Current*Sensor_Current + \
													1.0013*Sensor_Current + 0.0066;
	}
	if((int)Sensor_CalibCurrent != 0) return Sensor_CalibCurrent;
	return 0.0;
}

float ADC_map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PWR_ctrlPwrSensor(Pwr_State status)
{
	HAL_GPIO_WritePin(CTR_PWR_SS_GPIO_Port, CTR_PWR_SS_Pin, status? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_Delay(2000);
}

void PWR_getPwrSupply(uint8_t channel)
{
	HAL_ADC_Start_DMA(&hadc4, (uint32_t*) ADC_value, 2);
	HAL_Delay(1);
	for(uint8_t i=0; i<NSAMPLE; i++)
	{
		ADC_ArrayVal[i] = ADC_value[channel];
		HAL_Delay(2);
	}
	//Sap xep theo gia tri tang dan
	insertionSort(ADC_ArrayVal, NSAMPLE);
	// Tinh trung binh gia tri khoang giua
	ADC_Count = ADC_Avr = 0;
	for(uint8_t i = NSAMPLE/4+1; i < NSAMPLE*3/4+1; i++)
	{
		ADC_Avr += ADC_ArrayVal[i];
		ADC_Count++;
	}
	ADC_Avr /= ADC_Count;
	Voltage = CALCULATE_VIN(ADC_Avr);
	for(uint8_t i=0; i<NSAMPLE; i++)
	{
		ADC_ArrayVal[i] = 0;
	}
	ADC_Count = ADC_Avr = 0;
	HAL_ADC_Stop_DMA(&hadc4);
	HAL_Delay(10);
}

void insertionSort(uint16_t arr[], int n)
{
	int i, key, j;
	for (i = 1; i < n; i++)
	{
		key = arr[i];
		j = i - 1;
		while (j >= 0 && arr[j] > key) {
				arr[j + 1] = arr[j];
				j = j - 1;
		}
		arr[j + 1] = key;
	}
}

void insertionSort_2(float arr[], int n)
{
	int i, j;
	float key;
	for (i = 1; i < n; i++)
	{
		key = arr[i];
		j = i - 1;
		while (j >= 0 && arr[j] > key) {
				arr[j + 1] = arr[j];
				j = j - 1;
		}
		arr[j + 1] = key;
	}
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
