/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "rf.h"
#include "rtc.h"
#include "app_entry.h"
#include "app_common.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_seq.h"
#include "string.h"
#include "stdbool.h"
#include "AT0x.h"
#include "MMA865x.h"
#include "L80.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Test result */
typedef enum eTestStatus
{
	RET_FAIL=0,
	RET_OK,
	RET_TIMEOUT
}eTestStatus;
eTestStatus SYS_test, EEPROM_test, ACL_test, DISTANCE_test, LORA_test, GPS_test, ADC_test, BLE_test, BUTTON_test;

/* I2C Transfer */
typedef enum {
	I2C_W  = 1,
	I2C_R  = 2,
	I2C_WR = 4,
	I2C_WW = 8
} eI2CMode;

typedef struct {
	eI2CMode I2CMode_T;
	uint16_t DeviceAddress_U16;
	uint16_t Register_16;
	uint8_t* Data_U8P;
	uint16_t Length_U16;
}tI2CPackage;

/* EEPROM */
typedef enum {
	Flash_W  = 1,
	Flash_R  = 2,
	Flash_WR = 4,
	Flash_WW = 8
} eFlashMode;

typedef struct {
	eFlashMode FlashMode_T;
	uint16_t FlashAddress_U16;
	uint16_t MemoryAddress_U16;
	uint8_t *Data_U8P;
	uint16_t Length_U16;
}tFlashData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USED				1
#define NOT_USED			0
#define ENDLESS_LOOP_ACL	NOT_USED
#define ENDLESS_LOOP_DYP	NOT_USED
#define ENDLESS_BATT_MEASURING	NOT_USED
#define DEBUG_UART			USED
#define DEBUG_AT_UART		NOT_USED
#define FW_TEST				USED
#define EEPROM_TEST 		USED
#define	ACL_TEST			USED
#define	DISTANCE_TEST		USED
#define GPS_TEST			USED
#define ADC_TEST			USED
#define LORA_TEST			USED
#define CHANGE_DEVEUI		USED
#define TEST_DOWNLINK		NOT_USED
#define	TEST_SLEEPING		USED
///////////////////////////////////////////////////////////////////////////////
#if DEBUG_UART
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}
#endif
///////////////////////////////////////////////////////////////////////////////
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile bool g_acl_interrupt=false;
volatile bool g_rak4200_newdata=false;
volatile bool g_rtcwakeup=false;
volatile bool g_testingble=false;
RTC_TimeTypeDef curTime = {0};
RTC_DateTypeDef curDate = {0};
RTC_TimeTypeDef eventTime = {0};
RTC_DateTypeDef eventDate = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void MCU_Init(void);

#if FW_TEST

//System testing function
eTestStatus Sys_Test(void);
eTestStatus FW_Test1(void);
//Driver for testing
HAL_StatusTypeDef __attribute__((weak)) I2C_Transferring(tI2CPackage *I2CPackage_T);
//Function for testing devices
eTestStatus EEPROM_FWTest(void);
eTestStatus ACL_FWTest(void);
eTestStatus DISTANCE_FWTest(void);
eTestStatus LORA_FWTest(void);
eTestStatus GPS_FWTest(void);
eTestStatus ADC_FWTest(void);
eTestStatus BLE_FWTest(void);

void ButtonsHandler(void);
void EnterStopMode(void);
void DebugProbeInit(void);
#endif /*End of FW_TEST*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_RF_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);
  HAL_Delay(500);
  Sys_Test();
//  FW_Test1();

  printf("Testing BLE function (including button test)\n");
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  APPE_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  UTIL_SEQ_Run(~0);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */
/**************************************************************************************/
/* Main FW Test */
eTestStatus Sys_Test(void)
{
	SYS_test = RET_FAIL;
	DebugProbeInit();
	printf("FW Test started... \n");
	printf("Testing ACL ...\n");
	if(ACL_FWTest() == RET_OK )		printf("FW Test ACL: OK \n");
	else							printf("FW Test ACL: Not OK \n");
	printf("Testing EEPROM ...\n");
	if(EEPROM_FWTest() == RET_OK )	printf("FW Test EEPROM: OK \n");
	else							printf("FW Test EEPROM: Not OK \n");
	printf("Testing Distance sensor ...\n");
	if(DISTANCE_FWTest() == RET_OK) printf("FW Test Distance: OK \n");
	else							printf("FW Test Distance: Not OK \n");
	printf("Testing LoRa ...\n");
	if(LORA_FWTest() == RET_OK)	 	printf("FW Test LoRa: OK \n");
	else							printf("FW Test LoRa: Not OK \n");
	printf("Testing ADC ...\n");
	if(ADC_FWTest() == RET_OK)	 	printf("FW Test ADC: OK \n");
	else							printf("FW Test ADC: Not OK \n");
	printf("Testing GPS ...\n");
	if(GPS_FWTest() == RET_OK)		printf("\nFW Test GPS: OK \n");
	else							printf("FW Test GPS: Not OK \n");
	SYS_test = 	EEPROM_test & ACL_test & DISTANCE_test & LORA_test & GPS_test & ADC_test;
	if(SYS_test == RET_OK)			printf("\nFW Test: OK \n");
	else							printf("FW Test: Not OK \n");
	EnterStopMode();
	ButtonsHandler();
	if(g_acl_interrupt == true)
	{
		printf("ACL detected motion \n");
		g_acl_interrupt = false;
	}
	else printf("Wake up by button \n");
	HAL_Delay(100);

	return SYS_test;
}

eTestStatus FW_Test1(void)
{
	eTestStatus fw1_teststatus = RET_FAIL;
	printf("FW1 Test started... \n");
	printf("Testing LoRa ...\n");
	if(LORA_FWTest() == RET_OK)	 	printf("FW Test LoRa: OK \n"); /* Sending uplink after 30s */
	else							printf("FW Test LoRa: Not OK \n");
	//Init alarm
	RTC_AlarmTypeDef sAlarm = {0};
	sAlarm.AlarmTime.Hours = 0;
	sAlarm.AlarmTime.Minutes = 0;
	sAlarm.AlarmTime.Seconds = 10;
	sAlarm.AlarmTime.SubSeconds = 0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY| RTC_ALARMMASK_HOURS| RTC_ALARMMASK_MINUTES;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
	  Error_Handler();
	}
	while(1)
	{
		if(g_rtcwakeup == true)
		{
//			sAlarm.AlarmTime.Seconds += 30;
//			if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
//			{
//				Error_Handler();
//			}
			printf("Alarm occurred\n");
			g_rtcwakeup = false;
//			sAlarm.AlarmTime.Minutes+=5;
		}
//		EnterStopMode();
	}
	return fw1_teststatus;

}

/**************************************************************************************/
/* EEPROM FIRMWARE TEST - Read & Write data */
#define START_ADDR		0

#define LENGTH			50

uint8_t g_ui8dataRecv[LENGTH]={	1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
								1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
								1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
								1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
								1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint8_t g_ui8dataWrite[LENGTH]={ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
								11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
								21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
								31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
								41, 42, 43, 44, 45, 46, 47, 48, 49};
uint8_t g_ui8dataNull[LENGTH]={	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static void EEPROM_Read(uint16_t Address_U16, uint8_t* Data_U8P, uint16_t Length_U16)
{
	tFlashData FlashData;
	FlashData.MemoryAddress_U16	= Address_U16;
	FlashData.Data_U8P			= Data_U8P;
	FlashData.Length_U16		= Length_U16;

	FlashData.FlashMode_T = Flash_WR;
	FlashData.FlashAddress_U16 = mFlashAddress(FlashData.MemoryAddress_U16);
	FlashData.MemoryAddress_U16 = mMemoryAddress(FlashData.MemoryAddress_U16);
	I2C_Transferring((tI2CPackage*)&FlashData);
	HAL_Delay(5);
}

static void EEPROM_ReadUID(uint16_t Address_U16, uint8_t* Data_U8P, uint16_t Length_U16)
{
	tFlashData FlashData;
	FlashData.MemoryAddress_U16	= Address_U16;
	FlashData.Data_U8P			= Data_U8P;
	FlashData.Length_U16		= Length_U16;

	FlashData.FlashMode_T = Flash_WR;
	FlashData.FlashAddress_U16 = 0xb0;
	FlashData.MemoryAddress_U16 = mMemoryAddress(FlashData.MemoryAddress_U16);
	I2C_Transferring((tI2CPackage*)&FlashData);
	HAL_Delay(5);
}

static void EEPROM_WritePage(tFlashData FlashData_T)
{
	if(((FlashData_T.MemoryAddress_U16 & (cPageSize_U16-1)) + FlashData_T.Length_U16) > cPageSize_U16)
	{
		FlashData_T.Length_U16 = cPageSize_U16 - (FlashData_T.MemoryAddress_U16 & (cPageSize_U16-1));
	}

	FlashData_T.FlashMode_T = Flash_WW;
	FlashData_T.FlashAddress_U16  = mFlashAddress(FlashData_T.MemoryAddress_U16);
	FlashData_T.MemoryAddress_U16 = mMemoryAddress(FlashData_T.MemoryAddress_U16);

	I2C_Transferring((tI2CPackage*)&FlashData_T);
	HAL_Delay(5);
}

static void EEPROM_Write( uint16_t Address_U16, uint8_t* Data_U8P, uint16_t Len_U16)
{
	tFlashData FlashData;

	FlashData.MemoryAddress_U16	= Address_U16;
	FlashData.Data_U8P			= Data_U8P;
	FlashData.Length_U16		= Len_U16;

	// Write first page if not aligned
	uint16_t Length_U16 = FlashData.Length_U16;
	uint8_t	 pageOffset = FlashData.MemoryAddress_U16 % cPageSize_U16;

	FlashData.Length_U16 = 0;

    if (pageOffset > 0)
    {
    	FlashData.Length_U16 = cPageSize_U16 - pageOffset;

    	if (FlashData.Length_U16 >= Length_U16)
    		FlashData.Length_U16 = Length_U16;

    	EEPROM_WritePage(FlashData);
        Length_U16 -= FlashData.Length_U16;
    }

    if (Length_U16 > 0)
	{
    	FlashData.MemoryAddress_U16	+= FlashData.Length_U16;
    	FlashData.Data_U8P 			+= FlashData.Length_U16;

		// Write complete and aligned pages.
		uint8_t pageCount = Length_U16 / cPageSize_U16;

		for (uint8_t i = 0; i < pageCount; i++)
		{
			FlashData.Length_U16 = cPageSize_U16;
			EEPROM_WritePage(FlashData);

			FlashData.MemoryAddress_U16	+= cPageSize_U16;
			FlashData.Data_U8P		 	+= cPageSize_U16;
			Length_U16						-= cPageSize_U16;

		}

		if (Length_U16 > 0)
		{
			// Write remaining uncomplete page.
			FlashData.Length_U16 = Length_U16;
			EEPROM_WritePage(FlashData);
		}
	}
}

eTestStatus EEPROM_FWTest(void)
{
	uint8_t uid[32]={0};
	EEPROM_test = RET_FAIL;
	#if EEPROM_TEST
	HAL_GPIO_WritePin(EEPROM_EN_GPIO_Port, EEPROM_EN_Pin, GPIO_PIN_SET);	//Enable EEPROM module
	HAL_Delay(100);															//Wait for stable
	////////////////////////////////////////////////////////////
	EEPROM_ReadUID(0x80, uid, 32);
	////////////////////////////////////////////////////////////
	EEPROM_Write(START_ADDR, g_ui8dataWrite, LENGTH);
	EEPROM_Read(START_ADDR, g_ui8dataRecv, LENGTH);
	if( memcmp((char*)g_ui8dataRecv, (char*)g_ui8dataWrite, LENGTH) ==0) //Test Flash
	{
		EEPROM_test = RET_OK;
	}
	#endif /*End of EEPROM_TEST*/
	return EEPROM_test;
}

/**************************************************************************************/
/* ACL FIRMWARE TEST - Read raw data & Interrupt settings */

int16_t g_rawX=0, g_rawY=0, g_rawZ=0;

static void ACL_Read(uint8_t Register_U8, uint8_t* Data_U8P, uint8_t Length_U8)
{
	tI2CPackage I2CPackage_T;

	I2CPackage_T.I2CMode_T			= I2C_WR;
	I2CPackage_T.DeviceAddress_U16	= cMMA865X_Address;
	I2CPackage_T.Register_16		= Register_U8;
	I2CPackage_T.Data_U8P			= Data_U8P;
	I2CPackage_T.Length_U16			= Length_U8;
	I2C_Transferring(&I2CPackage_T);
}

static void ACL_Write(uint8_t Register_u8, uint8_t* Data_U8P, uint8_t Length_U8)
{
	tI2CPackage I2CPackage_T;

	I2CPackage_T.I2CMode_T			= I2C_WW;
	I2CPackage_T.DeviceAddress_U16	= cMMA865X_Address;
	I2CPackage_T.Register_16		= Register_u8;
	I2CPackage_T.Data_U8P			= Data_U8P;
	I2CPackage_T.Length_U16			= Length_U8;
	I2C_Transferring(&I2CPackage_T);
}

static uint8_t ACL_RegisterGet(uint8_t Register_U8)
{
	uint8_t Result_U8;
	ACL_Read(Register_U8, &Result_U8, 1);

	return Result_U8;
}

static void ACL_RegisterSet(uint8_t Register_U8, uint8_t Data_U8)
{
	ACL_Write(Register_U8, &Data_U8, 1);
}

static void ACL_Standby()
{
	ACL_RegisterSet(MMA865X_CTRL_REG1, 0);
}

static void ACL_Active()
{
	ACL_RegisterSet(MMA865X_CTRL_REG1, 0x49);
}

static bool ACL_Init(void)
{
	// Check Who AM I
	if (ACL_RegisterGet(MMA865X_WHO_AM_I) == cMMA865X_WHO_AM_I)	// 0x4A: 8652, 0x5A: 8653
	{
		ACL_test = RET_OK;
	}
	else
	{
		printf("ACL wasn't initialized properly, exit ACL test \n");
		return false;
	}

	// Place MMA865X in standby mode
	ACL_Standby();
	HAL_Delay(1);

	ACL_RegisterSet(MMA865X_XYZ_DATA_CFG, 0x00);	// Accelerometer range of +/-2g range with 0.244mg/LSB, no high-pass filtered
	HAL_Delay(1);

	ACL_RegisterSet(MMA865X_CTRL_REG3, 0x3A);	// 3A: Push-Pull output; 3B: Open-Drain output
	HAL_Delay(1);

	// Trigger the FIFO - Freefall/Motion Trigger
	ACL_RegisterSet(MMA865X_TRIG_CFG, 0x04);
	HAL_Delay(1);

	// Enter active mode
	ACL_Active();

	ACL_Standby();
	return true;
}

static void ACL_ReadAxis()
{
	uint8_t Buffer_U8[7];
	int16_t Temp_U16;

	ACL_Read(MMA865X_STATUS_00, Buffer_U8, 7);

	//X axis
	Temp_U16 = ((Buffer_U8[1] << 8) | Buffer_U8[2]) >> 4;

	if (Buffer_U8[1] > 0x7F)
		g_rawX = -((~Temp_U16 + 1) & 0xFFF);
	else
		g_rawX = Temp_U16;

	//Y axis
	Temp_U16 = ((Buffer_U8[3] << 8) | Buffer_U8[4]) >> 4;

	if (Buffer_U8[3] > 0x7F)
		g_rawY = -((~Temp_U16 + 1) & 0xFFF);
	else
		g_rawY = Temp_U16;

	//Z axis
	Temp_U16 = ((Buffer_U8[5] << 8) | Buffer_U8[6]) >> 4;

	if (Buffer_U8[5] > 0x7F)
		g_rawZ = -((~Temp_U16 + 1) & 0xFFF);
	else
		g_rawZ = Temp_U16;
}

static void ACL_EnableInterrupt(void)
{
	ACL_Standby();
	ACL_RegisterSet(MMA865X_CTRL_REG4, 0x04);

	ACL_RegisterSet(MMA865X_CTRL_REG5, 0x04);
	ACL_RegisterSet(MMA865X_FF_MT_CFG, 0xF8); // Motion detection - XYZ events enabled
	ACL_RegisterSet(MMA865X_FF_MT_THS, 25);
	ACL_Active();
}

static void ACL_ReadSource(void)
{
	ACL_Active();
	ACL_RegisterGet(MMA865X_INT_SOURCE);
	ACL_EnableInterrupt();
}

eTestStatus ACL_FWTest(void)
{
	ACL_test = RET_FAIL;
	#if ACL_TEST
	if(ACL_Init() == false) return ACL_test; //break immediately if can't initialized ACL
	ACL_Active();
	ACL_EnableInterrupt();
	ACL_ReadAxis();
	while(1)
	{
		ACL_ReadAxis();
		printf("Raw data ACL sensors x y z: %d : %d : %d \n",g_rawX, g_rawY, g_rawZ);
		HAL_Delay(50);
		#if !ENDLESS_LOOP_ACL
		break;
		#endif /* ENDLESS_LOOP_ACL */
	}
	#if TEST_SLEEPING
	ACL_Standby();	/* Prevent wake up from ACL */
	#endif /*End of TEST_SLEEPING*/
	#endif /*End of ACL_Test*/
	return ACL_test;
}

/**************************************************************************************/
/* DYP Distance sensor FW TEST - Read data */
#define DISTANCE_IRQ_SOURCE		EXTI3_IRQn
#define MAX_RANGE 				400 //cm
#define MAX_TIME_COUNT			(uint16_t)(MAX_RANGE*57.5)

volatile bool g_new_distancedata = false;
uint16_t g_time_ivt=0;
float g_distance=0.0;
float g_avgdistance=0.00;

static void Trigger_DistanceRX(void)
{
	HAL_GPIO_WritePin(TRIGGER_CABLE_GPIO_Port, TRIGGER_CABLE_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(TRIGGER_CABLE_GPIO_Port, TRIGGER_CABLE_Pin, GPIO_PIN_SET);
}

static bool DISTANCE_GetData(float *distance_fl)
{
	bool retval = false;
	HAL_GPIO_WritePin(DISTANCE_EN_GPIO_Port, DISTANCE_EN_Pin, GPIO_PIN_SET);
	HAL_NVIC_EnableIRQ(DISTANCE_IRQ_SOURCE); 		//Enable GPIO interrupt of TX pin
	Trigger_DistanceRX();
	HAL_Delay(100);									//wait for new data
	if(g_new_distancedata == true)					//Falling edge detected
	{
	  if(g_time_ivt <= MAX_TIME_COUNT)				//Filter wrong timer value base on data sheet
	  {
		  *distance_fl =(float)((double)g_time_ivt / 57.5);
		  retval = true;
	  }
	  g_new_distancedata = false;
	  HAL_Delay(400); 								//sensor much read each 200ms interval (datasheet)
	}
	HAL_NVIC_DisableIRQ(DISTANCE_IRQ_SOURCE);		//Disable GPIO interrupt of TX pin
	return retval;
}

eTestStatus DISTANCE_FWTest(void)
{
	uint16_t ui16distance=0;
	DISTANCE_test = RET_FAIL;
	HAL_GPIO_WritePin(DISTANCE_EN_GPIO_Port, DISTANCE_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	#if DISTANCE_TEST
	#if ENDLESS_LOOP_DYP
	while(true)
	{
		if (DISTANCE_GetData(&g_distance) == true)
		{
			DISTANCE_test = RET_OK;
			ui16distance = (uint16_t) g_distance;					//casting for quick debug
			printf("Distance to obstacle: %d \n",ui16distance);
		}
	}
	#else
	for (int count = 0; count < 10; ++count)						//max test time = 1s
	{
		if (DISTANCE_GetData(&g_distance) == true)
		{
			printf("Detected obstacles.. ");
			ui16distance = (uint16_t) g_distance;					//casting for quick debug
			printf("distance to obstacle: %d \n",ui16distance);
			DISTANCE_test = RET_OK;
			break;
		}
		else;
	}
	#endif /*End of ENDLESS_LOOP_DYP */
	#endif /*End of DISTANCE_TEST */
	return DISTANCE_test;
}
/**************************************************************************************/
/* RAK4200 LPUART FW Test - Send/Receive & Get DEVEUI */

#define RAK4200_WAKEUP			"at+set_config=device:sleep:0\r\n"
#define RAK4200_SLEEP			"at+set_config=device:sleep:1\r\n"
#define RAK4200_RESTART			"at+set_config=device:restart\r\n"
#define	RAK4200_HELP			"at+help\r\n"
#define RAK4200_GET_CONFIG		"at+get_config=lora:status\r\n"
#define RAK4200_JOIN			"at+join\r\n"
#define RAK4200_SET_DEVEUI		"at+set_config=lora:dev_eui:393331377c387234\r\n"
#define RAK4200_SET_APPEUI		"at+set_config=lora:app_eui:47454f3131323137\r\n"
#define RAK4200_SET_APPKEY		"at+set_config=lora:app_key:44464d47524f555047454f3131323137\r\n"
#define RAK4200_SET_GAP			"at+set_config=lora:send_interval:1:30\r\n" //Send interval = 60s
#define RAK4200_SET_DR0			"at+set_config=lora:dr:0\r\n" 				//Data rate = 0
#define RAK4200_SET_ADR			"at+set_config=lora:adr:1\r\n" 				//Data rate = 0
#define RAK4200_SENDTEST		"at+send=lora:1:11\r\n"						//Send 0F0A via port 1
#define RAK4200_TXTIMEOUT		10											//10ms

#if CHANGE_DEVEUI
#define INIT_COMMANDS			5
#else
#define INIT_COMMANDS			4
#endif

#define MAX_REJOIN				10
#define RAK_RESP_OK				"OK"
#define	RAK_RESP_NOTOK			"ERROR"

/* LoRa commands */
const char* LoRaInitCommands[] =
{
	RAK4200_WAKEUP,
	#if CHANGE_DEVEUI
	RAK4200_SET_DEVEUI,
	#endif
	RAK4200_SET_APPEUI,
	RAK4200_SET_APPKEY,
	RAK4200_SET_DR0,

};

typedef enum eLoRaState
{
    DEVICE_STATE_RESTORE,
    DEVICE_STATE_STARTED,
    DEVICE_STATE_JOIN,
	DEVICE_STATE_JOINED,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP,
}eLoRaState;
eLoRaState LoRa_curState = DEVICE_STATE_RESTORE;

bool g_LoRaInit = false;
uint8_t g_lora_datarecv[RX_BUF_LEN]={0};
uint8_t g_rxdmabuffer[RX_DMABUF_LEN]={0};
volatile bool g_dmaoverflow=false;
volatile bool g_dmanewdata=false;
#define TEST_DMA	1
eTestStatus LORA_FWTest(void)
{
	uint8_t lora_datarecv=0;
	uint8_t lora_dataindex=0;
	LORA_test = RET_FAIL;
	#if LORA_TEST
	HAL_Delay(1000);
	HAL_GPIO_WritePin(RAK_EN_GPIO_Port, RAK_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);												//Wait for stable
	#if DEBUG_AT_UART
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
	#else 															/* End of DEBUG_AT_UART  */
	__HAL_UART_ENABLE_IT(&hlpuart1,UART_IT_IDLE); 					/* Enable UART RX Idle interrupt */
	HAL_UART_Receive_DMA(&hlpuart1, g_rxdmabuffer, RX_DMABUF_LEN);  /* Enable receive data via DMA */
	if(g_LoRaInit == false) //Init LoRa & LoRaWan parameter in case not initialized yet
	{
		printf("Configuring RAK4200 \r\n");
		for(int command_count = 0; command_count < INIT_COMMANDS; ) /* Send devices initial commands */
		{
			//Clear buffer before receive new response
			memset(g_lora_datarecv, 0, RX_BUF_LEN);
			lora_dataindex = 0;
			//Send commands and wait for response in case of successfully transmit command
			if(HAL_UART_Transmit(&hlpuart1, (uint8_t*)LoRaInitCommands[command_count], strlen(LoRaInitCommands[command_count]), 100) == HAL_OK)
			{
				HAL_Delay(500);
				if(g_dmanewdata == true)
				{
					// Response OK
					if(strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_OK) != NULL)
					{
						command_count++;
					}
					// Response NOT OK
					else if (strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_NOTOK) != NULL)
					{
					}
					g_dmanewdata = false;
					HAL_Delay(100);
				}
			}
		}
		printf("Configure device successfully\r\n");
		LoRa_curState = DEVICE_STATE_STARTED;
		g_LoRaInit = true;
	}
	if(LoRa_curState != DEVICE_STATE_STARTED) {printf("Test LoRa failed \n"); return LORA_test;}
	//Testing Joining
	printf("Joining ... \n");
	for (int retry_count = 0; retry_count < MAX_REJOIN; ++retry_count)
	{
		memset(g_lora_datarecv, 0, RX_BUF_LEN);
		lora_dataindex = 0;
		HAL_Delay(10000);
		if (HAL_UART_Transmit(&hlpuart1, (uint8_t*) RAK4200_JOIN, strlen(RAK4200_JOIN), 100) == HAL_OK)
		{
			if(g_dmanewdata == true)
			{
				// Response OK
				if (strstr( (const char*)g_lora_datarecv, (const char*)"Join Success") != NULL)
				{
					LoRa_curState = DEVICE_STATE_JOINED;
					printf("Joined successfully\n");
					g_dmanewdata = false;
					HAL_Delay(100);
					break;
				}
				// Response NOT OK
				printf("Re-joining %d \r\n",retry_count);
				g_dmanewdata = false;
				HAL_Delay(100);
			}
		}
	}
	// Testing transfer data (up/downlink messages)
	#if TEST_DMA
	if(LoRa_curState == DEVICE_STATE_JOINED)
	{
		printf("Sending uplink\n");
		for (int retry_count = 0; retry_count < MAX_REJOIN; )
		{
			memset(g_lora_datarecv, 0, RX_BUF_LEN);
			lora_dataindex = 0;
			HAL_Delay(5000);
			if (HAL_UART_Transmit(&hlpuart1, (uint8_t*) RAK4200_SENDTEST, strlen(RAK4200_SENDTEST), 100) == HAL_OK)
			{
				if(g_dmanewdata == true)
				{
					retry_count++;
					// Response OK
					if(strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_OK) != NULL)
					{
						printf("Send uplink completed\n");
						LORA_test = true;
						g_dmanewdata = false;
						break;
					}
						printf("Re-send uplink %d \n", retry_count);
						g_dmanewdata = false;
						HAL_Delay(100);
				}
			}
		}
	}
	#else

	if(LoRa_curState == DEVICE_STATE_JOINED)
	{
		for (int retry_count = 0; retry_count < MAX_REJOIN; ++retry_count)
		{
			memset(g_lora_datarecv, 0, RX_BUF_LEN);
			lora_dataindex = 0;
			if (HAL_UART_Transmit(&hlpuart1, (uint8_t*) RAK4200_SENDTEST, strlen(RAK4200_SENDTEST), 100) == HAL_OK)
			{
				do
				{
					if( (HAL_UART_Receive_IT(&hlpuart1, &lora_datarecv, 1) == HAL_OK))
					{
							g_lora_datarecv[lora_dataindex++] = lora_datarecv;
					}
				}while( (strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_OK) == NULL));
				if( (strstr( (const char*)g_lora_datarecv, (const char*)"at+send")) != NULL)
				#if !TEST_DOWNLINK //Since this will cause losing downlink data
				{
					printf("Send uplink completed\n");
					break;
				}
				#else	//Test downlink
				{
					do
					{
						if(HAL_UART_Receive_IT(&hlpuart1, &lora_datarecv, 1) == HAL_OK)
						{
							g_lora_datarecv[lora_dataindex++] = lora_datarecv;
						}
					}

					while ( (strstr( (const char*)g_lora_datarecv, (const char*)"at+recv") == NULL));
					printf("Send uplink completed\n");
					if((strstr( (const char*)g_lora_datarecv, (const char*)"at+recv") != NULL))
					{
						printf("Receive downlink\n");
						HAL_UART_Transmit(&huart1, g_lora_datarecv, strlen(g_lora_datarecv), 100);
					}
				}
				#endif /*End of !TEST_DOWNLINK*/
			}
		}
	}
	#if TEST_SLEEPLORA
	for (int retry_count = 0; retry_count < MAX_REJOIN; ++retry_count)
	{
		memset(g_lora_datarecv, 0, RX_BUF_LEN);
		lora_dataindex = 0;
		if (HAL_UART_Transmit(&hlpuart1, (uint8_t*) RAK4200_SLEEP, strlen(RAK4200_SLEEP), 100) == HAL_OK)
		{
			do
			{
				if( (HAL_UART_Receive_IT(&hlpuart1, &lora_datarecv, 1) == HAL_OK) && (lora_datarecv !=0) )
				{
					g_lora_datarecv[lora_dataindex++] = lora_datarecv;
				}
			}while( (strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_OK) == NULL));
				printf("LoRa Sleep\n");
				break;
		}
	}
	#endif /*End of TEST_DMA*/
	#endif /*End of TEST_SLEEPLORA*/
	#endif /* End of DEBUG_AT_UART */
	#endif /* End of LORA_TEST */
	return LORA_test;
}

void UART_GetDataDMA(uint8_t* pui8buffer)
{
	static uint32_t old_pos=0;
	uint32_t pos=0;
	/* Calculate current position in buffer */
	pos = RX_DMABUF_LEN - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
	if (pos != old_pos)		/* Check change in received data */
	{
		if (pos > old_pos)	/* Current position is over previous one */
		{
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			memcpy(pui8buffer, &g_rxdmabuffer[old_pos], pos - old_pos);
		}
		else
		{
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */
			memcpy(pui8buffer, &g_rxdmabuffer[old_pos], RX_DMABUF_LEN - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos > 0)
			{
				memcpy((pui8buffer+(RX_DMABUF_LEN - old_pos)), &g_rxdmabuffer[0], pos);
			}
		}
		g_dmanewdata = true;
	}
	old_pos = pos;	/* Save current position as old */
	g_dmaoverflow = false;	/* Make sure that data never overflow twice */
}

/**************************************************************************************/
/* GPS sensor FW TEST - Read data */
#define UART_TIMEOUT		100 //in ms
#define MAX_LEN_GPS			100
uint8_t g_gps_datarecv[MAX_LEN_GPS]={0};

bool GPS_Settings(void)
{
	uint8_t data_recv=0;
	uint8_t gps_dataindex=0;
	uint8_t max_retry = 10;
	uint8_t recv_count=0;

	for (int count = 0; count < max_retry; ++count)		//Retry in case unsuccessful communication
	{
		printf("Data sent to module GPS: \n");
		if(HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_SET_NMEA_OUTPUT_GGAONLY, strlen( PMTK_SET_NMEA_OUTPUT_GGAONLY), UART_TIMEOUT) == HAL_OK)
		{
			gps_dataindex = 0;
			recv_count = 0;
			do
			{
				if (HAL_UART_Receive_IT(&huart1, &data_recv, 1) == HAL_OK)
				{
					if(data_recv == '$') 	gps_dataindex = 0;
					else
					{
						g_gps_datarecv[gps_dataindex++] = data_recv;
					}
				}
			}
			while( (g_gps_datarecv[gps_dataindex-1] != '\r') && (g_gps_datarecv[gps_dataindex] != '\n') );
			recv_count = gps_dataindex;
			if (strstr( (const char*)g_gps_datarecv, (const char*)"PMTK001,314") != NULL)
			{
				printf("Data recv GPS: \n");
				HAL_UART_Transmit(&huart1, g_gps_datarecv, recv_count, 100);
				return true;
			}
		}
	}
	return false;
}

eTestStatus GPS_FWTest(void)
{
	GPS_test = RET_FAIL;
	#if GPS_TEST
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);												//Wait for GPS supply power stable
	if (GPS_Settings() == true) GPS_test = RET_OK;
	else						GPS_test = RET_FAIL;
	HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);
	#endif /*End GPS_TEST*/
	return GPS_test;
}

/**************************************************************************************/
/* ADC FW Test */
#define BUF_SIZE	2
uint32_t g_ui32vref=0;
uint32_t g_ui32input=0;
uint32_t g_ui32ADCraw[BUF_SIZE]={0};
uint32_t g_ui32bat=0;
volatile bool g_newadcdata=false;
volatile bool g_endseq=false;
eTestStatus ADC_FWTest(void)
{
	ADC_test = RET_FAIL;
	#ifdef ADC_TEST
	HAL_GPIO_WritePin(EN_BATT_GPIO_Port, EN_BATT_Pin, GPIO_PIN_SET);
	HAL_Delay(100);			/* Wait for stable */
	while(1)
	{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_IT(&hadc1);
	while(g_newadcdata != true);
	ADC_test = RET_OK;
	HAL_ADC_Stop_IT(&hadc1);
	g_ui32vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(g_ui32ADCraw[0], ADC_RESOLUTION_12B);
	g_ui32input= __LL_ADC_CALC_DATA_TO_VOLTAGE(g_ui32vref, g_ui32ADCraw[1], ADC_RESOLUTION_12B);
	g_ui32bat = 4 * g_ui32input;
	printf("Battery voltages: %ld (mV)\n",g_ui32bat);
	g_newadcdata = false;
	#if !ENDLESS_BATT_MEASURING
	break;
	#endif /*End of ENDLESS_BATT_MEASURING*/
	}
	#endif /*End of ADC_TEST*/
	return ADC_test;
}

/**************************************************************************************/
volatile bool g_buttonpressed=false;
volatile uint16_t g_countbuttonpress=0;
void ButtonsHandler(void)
{
	if (g_buttonpressed == true)
	{
		if(HAL_GPIO_ReadPin(SW_DIS_GPIO_Port, SW_DIS_Pin) == GPIO_PIN_RESET) //Release
		{
			if(g_countbuttonpress <100)			//not stable press
			{

			}
			else
			{
				printf("Button pressed \n");
			}
			g_buttonpressed = false;
			g_countbuttonpress = 0;
		}
	}
}

/**************************************************************************************/
/* Additional functions for testing */

void DebugProbeInit(void)
{
	GPIO_InitTypeDef GPIOA_InitStructure;
	GPIOA_InitStructure.Pin = PROBE1 | PROBE2;
	GPIOA_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOA_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PROBE_PORT, &GPIOA_InitStructure);
	HAL_GPIO_WritePin(PROBE_PORT, PROBE1 | PROBE2, GPIO_PIN_RESET);
}

void EnterStopMode( void)
{
	uint32_t DeInitpinsPortA=GPIO_PIN_All;
	uint32_t DeInitpinsPortB=GPIO_PIN_All;
	printf("Entering stop 2 mode...\n");

	HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);

	#if TEST_SLEEPING
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	//Port B
	DeInitpinsPortB &=~(DISTANCE_EN_Pin);
	GPIO_InitStructure.Pin = DeInitpinsPortB;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Port A
	DeInitpinsPortA &= ~(WK_ACL_Pin| EEPROM_EN_Pin| GPS_EN_Pin| EN_BATT_Pin| RAK_EN_Pin);
	GPIO_InitStructure.Pin = DeInitpinsPortA;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Test 1/////////////////////////////////////////////////
	#if 0
	// Reusult: 11.8uA
	LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
	__HAL_RCC_ADC_CLK_DISABLE();
	__HAL_RCC_C2USB_CLK_DISABLE();
	__HAL_RCC_USB_CLK_DISABLE();
	__HAL_PWR_VDDUSB_DISABLE();
	LL_VREFBUF_Disable();
	HAL_PWR_DisablePVD();
	HAL_PWR_DisableBkUpAccess();
	HAL_PWREx_DisableVddUSB();
	LL_RCC_LSI1_Disable();
	LL_RCC_LSI2_Disable();
	LL_RCC_LSE_DisableCSS();
	LL_RCC_LSE_Disable();
	PWR->CR1 &= ~(PWR_CR1_LPR);
	LL_PWR_SetBORConfig(LL_PWR_BOR_SMPS_FORCE_BYPASS);
	LL_PWR_SMPS_SetMode(LL_PWR_SMPS_BYPASS);
	#endif
	///////////////////////////////////////////////////////////

	//Test 2/////////////////////////////////////////////////
	#if 0
	// Reusult: 11.8uA
	LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
	__HAL_RCC_ADC_CLK_DISABLE();
	__HAL_RCC_LPUART1_CLK_DISABLE();

	LL_VREFBUF_Disable();

	HAL_PWR_DisableBkUpAccess();
	HAL_PWR_DisablePVD();


	#if !NEW
	__HAL_RCC_LCD_CLK_DISABLE();
	__HAL_RCC_LPTIM1_CLK_DISABLE();
	__HAL_RCC_I2C3_CLK_DISABLE();

	#endif /*End of var*/
	#endif
	///////////////////////////////////////////////////////////

	//Test 3/////////////////////////////////////////////////
	// Result: 11.8uA
	LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
	__HAL_RCC_ADC_CLK_DISABLE();
	__HAL_RCC_LPUART1_CLK_DISABLE();

	LL_VREFBUF_Disable();
	HAL_PWR_DisablePVD();
	///////////////////////////////////////////////////////////

	#endif /*End of TEST_SLEEPING*/

	// Module control pins -> low output
	HAL_GPIO_WritePin(EN_BATT_GPIO_Port, EN_BATT_Pin, GPIO_PIN_RESET);   /* Turn off Batt */
	HAL_GPIO_WritePin(EEPROM_EN_GPIO_Port, EEPROM_EN_Pin, GPIO_PIN_RESET); /* Turn off EEPROM */
	HAL_GPIO_WritePin(DISTANCE_EN_GPIO_Port, DISTANCE_EN_Pin, GPIO_PIN_RESET);	/* Turn off Distance */
	HAL_GPIO_WritePin(RAK_EN_GPIO_Port, RAK_EN_Pin, GPIO_PIN_RESET);	/* Turn off RAk4200 */
	HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);	/* Turn off GPS */

    // Stop SYSTICK Timer
    HAL_SuspendTick();

    // Enter Stop Mode
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    //Wake up
    SystemClock_Config();

    // Resume SYSTICK Timer
    HAL_ResumeTick();
    printf("Waked up from stop 2 \n");

}

HAL_StatusTypeDef I2C_Transferring(tI2CPackage *I2CPackage_T)
{
	HAL_StatusTypeDef result;
	if (I2CPackage_T->I2CMode_T == I2C_W || I2CPackage_T->I2CMode_T == I2C_WW) {
		result = HAL_I2C_Mem_Write(&hi2c1, I2CPackage_T->DeviceAddress_U16,
				I2CPackage_T->Register_16,
				I2C_MEMADD_SIZE_8BIT, I2CPackage_T->Data_U8P,
				I2CPackage_T->Length_U16, 0x1000U);
	} else if (I2CPackage_T->I2CMode_T == I2C_WW
			|| I2CPackage_T->I2CMode_T == I2C_WR) {
		result = HAL_I2C_Mem_Read(&hi2c1, I2CPackage_T->DeviceAddress_U16,
				I2CPackage_T->Register_16,
				I2C_MEMADD_SIZE_8BIT, I2CPackage_T->Data_U8P,
				I2CPackage_T->Length_U16, 0x1000U);
	}
	if (result != HAL_OK) {
		//ErrorHander(result);
	}
	return result;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	 if(LL_ADC_IsActiveFlag_EOS(ADC1) != 0) /* EOS event */
	 {
		 g_ui32ADCraw[1] = HAL_ADC_GetValue(&hadc1);
		 g_newadcdata = true;
	 }
	 else									/* EOC event */
	 {
		 g_ui32ADCraw[0] = HAL_ADC_GetValue(&hadc1);
	 }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static bool rising_dectected= false;
	switch (GPIO_Pin)
	{
		case PWM_CABLE_Pin:
			if(HAL_GPIO_ReadPin(PWM_CABLE_GPIO_Port, PWM_CABLE_Pin) == GPIO_PIN_SET)		//Rising edge
			{
				__HAL_TIM_SET_COUNTER(&htim16, 0); //Reset timer value
				HAL_TIM_Base_Start(&htim16);
				rising_dectected = true;
			}
			else																			//Falling edge
			{
				if(rising_dectected == true)
				{
					g_time_ivt = __HAL_TIM_GET_COUNTER(&htim16);		//Get time elapse
					HAL_TIM_Base_Stop(&htim16);							//Stop timer
					g_new_distancedata = true;
				}
				rising_dectected = false;
			}
			break;

		case WK_ACL_Pin: //ACL wake up handler
			g_acl_interrupt = true;
			ACL_ReadSource();
			ACL_EnableInterrupt();
			break;

		case SW_DIS_Pin:
			g_buttonpressed = true;
			if(g_testingble == true)
			{
				UTIL_SEQ_SetTask(1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
			}
			break;
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	g_rtcwakeup = true;
}
/**************************************************************************************/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
