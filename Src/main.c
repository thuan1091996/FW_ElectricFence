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
#define DEBUG_UART			USED
#define DEBUG_AT_UART		NOT_USED
#define FW_TEST				USED
#define EEPROM_TEST 		USED
#define	ACL_TEST			USED
#define	DISTANCE_TEST		USED
#define GPS_TEST			USED
#define LORA_TEST			USED
#define CHANGE_DEVEUI		USED
#define TEST_DOWNLINK		NOT_USED
#define FW1_TEST 			USED
#define TEST_ITV			1
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
volatile bool g_rtcwakeup=true;
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

static bool DISTANCE_GetData(float *distance_fl);
void ConvertData(uint8_t* p_ui8buffer, uint16_t ui16data);
void Alarm_Init(void);
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
  /* USER CODE BEGIN 2 */
//  Sys_Test();
  FW_Test1();

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
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_LSE;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
	printf("Testing GPS ...\n");
	if(GPS_FWTest() == RET_OK)		printf("\nFW Test GPS: OK \n");
	else							printf("FW Test GPS: Not OK \n");
//	ADC_FWTest();
//	BLE_FWTest();
//	SYS_test = 	EEPROM_test & ACL_test & DISTANCE_test & LORA_test & GPS_test & ADC_test; //ADC not available
	SYS_test = 	EEPROM_test & ACL_test & DISTANCE_test & LORA_test & GPS_test; //ADC not available
	if(SYS_test == RET_OK)			printf("\nFW Test: OK \n");
	else							printf("FW Test: Not OK \n");
	EnterStopMode();
	if(g_acl_interrupt == true)
	{
		printf("ACL detected motion \n");
		g_acl_interrupt = false;
	}
	else printf("No ACL interrupt detected \n");
	return SYS_test;
}
uint8_t distance_str[10]={0};
eTestStatus FW_Test1(void)
{
	eTestStatus fw1_teststatus = RET_FAIL;
	float distance_data=0.0;
	uint16_t ui16distance=0;
	printf("FW1 Test started... \n");
	printf("Testing LoRa ...\n");
	LORA_FWTest();
	Alarm_Init();
	while(1)
	{
		if(g_rtcwakeup == true)				/* Make sure wake up by RTC */
		{
			if(DISTANCE_GetData(&distance_data) == true)
			{
				memset(distance_str, 0, 10);
				ui16distance = (uint16_t) distance_data;					//casting for quick debug
				printf("Distance to obstacle: %d \n",ui16distance);
				ConvertData(distance_str, ui16distance);
				strcat((char*)distance_str,"\r\n");
			}
			LORA_FWTest();					/* Send uplink */
			g_rtcwakeup = false;
			EnterStopMode();
		}
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
#define RAK4200_SENDTEST		"at+send=lora:1:11\r\n"						//Send 0F0A via port 1
#define RAK4200_TXTIMEOUT		10											//10ms

#define RAK_DATALEN				100
#define MAX_REJOIN				10

#if CHANGE_DEVEUI
#define INIT_COMMANDS			5
#else
#define INIT_COMMANDS			4
#endif



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
	RAK4200_SET_DR0
	//RAK4200_SET_GAP
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
uint8_t g_lora_datarecv[RAK_DATALEN]={0};

eTestStatus LORA_FWTest(void)
{
	uint8_t lora_datarecv=0;
	uint8_t lora_dataindex=0;
	uint8_t uplink_data[21]="at+send=lora:1:";
	LORA_test = RET_FAIL;
	#if LORA_TEST
	#if DEBUG_AT_UART
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
	#else /* End of DEBUG_AT_UART  */
	if(g_LoRaInit == false) //Init LoRa & LoRaWan parameter in case not initialized yet
	{
//		__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE); 		/* Enable UART IDLE interrupt */
		HAL_GPIO_WritePin(RAK_EN_GPIO_Port, RAK_EN_Pin, GPIO_PIN_SET);
		HAL_Delay(3000);									/* Wait for stable */
		for(int command_count = 0; command_count < INIT_COMMANDS; ++command_count) //Send devices initial commands
		{
			//Clear buffer before receive new response
			memset(g_lora_datarecv, 0, RAK_DATALEN);
			lora_dataindex = 0;
			//Send commands and wait for response in case of successfully transmit command
			if(HAL_UART_Transmit(&hlpuart1, (uint8_t*)LoRaInitCommands[command_count], strlen(LoRaInitCommands[command_count]), 100) == HAL_OK)
			{
				do
				{
					if( (HAL_UART_Receive_IT(&hlpuart1, &lora_datarecv, 1) == HAL_OK) && (lora_datarecv !=0) )
					{
						g_lora_datarecv[lora_dataindex++] = lora_datarecv;
					}
				}
				while ( (strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_OK) == NULL) &&
						(strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_NOTOK) == NULL) );
			}
			strcat((char*)g_lora_datarecv, "\r\n");
			HAL_UART_Transmit(&huart1, g_lora_datarecv, strlen((const char*) g_lora_datarecv), 100);
		}
		LoRa_curState = DEVICE_STATE_STARTED;
		//Testing Joining
		printf("Joining ... \n");
		for (int retry_count = 0; retry_count < MAX_REJOIN; ++retry_count)
		{
			memset(g_lora_datarecv, 0, RAK_DATALEN);
			lora_dataindex = 0;
			if (HAL_UART_Transmit(&hlpuart1, (uint8_t*) RAK4200_JOIN, strlen(RAK4200_JOIN), 100) == HAL_OK)
			{
				do
				{
					if( (HAL_UART_Receive_IT(&hlpuart1, &lora_datarecv, 1) == HAL_OK) && (lora_datarecv !=0) )
					{
						g_lora_datarecv[lora_dataindex++] = lora_datarecv;
					}
				}while( (strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_OK) == NULL));
				if( (strstr( (const char*)g_lora_datarecv, (const char*)"Join Success") != NULL) )
				{
					LoRa_curState = DEVICE_STATE_JOINED;
					printf("Joined successfully\n");
					HAL_Delay(100);
					break;
				}
			}
		}
		g_LoRaInit = true;
	}
	else
	{
		// Testing transfer data (up/downlink messages)
		if(LoRa_curState == DEVICE_STATE_JOINED)
		{
			for (int retry_count = 0; retry_count < MAX_REJOIN; ++retry_count)
			{
				memset(g_lora_datarecv, 0, RAK_DATALEN); //TODO
				lora_dataindex = 0;
				#if FW1_TEST
				strcat((char*)uplink_data, (char*)distance_str);
				if (HAL_UART_Transmit(&hlpuart1, (uint8_t*) uplink_data, strlen(uplink_data), 200) == HAL_OK)
				#else
				if (HAL_UART_Transmit(&hlpuart1, (uint8_t*) RAK4200_SENDTEST, strlen(RAK4200_SENDTEST), 100) == HAL_OK)
				#endif /*FW1_TEST*/
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
						LORA_test = RET_OK;
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
			/////////////////////////////// Sleeping //////////////////////////////////////////////
			for (int retry_count = 0; retry_count < MAX_REJOIN; ++retry_count)
			{
				memset(g_lora_datarecv, 0, RAK_DATALEN);
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
			/////////////////////////////// Sleeping //////////////////////////////////////////////
		}
	}
	#endif /* End of DEBUG_AT_UART */
	#endif /* End of LORA_TEST */
	return LORA_test;
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
	HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);												//Wait for GPS supply power stable
	if (GPS_Settings() == true) GPS_test = RET_OK;
	else						GPS_test = RET_FAIL;
	HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);
	#endif /*End GPS_TEST*/
	return GPS_test;
}

/**************************************************************************************/
/* GPS sensor FW TEST - Read data */
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

void EnterStopMode( void)
{
	printf("Entering stop 2 mode...\n");
    //GPIO Deinit if needed

	//Place CPU2 in Shutdown mode
	LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);

	//
	LL_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);
	LL_C2_EXTI_DisableIT_32_63(LL_EXTI_LINE_48);


    //Debug in stop mode
//    HAL_DBGMCU_EnableDBGStopMode();

    // Stop SYSTICK Timer
    HAL_SuspendTick();

    // Enter Stop Mode
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

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
			break;

		case SW_DIS_Pin:
			UTIL_SEQ_SetTask(1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
			break;
	}
}

void Alarm_Init(void)
{
	RTC_AlarmTypeDef sAlarm = {0};
	RTC_TimeTypeDef curTime = {0};
	RTC_DateTypeDef curDate = {0};
	HAL_RTC_GetDate(&hrtc, &curDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &curTime, RTC_FORMAT_BIN);
	sAlarm.AlarmTime.Hours = curTime.Hours;
	sAlarm.AlarmTime.Minutes = curTime.Minutes + TEST_ITV;
	sAlarm.AlarmTime.Seconds = curTime.Seconds;
	sAlarm.AlarmTime.SubSeconds = 0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
	  Error_Handler();
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	RTC_AlarmTypeDef cur_alarm;
	HAL_RTC_GetAlarm(hrtc, &cur_alarm, RTC_ALARM_A, FORMAT_BIN);
	cur_alarm.AlarmTime.Minutes +=TEST_ITV;
	cur_alarm.AlarmTime.Minutes %=60;
	HAL_RTC_DeactivateAlarm(hrtc, RTC_ALARM_A);
	HAL_RTC_SetAlarm_IT(hrtc, &cur_alarm, FORMAT_BIN);
	g_rtcwakeup = true;
}

/* ---------Convert_2Char(uint8_t numb_in)--------------
 * Operating: Convert number to character
 * Input:  Number want to convert to char
 * Output: corresponding char character
 * 0 - 9 -> '0' -> '9'
 * 0x0A  ->0x0F return 'A'->'F'
-------------------------------------------------------*/
char Convert_2Char(uint8_t numb_in){
	char char_out;
    if(numb_in<0x0A)  char_out=numb_in+0x30; //0-9 return '0'-'9'
    else              char_out=numb_in+55;   //0x0A-0x0F return 'A'-'F'
    return char_out;
}

void ConvertData(uint8_t* p_ui8buffer, uint16_t ui16data)
{
	uint16_t nibble;
	for (int count = 0; count < 4; ++count)
	{
		nibble = ui16data & 0x0F;
		ui16data >>= 4;
		p_ui8buffer[3-count] = Convert_2Char(nibble);
	}
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
