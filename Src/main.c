/* USER CODE BEGIN Header */
/****************************************************************************
 * Title                 :   DFM - Electrical fence firmware test
 * ProductLink           :	https://docs.google.com/spreadsheets/d/163NVGYAAz6Q9rcFRIo8FAkWSXMXqbl9w7QIAKOVKirg/edit#gid=984419045
 * Filename              :	main.c
 * Author                :   ItachiThuan
 * Origin Date           :	May 21, 2021
 * Version               :   1.0.0
 * Target                :   STM32WB55 with STM32CubeIDE
 * Notes                 :
 *****************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "rf.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"
#include "spi.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_seq.h"
#include "string.h"
#include "stdbool.h"
#include "AT0x.h"
#include "MMA865x.h"
#include "L80.h"
#include "AT25SF321x.h"
#include "PCF8574A.h"
#include "MC36xx.h"
#include "custom_stm.h"
#include "custom_app.h"
#include "st25dv.h"
#include "stdio.h"
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
eTestStatus SYS_test, EEPROM_test, ACL_test, LORA_test,
			GPS_test, ADC_test, BLE_test, BUTTON_test, ADCElecFence_Test, FLASH_test, EXT_IO_test, NFC_test;

enum Device_pos {ACL_pos=0, ADC_pos, GPS_pos, LORA_pos, EXT_IO_pos, FLASH_pos, NFC_pos, MAX_pos};
typedef enum Device_pos eDevicePos;

typedef struct
{
	eTestStatus *p_result;
	char		*p_name;
	eDevicePos	pos;
}Test_result_t;

Test_result_t el_fence_test[] = {
		{.p_result =&ACL_test, 		.p_name="ACL",		.pos = ACL_pos},
		{.p_result =&GPS_test, 		.p_name="GPS",		.pos = GPS_pos},
		{.p_result =&LORA_test, 	.p_name="LORA",		.pos = LORA_pos},
		{.p_result =&EXT_IO_test, 	.p_name="EXT_IO",	.pos = EXT_IO_pos},
		{.p_result =&FLASH_test, 	.p_name="FLASH", 	.pos = FLASH_pos},
		{.p_result =&NFC_test,	 	.p_name="NFC", 		.pos = NFC_pos},
};


/* I2C Transfer */
typedef enum {
	I2C_W  = 1,
	I2C_R  = 2,
	I2C_WR = 4,
	I2C_WW = 8,
	I2C_DR = 0x10,		/*Direct read*/
	I2C_DW = 0x20		/*Direct write*/
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if DEBUG_CONSOLE
#if	DEBUG_ITM
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	for (int i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}
#elif DEBUG_UART
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

#endif  /* End of DEBUG_UART */
#endif  /* End of DEBUG_CONSOLE */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool  g_nfc_init = false;
volatile bool g_button1pressed=false;
volatile bool g_button2pressed=false;
volatile uint16_t g_countbuttonpress=0;
volatile bool g_nfc_interrupt=false;
volatile bool g_acl_interrupt=false;
volatile bool g_button_interrupt=false;
volatile bool g_rak4200_newdata=false;
volatile bool g_rtcwakeup=false;
volatile bool g_testingble=false;
uint32_t g_max_hv=0;
volatile uint32_t g_pos_max=0;
volatile uint32_t g_neg_max=0;
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
eTestStatus LORA_FWTest(void);
eTestStatus GPS_FWTest(void);
eTestStatus ADC_FWTest(void);
eTestStatus BLE_FWTest(void);
eTestStatus Flash_FWTest(void);
eTestStatus EXT_IO_FWTest(void);
eTestStatus NFC_FWTest(void);
/* Find the max ADC value in both ADC channels using DMA */
eTestStatus ADC_ElecFenceTest(void);

/* New option use interrupt instead of DMA to find Max ADC of each channel*/
eTestStatus ADC_ElecFenceTestInt(void);


eTestStatus HV_Monitor(void);
void ButtonsHandler(void);
void EnterStopMode(void);
void DebugProbeInit(void);
#endif /*End of FW_TEST*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**************************************************************************************/
/* Main FW Test */
eTestStatus Sys_Test(void)
{
	SYS_test = RET_FAIL;

	printf("Testing extended I/O ... \n");
	if(EXT_IO_FWTest() == RET_OK)	printf("FW Test Extended I/O: OK \n");
	else							printf("FW Test Extended I/O: Not OK \n");

	printf("Testing ACL ...\n");
	#if 1
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	if(MC36xx_Init() == RET_OK )	{ ACL_test = RET_OK; printf("FW Test ACL: OK \n");}
	#else
	if(ACL_FWTest() == RET_OK )		printf("FW Test ACL: OK \n");
	#endif  /* End of 0 */
	else							printf("FW Test ACL: Not OK \n");

	if( HAL_NVIC_GetPendingIRQ(EXTI0_IRQn) == true)
	{
		HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
		NVIC->ICPR[0] |= (1<<6);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	}
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	printf("Testing EEPROM ...\n");
	if(EEPROM_FWTest() == RET_OK )	printf("FW Test EEPROM: OK \n");
	else							printf("FW Test EEPROM: Not OK \n");

	printf("Testing LoRa ...\n");
	if(LORA_FWTest() == RET_OK)	 	printf("FW Test LoRa: OK \n");
	else							printf("FW Test LoRa: Not OK \n");

	printf("Testing ADC ...\n");
	if(ADC_FWTest() == RET_OK)	 	printf("FW Test ADC: OK \n");
	else							printf("FW Test ADC: Not OK \n");

	printf("Testing GPS ...\n");
	if(GPS_FWTest() == RET_OK)		printf("FW Test GPS: OK \n");
	else							printf("FW Test GPS: Not OK \n");

	printf("Testing Flash ...\n");
	if(Flash_FWTest() == RET_OK)	printf("FW Test Flash: OK \n");
	else							printf("FW Test Flash: Not OK \n");

	printf("Testing NFC ...\n");
	if(NFC_FWTest() == RET_OK)		printf("FW Test NFC: OK \n");
	else							printf("FW Test NFC: Not OK \n");


	/* Test result |	!Perform		 == RESULT	(1 if test pass & not skip test)
	 * 		0				0				0
	 * 		0				1				1
	 * 		1				0				1
	 * 		1				1				1*/


	SYS_test = ((ACL_test 		| 	(!ACL_TEST)		)	&
				(LORA_test 		| 	(!LORA_TEST)	)	&
				(GPS_test  		| 	(!GPS_TEST)		)	&
				(ADC_test 		| 	(!ADC_TEST)		)	&
				(FLASH_test 	| 	(!FLASH_TEST)	)   &
				(EXT_IO_test 	|	(!EXT_IO_TEST)	));


	if(SYS_test == RET_OK)			printf("FW Test: OK \n");
	else							printf("FW Test: Not OK \n");

	printf("----------------Test results ----------------\r\n");
	for(uint8_t idx=0; idx<MAX_pos-1; idx++)
	{
		printf("%s: %d \r\n", (el_fence_test[idx].p_name), *(el_fence_test[idx].p_result));
	}
	printf("---------------------------------------------\r\n");
	#if DEV_SLEEP
	EnterStopMode();
	HAL_UART_MspInit(&huart1);
	printf("Waked up from stop 2 \n");
	HAL_Delay(100);
	ButtonsHandler();
	if(g_acl_interrupt == true)
	{
		printf("ACL detected motion \n");
		g_acl_interrupt = false;
	}
	else if (g_button1pressed == true)
	{
		printf("Wake up by SW1 \n");
	}

	else if (g_button2pressed == true)
	{
		printf("Wake up by SW2 \n");
	}

	else if(g_nfc_interrupt == true)
	{
		printf("Wake up by NFC \n");
	}
	else
	{
		printf("Wake up from no where \n");
	}
	HAL_Delay(100);
	#endif  /* End of DEV_SLEEP */
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
#define START_ADDR		900

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

	EEPROM_test = RET_FAIL;
#if EEPROM_TEST
	uint8_t uid[32]={0};
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

#else
	printf("----- Skipped test ----- \n");

#endif /*End of EEPROM_TEST*/
	return EEPROM_test;
}

/**************************************************************************************/
/* ACL FIRMWARE TEST - Read raw data & Interrupt settings */

static uint8_t CtrlReg4 = 0;
static uint8_t CtrlReg5 = 0;
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


static void ACL_DisableIntRegister(uint8_t Source_U8)
{
	CtrlReg4 &= ~Source_U8;
	ACL_RegisterSet(MMA865X_CTRL_REG4, CtrlReg4);

	CtrlReg5 &= ~Source_U8;
	ACL_RegisterSet(MMA865X_CTRL_REG5, CtrlReg5);
}

/**
 * DisableLndPrlInt
 *
 * Disable landscape-portrait orientation interrupt
 */
static void ACL_DisableLndPrtInt()
{
	// Enter stand-by mode
	ACL_Standby();
	HAL_Delay(1);

	ACL_DisableIntRegister(SRC_LNDPRT_MASK);
	HAL_Delay(1);

	// Disable Portrait/Ladscape orientation detection
	ACL_RegisterSet(MMA865X_PL_CFG, 0);
	HAL_Delay(1);

	// Back to active mode
	ACL_Active();
}

static void ACL_Sleeping(void* Def)
{
	if (ACL_RegisterGet(MMA865X_CTRL_REG4) & SRC_LNDPRT_MASK)
	{
	    // Disable Landscape/Portrait orientation interrupt, which is incompatible
	    // With the current sleep mode
		ACL_DisableLndPrtInt();
	    HAL_Delay(1);
	}

	// Re-enter active mode first
	ACL_Active();
	HAL_Delay(2);

	// Enter stand-by mode
	ACL_Standby();
	HAL_Delay(2);

	ACL_RegisterSet(MMA865X_ASLP_COUNT, 1);
	HAL_Delay(2);

	ACL_RegisterSet(MMA865X_CTRL_REG2, 0x1C);
	HAL_Delay(1);

	// Back to active mode
	ACL_Active();
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

#if DISABLE_ACL_IRQ
	ACL_Standby();	/* Prevent wake up from ACL */
#else
	ACL_Sleeping(NULL);
#endif /*End of DISABLE_ACL_IRQ*/
#else
	printf("----- Skipped test ----- \n");
#endif /*End of ACL_Test*/
	return ACL_test;
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
#define MAX_RESEND_CMD			5
#define MAX_TIMEOUT_RECV		2000 /*2s*/
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

bool LoRa_SendCMD(uint8_t* p_cmd, uint16_t len, uint16_t timeout)
{
	for (int retry_count = 0; retry_count < MAX_RESEND_CMD; retry_count++)
	{
		//Clear buffer before receive new response
		memset(g_lora_datarecv, 0, RX_BUF_LEN);
		//Send commands and wait for response in case of successfully transmit command
		if (HAL_UART_Transmit(&hlpuart1, p_cmd, len, 100) == HAL_OK)
		{
			printf("Sent successfully command: %s",p_cmd);
			HAL_Delay(timeout);
			if(g_dmanewdata == true)
			{
				if(strstr( (const char*)g_lora_datarecv, (const char*)RAK_RESP_OK) != NULL)
				{
					printf("OK \n");
					g_dmanewdata = false;
					return true;
				}
				g_dmanewdata = false;
				printf("Retry %d \n", retry_count + 1);
			}
			else
			{
				printf("No received response after %d ms\n", timeout);
			}
		}
		else
		{
			printf("UART transmitted failed \n");
		}
	}
	return false;
}

eTestStatus LORA_FWTest(void)
{
	LORA_test = RET_FAIL;
	#if LORA_TEST
	HAL_GPIO_WritePin(RAK_EN_GPIO_Port, RAK_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);												//Wait for stable
	__HAL_UART_ENABLE_IT(&hlpuart1,UART_IT_IDLE); 					/* Enable UART RX Idle interrupt */
	HAL_UART_Receive_DMA(&hlpuart1, g_rxdmabuffer, RX_DMABUF_LEN);  /* Enable receive data via DMA */

	#if DEBUG_AT_UART
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
	#else
	if(g_LoRaInit == false)
	{
		//Init LoRa & LoRaWan parameter in case not initialized yet
		printf("Configuring RAK4200 \r\n");
		for(int command_count = 0; command_count < INIT_COMMANDS; command_count++) /* Send devices initial commands */
		{
			if (LoRa_SendCMD((uint8_t*)LoRaInitCommands[command_count], strlen(LoRaInitCommands[command_count]), MAX_TIMEOUT_RECV) != true)
			{
				printf("Configure device failed \r\n");
				return RET_FAIL;
			}
		}
	}
	printf("Configure device successfully\r\n");
	LoRa_curState = DEVICE_STATE_STARTED;
	g_LoRaInit = true;

	#if LORAWAN_TEST
	printf("Joining ... \n");
	if( LoRa_SendCMD((uint8_t*)RAK4200_JOIN, strlen(RAK4200_JOIN), MAX_TIMEOUT_RECV*5) != true)
	{
		printf("Join failed \n");
		return RET_FAIL;
	}
	LoRa_curState = DEVICE_STATE_JOINED;
	printf("Joined successfully \n");

	printf("Sending uplink\n");
	if( LoRa_SendCMD((uint8_t*)RAK4200_SENDTEST, strlen(RAK4200_SENDTEST), MAX_TIMEOUT_RECV*6) != true)
	{
		printf("Send up-link failed\n");
		return RET_FAIL;
	}
	printf("Sent up-link successfully\n");
	LORA_test =  RET_OK;
	#endif  /* End of LORAWAN_TEST */

	#endif	/* End of DEBUG_AT_UART  */
	#else
	printf("----- Skipped test ----- \n");
	#endif /* End of LORA_TEST */
	#if TEST_SLEEPLORA
	if( LoRa_SendCMD((uint8_t*)RAK4200_SLEEP, strlen(RAK4200_SLEEP), MAX_TIMEOUT_RECV) != true)
	{
		printf("Failed to put RAK4200 into sleep mode \n");
		return RET_FAIL;
	}
	else
	{
		printf("RAK4200 was slept\n");
		return RET_OK;
	}
	#endif /*End of TEST_SLEEPLORA*/
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
	uint8_t max_retry = 3;
	uint8_t expected_respond[]="PMTK001,314";
	uint8_t respond_len=sizeof(expected_respond)-1;
	for (int count = 0; count < max_retry; ++count)		//Retry in case unsuccessful communication
	{
		uint32_t cur_time = HAL_GetTick();
		printf("Data sent to module GPS: \n");
		if(HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_SET_NMEA_OUTPUT_GGAONLY, strlen( PMTK_SET_NMEA_OUTPUT_GGAONLY), UART_TIMEOUT) == HAL_OK)
		{
			gps_dataindex = 0;
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
			while( (g_gps_datarecv[gps_dataindex-1] != '\r') && (g_gps_datarecv[gps_dataindex] != '\n')
					&& (HAL_GetTick() <= cur_time + 3000) );

			if ( memcmp(g_gps_datarecv, expected_respond, respond_len) == 0)
			{
				#if DEBUG_CONSOLE
				printf("Data recv GPS:  %s \n", g_gps_datarecv);
				#else
				HAL_UART_Transmit(&huart1, g_gps_datarecv, gps_dataindex, 100);
				#endif  /* End of DEBUG_CONSOLE */
				return true;
			}
		}
		HAL_UART_DeInit(&huart1);
		MX_USART1_UART_Init();
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
//	HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);
#else
	printf("----- Skipped test ----- \n");
#endif /*End GPS_TEST*/
	return GPS_test;
}

/**************************************************************************************/

/********************************* Flash FW Test **************************************/
static int8_t SPI_TransmitReceive(void* TxData_P,void* RxData_P, uint32_t Length_U32)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_StatusTypeDef comm_status = HAL_SPI_TransmitReceive(&hspi1, TxData_P, RxData_P, Length_U32, 1000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	if (comm_status != HAL_OK)
			return -1;
		else
			return 0;
}

static int8_t SPI_Transmit(void* TxData_P, uint32_t Length_U32)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_StatusTypeDef comm_status = HAL_SPI_Transmit(&hspi1, TxData_P, Length_U32, 1000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	if (comm_status != HAL_OK)
		return -1;
	else
		return 0;
}

static int8_t SPI_Receive(void* RxData_P, uint32_t Length_U32)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_StatusTypeDef comm_status = HAL_SPI_Receive(&hspi1, RxData_P, Length_U32, 1000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	if (comm_status != HAL_OK)
			return -1;
		else
			return 0;
}

static int8_t AT25SF321X_ReadStatusReg(uint8_t OpcodeReg, tStatusRegister* RegStatus)
{
	int8_t Result_U8 = -1;
	uint8_t buff[2] = {OpcodeReg, 1};

	Result_U8 = SPI_TransmitReceive(buff, buff, sizeof(buff));
	memcpy(RegStatus, &buff[1], 1);

	return Result_U8;
}


/* A block of 4 32 or 64 can be erase by call this function.
 * Attention: WEL bit in status register 1 must be set
 *
 * @param OpCode: Ref CMD_BLOCK_ERASE_xx, in header file
 *		  CheckProgress: pointer function callback to determine progress finished o not
 *		  		   CheckProgress: return 1 in check progressing,
 *		  		   				  0 complete stop polling
 *
 * @return  if executed successfully return 0 if done, 1 if on progeess, somthing else return error
 */

static int8_t PollingReadStatusRegister(uint8_t Opcode, tStatusRegister* Register, int8_t (*CheckProgress)(tStatusRegister* Register_TP), uint16_t Timeout)
{
	int8_t Result_S8 = -1;
	tStatusRegister StatusRegister_T = {0};

	do
	{
		Result_S8 = AT25SF321X_ReadStatusReg(Opcode, &StatusRegister_T);
	}
	while (!Result_S8 && CheckProgress(&StatusRegister_T) );

	if (Register)
		*Register = StatusRegister_T;

	return Result_S8;
}

/**
 * @Description
 * This function allows 1 to 256B of data can be programmed/write in to previously erased memory location.
 * If data is more than 256 byte, Only the last 256Byte is programmed . Read to to more detail.
 * Ref Docs
 *
 * @param  Address in range 0 - 255
 * @param  Data: pointer to memory data write
 * @param  Length: of data write and return back number byte is programmed
 * @return execute success return length of data programmed , else return error
 */
static int16_t AT25SF321X_WriteBytePage(tMemoryData MemoryData)
{
	int16_t  Result_S16 = -1;
	uint16_t Length = 0;
	uint8_t  Instruction[] = {
								CMD_BYTEPAGE_PROGRAM,
								MemoryData.Address_U32 >> 16,
								MemoryData.Address_U32 >> 8,
								MemoryData.Address_U32 >> 0,
							 };

	assert_param(!(MemoryData.Address_U32 & 0xFF));

	if (((MemoryData.Address_U32 & (cBLOCK_256B - 1)) + MemoryData.Length_U32) > cBLOCK_256B)
		Length = cBLOCK_256B - (MemoryData.Address_U32 & (cBLOCK_256B - 1));
	else
		Length = MemoryData.Length_U32;


	uint8_t MemAllocated[sizeof(Instruction) + Length];
	memcpy(MemAllocated, Instruction, sizeof(Instruction));
	memcpy(MemAllocated + sizeof(Instruction), MemoryData.Data_U8P, Length);

	Result_S16 = SPI_Transmit(MemAllocated, sizeof(Instruction) + Length);

	int8_t CheckProgress(tStatusRegister* Status) { return Status->Reg1_T.ReadyFlag_B;};	//0 Device readly, stop polling

	uint16_t T_bpp = (Length % cBLOCK_256B) ? ((Length / cBLOCK_256B) * T_pp) :	(T_bp1 + (T_bp2 * (Length - 1)));

	//Recommended: The status register be polled to determine if device has finished execution
	Result_S16 = PollingReadStatusRegister(OPC_STATUS_REG_1, NULL, &CheckProgress, T_bpp);

	return Result_S16 ? Result_S16 : Length;
}


static int8_t AT25SF321X_ByteStandradRead(tMemoryData MemoryData)
{
	int8_t  Result_S8 = -1;
	uint8_t Instruction[] = {
								MOD_STANDARD_READ,
								MemoryData.Address_U32 >> 16,
								MemoryData.Address_U32 >> 8,
								MemoryData.Address_U32 >> 0,
#if MOD_STANDARD_READ == CMD_FAST_READ
								0					// Dummy
#endif
							};

	uint8_t MemAllocated[sizeof(Instruction) + MemoryData.Length_U32];
	memcpy(MemAllocated, Instruction, sizeof(Instruction));
	memset(MemAllocated + sizeof(Instruction), 0, MemoryData.Length_U32);

	Result_S8 = SPI_TransmitReceive(MemAllocated, MemAllocated, sizeof(Instruction) + MemoryData.Length_U32);
	memcpy(MemoryData.Data_U8P, MemAllocated + sizeof(Instruction), MemoryData.Length_U32);

	return Result_S8;
}

int8_t AT25SF321X_Flash_Read(tMemoryData MemoryData)
{
	int16_t RResutl_S8 = -1;

	if (!MemoryData.Data_U8P || !MemoryData.Length_U32)
		return RResutl_S8;

	RResutl_S8 = AT25SF321X_ByteStandradRead(MemoryData);

	return RResutl_S8;
}

/* A block of 4 32 or 64 can be erase by call this function.
 * Attention: WEL bit in status register 1 must be set
 *
 * @param OpCode: Ref CMD_BLOCK_ERASE_xx, in header file
 *		  DevAddress_U32: flash address begin each block.
 *
 * @return  if executed successfully return 0 else return error
 */

static int8_t AT25SF3321x_EraseBlock(uint8_t Opcode, uint32_t DevAddress_U32, uint8_t NbBlock)
{
	uint8_t Result = -1;
	uint16_t Timeout_U16;

	if (!NbBlock)
		return 0;

	DevAddress_U32 &= (Opcode == CMD_BLOCK_ERASE_4K) ?  MASK_BLK_4K :
									((Opcode == CMD_BLOCK_ERASE_32K) ? MASK_BLK_32K :
											CMD_BLOCK_ERASE_64K);
	Timeout_U16 = 	(Opcode == CMD_BLOCK_ERASE_4K) ?  T_blk4e :
					((Opcode == CMD_BLOCK_ERASE_32K) ? T_blk32e :
							T_blk64e);

	while (NbBlock--)
	{
		Result = -1;

		if (1 == AT25SF321X_WriteEnable())
		{
			uint8_t frame[] = { Opcode,
								DevAddress_U32 >> 16,
								DevAddress_U32 >> 8,
								DevAddress_U32,
				};

			Result = SPI_Transmit(frame, sizeof(frame));

			int8_t CheckProgress(tStatusRegister* Status) { return Status->Reg1_T.ReadyFlag_B;};		//0 Device readly, stop polling

			Result = PollingReadStatusRegister(OPC_STATUS_REG_1, NULL, &CheckProgress, Timeout_U16);
			//if not use polling check can be use delay with specific time, ref datasheet, page 49
		}

		if (Result)
			break;
	};

	return Result;
}

static void AT25SF321X_Flash_Write(tMemoryData Input)
{
	int8_t Result_S8 = -1;
	uint16_t index = 0;
	uint16_t Length = 0;
	uint8_t MemAllocated[cBLOCK_4KB];
	tMemoryData RWData = {
							.Address_U32 = Input.Address_U32 & MASK_BLK_4K,
							.Data_U8P	 = MemAllocated,
							.Length_U32	 = cBLOCK_4KB
						 };

	do
	{
		Result_S8 = -1;

		index = Input.Address_U32 & (cBLOCK_4KB - 1);

		if ((index + Input.Length_U32) > cBLOCK_4KB)
			Length = cBLOCK_4KB - index;
		else
			Length = Input.Length_U32;

		Result_S8 = AT25SF321X_Flash_Read(RWData);
		memcpy(RWData.Data_U8P + index, Input.Data_U8P, Length);

		Result_S8 = AT25SF3321x_EraseBlock(CMD_BLOCK_ERASE_4K, RWData.Address_U32, 1);
		Result_S8 = AT25SF321X_Flash_WriteSector(RWData);

		if (Result_S8 == 0)
		{
			Input.Address_U32 += Length;
			Input.Data_U8P	  += Length;
			Input.Length_U32  -= Length;

			RWData.Address_U32 += cBLOCK_4KB;
		}

	} while ((Result_S8 == 0) && Input.Length_U32);

}

/**
 * @Description: The _WriteEnable command sets WEL bit in Status register, to execute command as
 * 	Byte/Page Program, Erase Block, etc...
 * @return  if executed successfully return WEL else return error
 */

int8_t AT25SF321X_WriteEnable()
{
	int8_t Result_U8 = -1;
	uint8_t Command = CMD_WRITE_EN;

	tStatusRegister StatusReg1 = {0};

	Result_U8 = SPI_Transmit(&Command, sizeof(Command));

	if (Result_U8 == 0)
	{
		int8_t CheckProgress(tStatusRegister* Status) { return !Status->Reg1_T.WEL_B;};		//0 Device readly, stop polling

		//Recommended: The status register be polled to determine if device has finished execution
		Result_U8 = PollingReadStatusRegister(OPC_STATUS_REG_1, &StatusReg1, &CheckProgress, T_wrsr);
	}

	if (Result_U8 == 0)
		return StatusReg1.Reg1_T.WEL_B;

	return Result_U8;
}

/*
 * Warning: The bit WEL on RgS1 must be set before call this function
 */
int8_t AT25SF321X_Flash_WriteSector(tMemoryData MemoryData)
{
	int16_t WResutl_S16 = -1;

	if (!MemoryData.Data_U8P || !MemoryData.Length_U32)
		return WResutl_S16;

	do
	{
		WResutl_S16 = -1;

		if (1 == AT25SF321X_WriteEnable())
		{
			WResutl_S16 = AT25SF321X_WriteBytePage(MemoryData);

			if (WResutl_S16 > 0)
			{
				MemoryData.Address_U32 += WResutl_S16;
				MemoryData.Data_U8P	   += WResutl_S16;
				MemoryData.Length_U32  -= WResutl_S16;
			}
		}
	} while ((WResutl_S16 > 0) && MemoryData.Length_U32);

	return MemoryData.Length_U32 ? (int8_t)WResutl_S16 : 0;
}

int8_t AT25SF321X_ReadManufactureAndDeviceID(uint32_t* Value)
{
	int8_t Result_U8 = -1;
	uint8_t Command[] = {
			CMD_READ_MANUFACTURE_DEV_ID,
			1,2,3								//Dummy
	};

	Result_U8 = SPI_TransmitReceive(Command, Value, sizeof(Command));
	(*Value) >>= 8;

	return Result_U8;
}

bool Flash_Initialize()
{
	bool ret_val=false;

	uint32_t TempData_U32 = {0x01234567};

	tMemoryData TestData_T = {
		.Address_U32 = 0,
		.Data_U8P = (void*)&TempData_U32,
		.Length_U32 = sizeof(TempData_U32),
	};

	AT25SF321X_Flash_Write(TestData_T);
	TempData_U32 = 0x0;
	AT25SF321X_Flash_Read(TestData_T);
	uint32_t Flash_UID= 0;
	AT25SF321X_ReadManufactureAndDeviceID(&Flash_UID);
	ret_val = ((0x01234567 == TempData_U32) && (Flash_UID == MANUFACTURE_DEVICE_ID));
	return ret_val;
}



eTestStatus Flash_FWTest(void)
{
	FLASH_test = RET_FAIL;
	#if FLASH_TEST
	HAL_GPIO_WritePin(FLASH_EN_GPIO_Port, FLASH_EN_Pin, GPIO_PIN_SET);
	if (Flash_Initialize() == true)
		FLASH_test = RET_OK;
	#endif  /* End of FLASH_TEST */
	return FLASH_test;
}

/**************************************************************************************/

/***************************** Extended I/O FW Test ***********************************/
// At power on, the I/Os are high. The I/Os should be high before being used as inputs
static uint8_t PCF8574A_PORT_DATA_T = 0x00;					//hold on the last state GPIO Pin

static int8_t I2C_Read(uint32_t DevAddress,  uint8_t* Data_P, uint16_t Length)
{
	tI2CPackage I2CPackage;

	I2CPackage.I2CMode_T		 = I2C_DR;
	I2CPackage.DeviceAddress_U16 = DevAddress;
	I2CPackage.Data_U8P			 = Data_P;
	I2CPackage.Length_U16		 = Length;
	I2C_Transferring(&I2CPackage);
	return 0;
}

static int8_t I2C_Write(uint32_t DevAddress, uint8_t* Data_P, uint16_t Length)
{
	tI2CPackage I2CPackage;

	I2CPackage.I2CMode_T		 = I2C_DW;
	I2CPackage.DeviceAddress_U16 = DevAddress;
	I2CPackage.Data_U8P			 = Data_P;
	I2CPackage.Length_U16		 = Length;

	I2C_Transferring(&I2CPackage);

	return 0;
}


static int8_t PCF8574A_Write_Port()
{
	return I2C_Write(PCF8574A_ADR, &PCF8574A_PORT_DATA_T, sizeof(PCF8574A_PORT_DATA_T));
}

static int8_t PCF8574A_Read_Port()
{
	return I2C_Read(PCF8574A_ADR, &PCF8574A_PORT_DATA_T, sizeof(PCF8574A_PORT_DATA_T));
}

eTestStatus EXT_IO_FWTest(void)
{
	EXT_IO_test = RET_FAIL;
	#if EXT_IO_TEST
	HAL_GPIO_WritePin(EXT_IO_EN_GPIO_Port, EXT_IO_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	for(uint8_t count=0; count<3; count++)
	{
			PCF8574A_PORT_DATA_T |= (1<<count);
			PCF8574A_Write_Port();
			HAL_Delay(1000);

			PCF8574A_PORT_DATA_T = 0;
			PCF8574A_Write_Port();
			HAL_Delay(1000);
		}
		PCF8574A_PORT_DATA_T = 0;
		PCF8574A_Write_Port();

	PCF8574A_PORT_DATA_T = 0xFA;
	PCF8574A_Read_Port();
	if(PCF8574A_PORT_DATA_T == 0x00)
		EXT_IO_test = RET_OK;
	#endif  /* End of EXT_IO_TEST */
	return EXT_IO_test;
}
/**************************************************************************************/




/********************************* ADC FW Test ************************************/
extern DMA_HandleTypeDef hdma_adc1;
uint32_t g_ui32vref=0;
uint32_t g_ui32bat=0;
#ifndef FW_ELECFENCE
#define BUF_SIZE	2	/* VREF | ADC 3.3 */
#else
#define BUF_SIZE	5	/* VREF | ADC 3.3 | ADC 12V | ADC+ | ADC- */
#define ADC_DMA_BUF_LEN			10000
#define T_HIGHVOLTAGE 1200*2	/* 1200 ms */

uint32_t ADC_DMA_BUF[ADC_DMA_BUF_LEN]={0};
uint32_t ADC_DMA_DATA[ADC_DMA_BUF_LEN/2]={0};


uint32_t g_ui32_12v=0;
uint32_t g_ui32_pos=0;
uint32_t g_ui32_neg=0;

uint32_t g_adc_pos=0;
uint32_t g_adc_neg=0;
#endif  /* End of ifdef FW_ELECFENCE */

uint32_t g_ui32ADCraw[BUF_SIZE]={0};	/* Raw ADC input data */
uint32_t g_ui32input[BUF_SIZE-1]={0}; 	/* Ignore Vref */

volatile uint32_t g_max_eoc=0;
volatile uint32_t g_max_eos=0;

volatile bool g_newadcdata=false;		/* New ADC data flag */
volatile bool g_useddma = false;		/* true => DMA(HV), false => ADC interrupt (EOS,EOC - battery)*/
volatile bool g_flag_dmahalf = false;
volatile bool g_flag_dmafull = false;


uint16_t ADC_ReadData()
{
	uint16_t ui16_adcdata=0;

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
	{
		/* Calibration Error */
		return ui16_adcdata;
	}

	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		/* ADC conversion start error */
		return ui16_adcdata;
	}

	/* Read data of configured channel */
	if (HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK)
	{
		return ui16_adcdata;
	}

	if (HAL_ADC_Stop(&hadc1) != HAL_OK)
	{
		/* ADC conversion stop error */
		return ui16_adcdata;
	}
	ui16_adcdata = HAL_ADC_GetValue(&hadc1);

	return ui16_adcdata;
}

void ADC_PowerInit()
{
	//FIXME: This is what 1h est make not me
	ADC_ChannelConfTypeDef sConfig = {0};
	HAL_ADC_DeInit(&hadc1);

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* Read Raw ADC VREF  */
	g_ui32ADCraw[0] = ADC_ReadData();

	/** Configure Regular Channel
	 */
	sConfig.Channel = BATT_ADC_CHANNEL;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* Read Raw ADC Vbat  */
	g_ui32ADCraw[1] = ADC_ReadData();

	/** Configure Regular Channel
	 */
	sConfig.Channel = V12V_ADC_CHANNEL;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* Read Raw ADC V12V  */
	g_ui32ADCraw[2] = ADC_ReadData();

}

void ADC_HVInit()
{
	g_useddma = false;
	g_newadcdata = false;
	ADC_ChannelConfTypeDef sConfig = {0};
	HAL_ADC_DeInit(&hadc1);
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	 sConfig.Channel = ADC_CHANNEL_VREFINT;
	 sConfig.Rank = ADC_REGULAR_RANK_1;
	 sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	 sConfig.SingleDiff = ADC_SINGLE_ENDED;
	 sConfig.OffsetNumber = ADC_OFFSET_NONE;
	 sConfig.Offset = 0;
	 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 /** Configure Regular Channel
	  */
	 sConfig.Channel = ADC_CHANNEL_9;
	 sConfig.Rank = ADC_REGULAR_RANK_2;
	 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	 {
		 Error_Handler();
	 }
	 /** Configure Regular Channel
	  */
	 sConfig.Channel = ADC_CHANNEL_15;
	 sConfig.Rank = ADC_REGULAR_RANK_3;
	 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	 {
		 Error_Handler();
	 }

}

void ADC_ElecFenceInit()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	HAL_ADC_DeInit(&hadc1);
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;	/* Continuous */
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = HV_POS_CHANNEL;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = HV_NEG_CHANNEL;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/* This function configure the MCU to continously read 2 ADC channels without doing anything else */
eTestStatus HV_Monitor(void)
{
	HAL_GPIO_WritePin(OPA_SW_GPIO_Port, OPA_SW_Pin, GPIO_PIN_SET);
	ADC_HVInit();
	HAL_Delay(100);
	while(1)
	{
		g_newadcdata = false;
		HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		HAL_ADC_Start_IT(&hadc1);
		while(g_newadcdata != true);
		HAL_ADC_Stop_IT(&hadc1);
		g_ui32vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(g_ui32ADCraw[0], ADC_RESOLUTION_12B);
		g_adc_pos= __LL_ADC_CALC_DATA_TO_VOLTAGE(g_ui32vref, g_ui32ADCraw[1], ADC_RESOLUTION_12B);
		g_adc_neg= __LL_ADC_CALC_DATA_TO_VOLTAGE(g_ui32vref, g_ui32ADCraw[2], ADC_RESOLUTION_12B);
		g_newadcdata = false;
		#if !ENDLESS_ADC_HV_MEASURING
		break;
		#endif /*End of ENDLESS_ADC_HV_MEASURING*/
	}
	return RET_OK;
}

eTestStatus ADC_FWTest(void)
{
	ADC_test = RET_FAIL;
	#if ADC_TEST
	HAL_GPIO_WritePin(VREF_EN_GPIO_Port, VREF_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);			/* Wait for stable */
	ADC_PowerInit();
	while(1)
	{
		ADC_test = RET_OK;
		g_ui32vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(g_ui32ADCraw[0], ADC_RESOLUTION_12B);
		g_ui32input[0]= __LL_ADC_CALC_DATA_TO_VOLTAGE(g_ui32vref, g_ui32ADCraw[1], ADC_RESOLUTION_12B);
		g_ui32input[1]= __LL_ADC_CALC_DATA_TO_VOLTAGE(g_ui32vref, g_ui32ADCraw[2], ADC_RESOLUTION_12B);
		g_ui32bat = 4 * g_ui32input[0];
		g_ui32_12v = g_ui32input[1] *5;
		printf("Battery voltages: %ld (mV)\n",g_ui32bat);
		printf("Relay voltages: %ld (mV)\n",g_ui32_12v);
		g_newadcdata = false;
		#if !ENDLESS_BATT_MEASURING
		break;
		#endif /*End of ENDLESS_BATT_MEASURING*/
	}

	#else
	printf("----- Skipped test ----- \n");
	#endif /*End of ADC_TEST*/
	return ADC_test;
}

/* If generate speed > processing speed ===> OK */
/* If generate speed < processing speed ===> NOT OK since data will be all zeros */
uint32_t GetLargest(uint32_t* p_arr, uint32_t len)
{
    // Initialize maximum element
    uint32_t max = *p_arr;
    // Traverse array elements from second and compare every element with current max
    for (uint32_t i = 1; i < len; i++)
    {
        if (*(p_arr + i) > max)
            max = *(p_arr + i);
    }
    return max;
}

/* Find the max ADC value in both ADC channels using DMA */
eTestStatus ADC_ElecFenceTest(void)
{
	ADCElecFence_Test = RET_FAIL;
	#ifdef ADC_ELECFENCE_TEST
	static bool Isinit = false;
	if(Isinit == false)
	{
		ADC_ElecFenceInit();
		g_useddma = true;
		Isinit = true;
	}
	/* Start measuring & converting */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	g_flag_dmafull = false;
	g_flag_dmahalf = false;
	uint32_t max_temp = 0;
	uint32_t max_ret=0;
	HAL_ADC_Start_DMA(&hadc1, ADC_DMA_BUF, ADC_DMA_BUF_LEN);
	for(uint32_t count = 0; count < 3440; )	//172 ~ 1s
	{
		/* Find max of [0 -> Len/2] */
		max_temp = GetLargest(&ADC_DMA_BUF[0], (ADC_DMA_BUF_LEN / 2));
		if(max_temp > max_ret)
		{
			max_ret = max_temp;
		}
		if(g_flag_dmahalf == true)
		{
			/* Find max of [Len/2 -> Len] */
			max_temp = GetLargest(&ADC_DMA_BUF[ADC_DMA_BUF_LEN / 2], (ADC_DMA_BUF_LEN / 2));
			if(max_temp > max_ret)
			{
				max_ret = max_temp;
			}
			g_flag_dmahalf = false;
		}
		if(g_flag_dmafull == true)
		{
			count ++;
			g_flag_dmafull = false;
		}
	}
	HAL_ADC_Stop_DMA(&hadc1);
	g_max_hv= __LL_ADC_CALC_DATA_TO_VOLTAGE(g_ui32vref, max_ret, ADC_RESOLUTION_12B);
	#else
	printf("----- Skipped test ----- \n");
	#endif /*End of ADC_ELECFENCE_TEST*/
	return ADCElecFence_Test;
}

/* Find the max values in each ADC channel using interrupt */
eTestStatus ADC_ElecFenceTestInt(void)
{
	uint32_t est_time = HAL_GetTick() + T_HIGHVOLTAGE;
	uint32_t cur_time=0;
	ADCElecFence_Test = RET_FAIL;
	#ifdef ADC_ELECFENCE_TEST
	static bool IsReady = false;
	if(IsReady == false)
	{
		HAL_GPIO_WritePin(OPA_SW_GPIO_Port, OPA_SW_Pin, GPIO_PIN_SET);
		ADC_ElecFenceInit();
		IsReady = true;
	}
	/* Start measuring & converting */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_IT(&hadc1);
	while( cur_time < est_time)	/* Wait */
	{
		cur_time = HAL_GetTick();
	}
	HAL_ADC_Stop_IT(&hadc1);
	g_pos_max= __LL_ADC_CALC_DATA_TO_VOLTAGE(g_ui32vref, g_max_eoc, ADC_RESOLUTION_12B);
	g_neg_max= __LL_ADC_CALC_DATA_TO_VOLTAGE(g_ui32vref, g_max_eos, ADC_RESOLUTION_12B);

	g_pos_max = (uint32_t) (g_pos_max * 5.318);
	g_neg_max = (uint32_t) (g_neg_max *5.318);

	printf("HV positive: %d (V)    HV negative: %d (V)\n\n", g_pos_max, g_neg_max);

	g_max_eoc=0;
	g_max_eos=0;
	#else
	printf("----- Skipped test ----- \n");
	#endif /*End of ADC_ELECFENCE_TEST*/
	return ADCElecFence_Test;
}

/**************************************************************************************/
/********************************---BLE TEST---****************************************/
extern ELFence_Batt_Data_t Batt_Data;
volatile uint16_t g_test_vbat=0;
void BLE_TestNotify(void)
{
	static uint16_t prev_vbat=0;
	if( g_test_vbat != prev_vbat)
	{
		Batt_Data.Batt_volt_U16 = g_test_vbat;
		Custom_Vbat_Send_Notification();
		prev_vbat = g_test_vbat;
	}
}

/********************************---NFC TEST---****************************************/
eTestStatus NFC_FWTest(void)
{
	NFC_test = RET_FAIL;
	GPIOB->ODR |= (0X02);
	if (NFCTAG_OK == CUSTOM_NFCTAG_Init(0) )
	{
		CUSTOM_NFCTAG_WriteData(0, (uint8_t * const)&g_ui8dataWrite, 0, LENGTH);
		memset(g_ui8dataRecv, 1, LENGTH);
		CUSTOM_NFCTAG_ReadData(0, (uint8_t * const)&g_ui8dataRecv, 0, LENGTH);
		if( memcmp((char*)g_ui8dataRecv, (char*)g_ui8dataWrite, LENGTH) == 0) //Test read/write NFC
		{
			g_nfc_init = true;
			NFC_test = RET_OK;
			uint8_t dummy_clear_int=0;
			CUSTOM_NFCTAG_ReadITSTStatus_Dyn(0, &dummy_clear_int);
			CUSTOM_NFCTAG_SetGPO_en_Dyn(0);
		}
	}
//	GPIOB->ODR &= ~(0X02);
	return	NFC_test;
}

/**************************************************************************************/

void ButtonsHandler(void)
{
	if (g_button2pressed == true)
	{
		if(HAL_GPIO_ReadPin(SW_DIS_GPIO_Port, SW_DIS_Pin) == GPIO_PIN_SET) //Release
		{
			if(g_countbuttonpress <10)			//not stable press
			{

			}
			else
			{
				g_button_interrupt = true;
				printf("Button pressed \n");
			}
			g_button2pressed = false;
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
	printf("Entering stop 2 mode...\n");

	LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);
	__HAL_RCC_LPUART1_CLK_DISABLE();
	#if DEBUG_LPOWER
	LL_DBGMCU_EnableDBGSleepMode();
	LL_DBGMCU_EnableDBGStopMode();
	LL_DBGMCU_EnableDBGStandbyMode();
	#else
	LL_DBGMCU_DisableDBGSleepMode();
	LL_DBGMCU_DisableDBGStopMode();
	LL_DBGMCU_DisableDBGStandbyMode();
	#endif  /* End of DEBUG_LPOWER */

	// Module control pins -> low output
	HAL_GPIO_WritePin(GPIOB, PORTB_OUTPUT_PINS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, PORTA_OUTPUT_PINS, GPIO_PIN_RESET);


	HAL_GPIO_DeInit(GPIOB, (GPIO_PIN_6 | GPIO_PIN_7) );
	HAL_GPIO_DeInit(GPIOA, (GPIO_PIN_2 | GPIO_PIN_3) );

	HAL_SPI_MspDeInit(&hspi1);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Stop SYSTICK Timer
	HAL_SuspendTick();

	// Enter Stop Mode
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

	//Wake up
	SystemClock_Config();

	// Resume SYSTICK Timer
	HAL_ResumeTick();

}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	g_flag_dmahalf = true;
}


#if PLOT_HV_ADC
/* Quickly collect ADC data (raw ADC negative and ADC positive) */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(LL_ADC_IsActiveFlag_EOS(ADC1) != 0)  /* EOS event */
	{
		g_ui32_neg = HAL_ADC_GetValue(&hadc1);
	}
	else if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)									/* EOC event */
	{
		g_ui32_pos = HAL_ADC_GetValue(&hadc1);
	}

}
#elif ADC_ELECFENCE_TEST
/* Find max ADC data from both channels (raw ADC negative and ADC positive) */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	//FIXME: Continuously interrupt for about 1200*2 (ms) to find HV max values
	uint32_t adc_data;
	if(LL_ADC_IsActiveFlag_EOS(ADC1) != 0)  /* EOS event */
	{
		adc_data = HAL_ADC_GetValue(&hadc1);
		if (adc_data > g_max_eos )
			g_max_eos = adc_data;
	}
	if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)/* EOC event */
	{
		adc_data = HAL_ADC_GetValue(&hadc1);
		if (adc_data > g_max_eoc )
			g_max_eoc = adc_data;
	}
}
#else
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *p_hadc)
{
}
#endif  /* End of ADC_ELECFENCE_TEST */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case WAKEUP_Pin: //ACL wake up handler -- FIX ME NFC
		g_acl_interrupt = true;
		ACL_ReadSource();
		ACL_EnableInterrupt();
		break;

		case SW_DIS_Pin:
			g_button2pressed = true;
			printf("SW2 pressed \n");
			if(g_testingble == true)
			{
				UTIL_SEQ_SetTask(1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
			}
		break;

		case GPIO_PIN_4:
		{
			printf("SW1 pressed \n");
			g_button1pressed=true;

		}
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	g_rtcwakeup = true;
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
	else if (I2CPackage_T->I2CMode_T == I2C_DW)
	{
		result = HAL_I2C_Master_Transmit(&hi2c1,
										 I2CPackage_T->DeviceAddress_U16,
										 I2CPackage_T->Data_U8P,
										 I2CPackage_T->Length_U16,
										 0x1000U);
	}
	else if (I2CPackage_T->I2CMode_T == I2C_DR)
	{
		result = HAL_I2C_Master_Receive(&hi2c1,
										I2CPackage_T->DeviceAddress_U16,
										I2CPackage_T->Data_U8P,
										I2CPackage_T->Length_U16,
										0x1000U);
	}
	if (result != HAL_OK) {
		//ErrorHander(result);
	}
	return result;
}

/**************************************************************************************/
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
	HAL_Delay(100);
	/* USER CODE END Init */

	/* Configure the system clock */

	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	HAL_Delay(100);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	/* Onboard LED */
	GPIOA->ODR |= LED_D2_Pin;
	HAL_Delay(500);
	GPIOA->ODR |= LED_D3_Pin;
	HAL_Delay(500);

	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_RF_Init();
	MX_RTC_Init();
	MX_I2C1_Init();
	MX_LPUART1_UART_Init();
	MX_TIM16_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
#if DEBUG_ITM
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif  /* End of DEBUG_ITM */


#if TRIGGER_BUTTON
	printf("Trigger SW2 to start the H/W test \n");
	while(g_button2pressed != true);
#endif  /* End of TRIGGER_BUTTON */

#if HW_SYSTEST
	printf("FW Test started... \n");
	Sys_Test();
#endif  /* End of HW_SYSTEST */
	/* USER CODE END 2 */

	/* Init code for STM32_WPAN */
#if BLE_TEST
	g_testingble = true;
	printf("Initializing BLE ...\n");
	APPE_Init();
#endif  /* End of BLE_TEST */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	printf("Electrical fence testing... \n");
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
#if !HV_MAX_EACH_ADC_CHANNEL
		ADC_ElecFenceTestInt();
#endif  /* End of HV_MAX_EACH_ADC_CHANNEL */

#if BLE_TEST
		UTIL_SEQ_Run(~0);
		//		BLE_TestNotify();
#endif  /* End of BLE_TEST */

#if PLOT_HV_ADC
		HV_Monitor();
#endif  /* End of PLOT_HV_ADC */

#if HV_MAX_OF_BOTH_ADCS
		ADC_ElecFenceTest();
		printf("HV %d (mV)\n", g_max_hv);
#endif  /* End of HV_MAX_OF_BOTH_ADCS */

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
