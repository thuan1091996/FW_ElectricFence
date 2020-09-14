/*
 * AT0x.c
 * x = 4 or 8
 *
 *  Created on: Sep 19, 2019
 *      Author: DFMPC-01201117
 */

#include "AT0x.h"

/*****************************************************************************************************************/
/*													GLOBAL VARIABLES											 */
/*****************************************************************************************************************/
extern tFlash_HAL Flash_HAL_T;
static tGPIODef Flash_GPIO_PWR = {
#ifdef MCU_STM32
		.pin	= PA_2,
#else
		.pin	= PB_13,
#endif
		.mode	= PIN_OUTPUT,
		.pull	= PIN_PUSH_PULL,	//PIN_PUSH_PULL,
};

/*****************************************************************************************************************/
/*												HARDWARE ABSTRACT LAYER									 		 */
/*****************************************************************************************************************/
static void FlashInit();

static void FlashRead(tFlashData FlashData);
static void FlashWrite(tFlashData FlashData);
static bool FlashErase(tFlashData FlashData);
static void FlashSleeping(void* Def);
static void FlashWakeup(void* Def);

void __attribute__((weak)) HAL_Flash_Interaction(tFlash_HAL *HAL)
{
	HAL->FlashType_E = AT08;
	HAL->PERIPHERAL_SERVICES_T.Initialize	= &FlashInit;
	HAL->PERIPHERAL_SERVICES_T.Read			= &FlashRead;
	HAL->PERIPHERAL_SERVICES_T.Write		= &FlashWrite;
	HAL->PERIPHERAL_SERVICES_T.Erase		= &FlashErase;
	HAL->PERIPHERAL_SERVICES_T.Sleeping		= &FlashSleeping;
	HAL->PERIPHERAL_SERVICES_T.Wakeup		= &FlashWakeup;
}

/*****************************************************************************************************************/
/*													PRIVATE VARIABLES											 */
/*****************************************************************************************************************/


/*****************************************************************************************************************/
/*													PRIVATE FUNCTION											 */
/*****************************************************************************************************************/
static void FlashWritePage(tFlashData FlashData_T);

/*****************************************************************************************************************/
/*													FUNCTION IMPLEMENT									 	 	 */
/*****************************************************************************************************************/
static void FlashInit()
{
	Flash_HAL_T.Enable_B = 1;
	Flash_HAL_T.DeviceStatus_E = Activated;

	Flash_HAL_T.Flash_GPIO_PWR->Initialize(&Flash_GPIO_PWR, SET);
}

static void FlashRead(tFlashData FlashData)
{
	FlashData.FlashMode_T = Flash_WR;
	FlashData.FlashAddress_U16 = mFlashAddress(FlashData.MemoryAddress_U16);
	FlashData.MemoryAddress_U16 = mMemoryAddress(FlashData.MemoryAddress_U16);

	HAL_CORE_TRANSFER((tI2CPackage*)&FlashData);
	Delay_ms(5);
}

void FlashWrite(tFlashData FlashData)
{
	// Write first page if not aligned
	uint16_t Length_U16 = FlashData.Length_U16;
	uint8_t	 pageOffset = FlashData.MemoryAddress_U16 % cPageSize_U16;

	FlashData.Length_U16 = 0;

    if (pageOffset > 0)
    {
    	FlashData.Length_U16 = cPageSize_U16 - pageOffset;

    	if (FlashData.Length_U16 >= Length_U16)
    		FlashData.Length_U16 = Length_U16;

        FlashWritePage(FlashData);
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
			FlashWritePage(FlashData);

			FlashData.MemoryAddress_U16	+= cPageSize_U16;
			FlashData.Data_U8P		 	+= cPageSize_U16;
			Length_U16						-= cPageSize_U16;

		}

		if (Length_U16 > 0)
		{
			// Write remaining uncomplete page.
			FlashData.Length_U16 = Length_U16;
			FlashWritePage(FlashData);
		}
		return 1;
	}

    return 0;
}

static void FlashWritePage(tFlashData FlashData_T)
{
	if(((FlashData_T.MemoryAddress_U16 & (cPageSize_U16-1)) + FlashData_T.Length_U16) > cPageSize_U16)
	{
		FlashData_T.Length_U16 = cPageSize_U16 - (FlashData_T.MemoryAddress_U16 & (cPageSize_U16-1));
	}

	FlashData_T.FlashMode_T = Flash_WW;
	FlashData_T.FlashAddress_U16  = mFlashAddress(FlashData_T.MemoryAddress_U16);
	FlashData_T.MemoryAddress_U16 = mMemoryAddress(FlashData_T.MemoryAddress_U16);

	HAL_CORE_TRANSFER((tI2CPackage*)&FlashData_T);
	Delay_ms(5);
}

static bool FlashErase(tFlashData FlashData)
{
	memset(FlashData.Data_U8P, 0, FlashData.Length_U16);
	FlashWrite(FlashData);
}

static void FlashSleeping(void* Def)
{
	Flash_HAL_T.Flash_GPIO_PWR->Write(&Flash_GPIO_PWR, RESET);
}

static void FlashWakeup(void* Def)
{
	Flash_HAL_T.Flash_GPIO_PWR->Write(&Flash_GPIO_PWR, SET);
}




