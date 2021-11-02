/*
 * PCF8574A.c
 *
 *  Created on: Aug 13, 2021
 *      Author: ChienNguyen
 */
#include "PCF8574A.h"


#define mBIT(idx) (1 << idx)

// At power on, the I/Os are high. The I/Os should be high before being used as inputs
static uint8_t PCF8574A_PORT_CONFIGURE_T = 0x00;			//Input.Exti Pin: 0, Output: 1
static uint8_t PCF8574A_PORT_DATA_T = 0x00;					//hold on the last state GPIO Pin

static void* PIN_IRQ_CALLBACK[8] = {NULL};

static tGPIO_EXT_HAL* GPIO_HAL_TP = 0;

static int8_t PCF8574A_Write_Pin(uint8_t Pin, uint32_t State);
static int8_t PCF8574A_Read_Pin(uint8_t Pin, uint32_t* State);
static int8_t PCF8574A_Write_Port();
static int8_t PCF8574A_Read_Port();

/*****************************************************************************************************************/
/*												INTEGRATION								 						 */
/*****************************************************************************************************************/
static int8_t I2C_Read(uint32_t DevAddress,  uint8_t* Data_P, uint16_t Length)
{
	tI2CPackage I2CPackage;

	I2CPackage.I2CMode_T		 = I2C_DR;
	I2CPackage.DeviceAddress_U16 = DevAddress;
	I2CPackage.Data_U8P			 = Data_P;
	I2CPackage.Length_U16		 = Length;

	((tI2C*)GPIO_HAL_TP->CORE_SERVICES_T.Driver_TP)->TxSend((uint8_t*)&I2CPackage, 0);

	return 0;
}

static int8_t I2C_Write(uint32_t DevAddress, uint8_t* Data_P, uint16_t Length)
{
	tI2CPackage I2CPackage;

	I2CPackage.I2CMode_T		 = I2C_DW;
	I2CPackage.DeviceAddress_U16 = DevAddress;
	I2CPackage.Data_U8P			 = Data_P;
	I2CPackage.Length_U16		 = Length;

	((tI2C*)GPIO_HAL_TP->CORE_SERVICES_T.Driver_TP)->TxSend((uint8_t*)&I2CPackage, 0);

	return 0;
}

static int8_t PCF8574A_Initialize(void* Driver)
{
	uint8_t Result = -1;

	//TODO:  Check Info Register

	if (PCF8574A_Write_Port())
		GPIO_HAL_TP->Enable_B = 1;

	return Result;
}

static int8_t PCF8574A_Write_Port()
{
	return I2C_Write(PCF8574A_ADR, &PCF8574A_PORT_DATA_T, sizeof(PCF8574A_PORT_DATA_T));
}

static int8_t PCF8574A_Read_Port()
{
	return I2C_Read(PCF8574A_ADR, &PCF8574A_PORT_DATA_T, sizeof(PCF8574A_PORT_DATA_T));
}

/*****************************************************************************************************************/
/*												        								 						 */
/*****************************************************************************************************************/

static int8_t PCF8574A_Pin_Init(uint8_t Pin, uint8_t Mode, uint8_t value)
{
	int8_t Result_S8 = -1;

	PCF8574A_PORT_CONFIGURE_T &= ~mBIT(Pin);
	PCF8574A_PORT_CONFIGURE_T |= Mode << Pin;

	if (MODE_OUTPUT == Mode)
	{
		Result_S8 = PCF8574A_Write_Pin(Pin, value);
	}

	return Result_S8;
}

static int8_t PCF8574A_Pin_Deinit(uint8_t Pin)
{
	PCF8574A_PORT_CONFIGURE_T &= ~mBIT(Pin);			//set pin mode output
	return 0;
}

static int8_t PCF8574A_Pin_Sleep()
{
	int8_t Result = -1;

	// ......
	Result = 0;

	return Result;
}

static int8_t PCF8574A_Pin_Wakeup()
{
	int8_t Result = -1;

	if (PCF8574A_Write_Port())
		Result = 0;

	return Result;
}

static int8_t PCF8574A_Set_Mode(uint8_t Pin, uint8_t Mode, void (*IrqCallback)())
{
	PCF8574A_PORT_CONFIGURE_T &= ~mBIT(Pin);
	PCF8574A_PORT_CONFIGURE_T |= Mode << Pin;

	if (MODE_INPUT == Mode)
		PIN_IRQ_CALLBACK[Pin] = IrqCallback;

	return 0;
}

static int8_t PCF8574A_Clear_Mode(uint8_t Pin)
{
	PIN_IRQ_CALLBACK[Pin] = 0;
	return 0;
}

int8_t PCF8574A_Write_Pin(uint8_t Pin, uint32_t State)
{
	int8_t Result = -1;

	assert_param(State);
	assert_param(IS_PCF8574A_PIN(Pin));

	if (mBIT(Pin) & PCF8574A_PORT_CONFIGURE_T)	// Output
	{
		PCF8574A_Read_Port();

		PCF8574A_PORT_DATA_T &= ~mBIT(Pin);
		PCF8574A_PORT_DATA_T |= (State != 0) << Pin;

		Result = PCF8574A_Write_Port();
	}

	return Result;
}

int8_t PCF8574A_Read_Pin(uint8_t Pin, uint32_t* State)
{
	int8_t Result = -1;

	assert_param(State);
	assert_param(IS_PCF8574A_PIN(Pin));

	Result = PCF8574A_Read_Port();

	if (0 == Result)
	{
		*State = (0 != (PCF8574A_PORT_DATA_T & mBIT(Pin)));
	}

	return Result;
}

int8_t PCF8574A_Toggle_Pin(uint8_t Pin)
{
	uint8_t PinState = (0 == (PCF8574A_PORT_DATA_T & mBIT(Pin)));

	return PCF8574A_Write_Pin(Pin, PinState);
}

static int8_t ClearIrq()
{
	return 0;
}

static void PCF8574A_IRQHandler(uint16_t Pin)
{
	ePinIRQ PinsIRQ_E = 0;
	uint8_t IrqIdx = 0;

	uint8_t LastGPIOState = PCF8574A_PORT_DATA_T;

	 if (0 == PCF8574A_Read_Port())
	 {
		 PinsIRQ_E = PCF8574A_PORT_CONFIGURE_T & PCF8574A_PORT_DATA_T & LastGPIOState;

		 do
		 {
			if ((PinsIRQ_E & 0x01) && (PIN_IRQ_CALLBACK[IrqIdx] != NULL))
			{
				void (*IrqCallback)(uint32_t)  = PIN_IRQ_CALLBACK[IrqIdx];
				IrqCallback(IrqIdx);
			}

			IrqIdx++;
		 } while(PinsIRQ_E >>= 1);
	 }

	 ClearIrq();
}

void  HAL_DEXT_GPIO_Interaction(tGPIO_EXT_HAL *HAL)
{

#define EXT_GPIO_SERVICE    HAL->PERIPHERAL_SERVICES_T
#define MCU_SERVICE         HAL->CORE_SERVICES_T

	GPIO_HAL_TP = HAL;

	if (PCF8574A_Initialize(HAL->CORE_SERVICES_T.Driver_TP))
	{
		EXT_GPIO_SERVICE.GPIOInit			= &PCF8574A_Pin_Init;
		EXT_GPIO_SERVICE.SetInterrupt		= &PCF8574A_Set_Mode;
		EXT_GPIO_SERVICE.RemoveInterrupt	= &PCF8574A_Clear_Mode;
		EXT_GPIO_SERVICE.Read				= &PCF8574A_Read_Pin;
		EXT_GPIO_SERVICE.Write				= &PCF8574A_Write_Pin;
		EXT_GPIO_SERVICE.Toggle				= &PCF8574A_Toggle_Pin;
		EXT_GPIO_SERVICE.DeInit				= &PCF8574A_Pin_Deinit;
		EXT_GPIO_SERVICE.Sleep				= &PCF8574A_Pin_Sleep;
		EXT_GPIO_SERVICE.Wakeup				= &PCF8574A_Pin_Wakeup;

		MCU_SERVICE.IrqHandle = &PCF8574A_IRQHandler;
	}
}
