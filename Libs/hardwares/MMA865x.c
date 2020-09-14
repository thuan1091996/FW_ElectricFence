/*
 * MMA865x.c
 * x = 2 or 3
 *
 *  Created on: Sep 19, 2019
 *      Author: DFMPC-01201117
 */

#include "MMA865x.h"
#include "../HAL.h"
#include "../../../../app/appdef.h"

/*****************************************************************************************************************/
/*													GLOBAL VARIABLES											 */
/*****************************************************************************************************************/
extern tAccelerometer_HAL Accelerometer_HAL_T;

static tGPIODef GPIO_Wakeup_T = {
#ifdef APP_ORANGE
		.pin	= PA_9,
		.mode	= PIN_INPUT,
		.pull	= PIN_NO_PULL,
#else
		.pin	= PC_10,
		.mode	= 3,//PIN_INPUT_PULL_FILTER,
		.pull	= PIN_PUSH_PULL,
#endif
};

/*****************************************************************************************************************/
/*													PRIVATE FUNCTION											 */
/*****************************************************************************************************************/
static void AccelerometerInit();
static void Active();
static void Standby();
static void Reset();

static void Read(uint8_t Register_U8, uint8_t* Data_U8P, uint8_t Length_U8);
static void Write(uint8_t Register_u8, uint8_t* Data_U8P, uint8_t Length_U8);
static void ReadAxis();

static void EnableInterrupt(uint8_t Source_U8);
static void DisableInterrupt(uint8_t Source_U8);

static void Sleeping(void *);
static void Wakeup(void *);

static void GetAndClearSourceINT(uint8_t* Source_U8P);

/*****************************************************************************************************************/
/*											HARDWARE ABSTRACT LAYER									 			 */
/*****************************************************************************************************************/
void __attribute__((weak)) HAL_Accelerometer_Interaction(tAccelerometer_HAL *HAL)
{

#ifdef ACCELEROMETER_MMA8652
	HAL->AccelerometerType_E = MMA8652;
#else
	HAL->AccelerometerType_E = MMA8653;
#endif

	HAL->SENSOR_SERVICES_T.Initialize			= &AccelerometerInit;
	HAL->SENSOR_SERVICES_T.ActiveMode			= &Active;
	HAL->SENSOR_SERVICES_T.StandByMode			= &Standby;
	HAL->SENSOR_SERVICES_T.Reset				= &Reset;
	HAL->SENSOR_SERVICES_T.Read					= &Read;
	HAL->SENSOR_SERVICES_T.Write				= &Write;
	HAL->SENSOR_SERVICES_T.ReadAxis				= &ReadAxis;
	HAL->SENSOR_SERVICES_T.Sleeping				= &Sleeping;
	HAL->SENSOR_SERVICES_T.Wakeup				= &Wakeup;
	HAL->SENSOR_SERVICES_T.EnableInterrupt		= &EnableInterrupt;
	HAL->SENSOR_SERVICES_T.DisableInterrupt		= &DisableInterrupt;
	HAL->SENSOR_SERVICES_T.GetAndClearSourceINT	= &GetAndClearSourceINT;
}

/*****************************************************************************************************************/
/*													PRIVATE VARIABLES											 */
/*****************************************************************************************************************/
//static PinNames 	SDA_E 		= 	SDA_PB9;
//static PinNames 	SCL_E 		= 	SCL_PB8;
//static uint8_t		CTRLREG4;
//uint8_t bufff;

static uint8_t CtrlReg4 = 0;
static uint8_t CtrlReg5 = 0;

/*****************************************************************************************************************/
/*													PRIVATE FUNCTION											 */
/*****************************************************************************************************************/

static uint8_t RegisterGet(uint8_t Register_U8);
static void RegisterSet(uint8_t Register_U8, uint8_t Data_U8P);

static void EnableIntRegister(uint8_t Source_U8, bool PinINT);
static void DisableIntRegister(uint8_t Source_U8);

static void EnableLndPrtInt(tLandscapePortaitDef Def);
static void DisableLndPrtInt();

static void EnableTapInt(tPulseDef Def);
static void DisableTapInt();

static void EnableFfMtInt(tFreefallMotionDef Def);
static void DisableFfMtInt();

/*****************************************************************************************************************/
/*													FUNCTION IMPLEMENT									 	 	 */
/*****************************************************************************************************************/
/**
 * Init
 *
 * Initialize MMA8652 sensor
 */
static void AccelerometerInit()
{
	// Check Who AM I
	if (RegisterGet(MMA865X_WHO_AM_I) != cMMA865X_WHO_AM_I)	// 0x4A: 8652, 0x5A: 8653
	{
		// TODO: implement
	}

	Accelerometer_HAL_T.Enable_B = 1;
	Accelerometer_HAL_T.DeviceStatus_E = Activated;

	GPIO_INIT(&GPIO_Wakeup_T, RESET);
	GPIO_SetInterrupt(&GPIO_Wakeup_T, IRQ_RISING_EDGE, IRQ_LOW_PRIORITY, (GpioIrqHandler*)IQR_Handling);

	// Place MMA865X in standby mode
	Standby();
	Delay_ms(1);


//	RegisterSet(MMA865X_XYZ_DATA_CFG, 0x10);	// Accelerometer range of +/-2g range with 0.244mg/LSB, high-pass filtered
	RegisterSet(MMA865X_XYZ_DATA_CFG, 0x00);	// Accelerometer range of +/-2g range with 0.244mg/LSB, no high-pass filtered
	Delay_ms(1);

	// Enable interrupts - Open-drain output
	// Interrupt signals - Active high
	// Pulse function interrupt can wake-up system
	// Orientation interrupt can wake-up system
	// Free-fall/motion function interrupt can wake-up system
	RegisterSet(MMA865X_CTRL_REG3, 0x3A);	// 3A: Push-Pull output; 3B: Open-Drain output
	Delay_ms(1);

	// Trigger the FIFO - Freefall/Motion Trigger
	RegisterSet(MMA865X_TRIG_CFG, 0x04);
	Delay_ms(1);

	// Enter active mode
	Active();

	Standby();
	///////////////////////////////////////////////////////
	Sleeping(NULL);
	///////////////////////////////////////////////////////
}

/**
 * Active
 *
 * Enter active mode
 */
static void Active()
{
    // Active=1 to take the part out of standby and enable sampling
    // Data rate = 400 Hz, sleep rate = 12.5 Hz
	RegisterSet(MMA865X_CTRL_REG1, 0x49);
}

/**
 * Stanby
 *
 * Enter stanby mode, only can bring back to the active mode by calling active mode
 */
static void Standby()
{
	RegisterSet(MMA865X_CTRL_REG1, 0);
}

/**
 * Sleeping
 *
 * Enter sleep mode, ready to be brought back to the active mode by an interrupt
 */
static void Sleeping(void* Def)
{
	if (RegisterGet(MMA865X_CTRL_REG4) & SRC_LNDPRT_MASK)
	{
	    // Disable Landscape/Portrait orientation interrupt, which is incompatible
	    // With the current sleep mode
		DisableLndPrtInt();
	    Delay_ms(1);
	}

	// Re-enter active mode first
	Active();
	Delay_ms(2);

	// Enter stand-by mode
	Standby();
	Delay_ms(2);

	RegisterSet(MMA865X_ASLP_COUNT, 1);
	Delay_ms(2);

	RegisterSet(MMA865X_CTRL_REG2, 0x1C);
	Delay_ms(1);

	// Back to active mode
	Active();
}

static void Wakeup(void* Def)
{
	;
}

/**
 * Reset
 *
 * Software reset device
 */
static void Reset()
{
	Standby();
	Delay_ms(1);

	RegisterSet(MMA865X_CTRL_REG2, RST_MASK);
	Delay_ms(1);

	Active();
}

/**
 * Read
 *
 * Read multiple bytes from register
 *
 * @param Register adddress
 * @param Pointer buffer output buffer
 * @param Length amount of bytes to be read
 */
static void Read(uint8_t Register_U8, uint8_t* Data_U8P, uint8_t Length_U8)
{
	tI2CPackage I2CPackage_T;

	I2CPackage_T.I2CMode_T			= I2C_WR;
	I2CPackage_T.DeviceAddress_U16	= cMMA865X_Address;
	I2CPackage_T.Register_16		= Register_U8;
	I2CPackage_T.Data_U8P			= Data_U8P;
	I2CPackage_T.Length_U16			= Length_U8;

	HAL_CORE_TRANSFER(&I2CPackage_T);
}

/**
 * Write
 *
 * Write multiple bytes in register
 *
 * @param Register adddress
 * @param Pointer buffer intput buffer
 * @param Length amount of bytes to be write
 */
static void Write(uint8_t Register_u8, uint8_t* Data_U8P, uint8_t Length_U8)
{
	tI2CPackage I2CPackage_T;

	I2CPackage_T.I2CMode_T			= I2C_WW;
	I2CPackage_T.DeviceAddress_U16	= cMMA865X_Address;
	I2CPackage_T.Register_16		= Register_u8;
	I2CPackage_T.Data_U8P			= Data_U8P;
	I2CPackage_T.Length_U16			= Length_U8;

	HAL_CORE_TRANSFER(&I2CPackage_T);
}

/**
 * RegisterGet
 *
 * Get single byte from register
 *
 * @param Register adddress
 */
static uint8_t RegisterGet(uint8_t Register_U8)
{
	uint8_t Result_U8;
	Read(Register_U8, &Result_U8, 1);

	return Result_U8;
}

/**
 * RegisterSet
 *
 * Set single byte in register
 *
 * @param Register adddress
 * @param Data
 */
static void RegisterSet(uint8_t Register_U8, uint8_t Data_U8)
{
	Write(Register_U8, &Data_U8, 1);
}

/**
 * ReadXYZ
 *
 * Read XYZ axis. Place result in global variable axis
 * Each count equals to 1/1024 gr
 */
static void ReadAxis()
{
	uint8_t Buffer_U8[7];
	uint16_t Temp_U16;

	Read(MMA865X_STATUS_00, Buffer_U8, 7);

	//X axis
	Temp_U16 = ((Buffer_U8[1] << 8) | Buffer_U8[2]) >> 4;

	if (Buffer_U8[1] > 0x7F)
		mAxisX = -((~Temp_U16 + 1) & 0xFFF);
	else
		mAxisX = Temp_U16;

	//Y axis
	Temp_U16 = ((Buffer_U8[3] << 8) | Buffer_U8[4]) >> 4;

	if (Buffer_U8[3] > 0x7F)
		mAxisY = -((~Temp_U16 + 1) & 0xFFF);
	else
		mAxisY = Temp_U16;

	//Z axis
	Temp_U16 = ((Buffer_U8[5] << 8) | Buffer_U8[6]) >> 4;

	if (Buffer_U8[5] > 0x7F)
		mAxisZ = -((~Temp_U16 + 1) & 0xFFF);
	else
		mAxisZ = Temp_U16;
}

static void EnableInterrupt(uint8_t Source_U8)
{
	tAccelerometerDef Config;
	GetConfig(&Config);

	uint8_t i = 0;

	do
	{
		if (Source_U8 & 0x01)
		{
			switch(1 << i)
			{
			case SRC_FF_MT_MASK:
				Accelerometer_HAL_T.AccelerometerData_T->ACLDectect_T.FF_MT = 0;
				EnableFfMtInt(Config.FreefallMotion_T);
				break;

			case SRC_PULSE_MASK:
				Accelerometer_HAL_T.AccelerometerData_T->ACLDectect_T.PULSE = 0;
				EnableTapInt(Config.Pulse_T);
				break;

			case SRC_LNDPRT_MASK:
				Accelerometer_HAL_T.AccelerometerData_T->ACLDectect_T.LNDPRT = 0;
				EnableLndPrtInt(Config.LandscapePortait_T);
				break;

			}
		}

		Source_U8 >>= 1;
		i++;
	}
	while(i < 8);
}

static void DisableInterrupt(uint8_t Source_U8)
{
	uint8_t i = 0;

	do
	{
		if (Source_U8 & 0x01)
		{
			switch(1 << i)
			{
			case SRC_FF_MT_MASK:
				DisableFfMtInt();
				break;

			case SRC_PULSE_MASK:
				DisableTapInt();
				break;

			case SRC_LNDPRT_MASK:
				DisableLndPrtInt();
				break;

			}
		}

		Source_U8 >>= 1;
		i++;
	}
	while(i < 8);
}

/**
 * GetAndClearSourceINT
 *
 * Get source interrupts and clear interrupt flags
 *
 * @return Ret_U8P: (0x80: SRC_ASLP,  0x40: SRC_FIFO,  0x20 SRC_TRANS, 0x10: SRC_LNDPRT,
 *   				 0x08: SRC_PULSE, 0x04: SRC_FF_MT, 				   0x01: SRC_DRDY)
 */
static void GetAndClearSourceINT(uint8_t* Source_U8P)
{
	*Source_U8P = RegisterGet(MMA865X_INT_SOURCE);

	uint8_t SourceTemp_U8 = *Source_U8P;
	uint8_t i = 0;

	do
	{
		if (SourceTemp_U8 & 0x01)
		{
			switch(1 << i)
			{
			case SRC_FF_MT_MASK:
				RegisterGet(MMA865X_FF_MT_SRC);
				break;

			case SRC_PULSE_MASK:
				RegisterGet(MMA865X_PULSE_SRC);
				break;

			case SRC_LNDPRT_MASK:
				RegisterGet(MMA865X_PL_STATUS);
				break;

			case SRC_TRANS_MASK:
				RegisterGet(MMA865X_TRANSIENT_SRC);
				break;

			case SRC_FIFO_MASK:
				RegisterGet(MMA865X_STATUS_00);
				break;
			}
		}

		SourceTemp_U8 >>= 1;
		i++;
	}
	while(i < 8);
}

/**
 * EnableIntRegister
 *
 * Enable interrupt and attach interrupt to output pin
 *
 * @param Source of interrupt - mask (SRC_ASLP_MASK, SRC_FIFO_MASK, SRC_TRANS_MASK
 *   SRC_LNDPRT_MASK, SRC_PULSE_MASK, SRC_FF_MT_MASK, SRC_DRDY_MASK)
 */
static void EnableIntRegister(uint8_t Source_U8, bool PinINT)
{
	CtrlReg4 |= Source_U8;
	RegisterSet(MMA865X_CTRL_REG4, CtrlReg4);

	if (PinINT)
		CtrlReg5 &= ~Source_U8;
	else
		CtrlReg5 |= Source_U8;

	RegisterSet(MMA865X_CTRL_REG5, CtrlReg5);
}

/**
 * DisableIntRegister
 *
 * Disable interrupt
 *
 * @param Source of interrupt - mask (SRC_ASLP_MASK, SRC_FIFO_MASK, SRC_TRANS_MASK
 *   SRC_LNDPRT_MASK, SRC_PULSE_MASK, SRC_FF_MT_MASK, SRC_DRDY_MASK)
 */
static void DisableIntRegister(uint8_t Source_U8)
{
	CtrlReg4 &= ~Source_U8;
	RegisterSet(MMA865X_CTRL_REG4, CtrlReg4);

	CtrlReg5 &= ~Source_U8;
	RegisterSet(MMA865X_CTRL_REG5, CtrlReg5);
}

/**
 * EnableFfMtInt
 *
 * Enable Freefall/Motion interrupt and attach interrupt to output pin
 *
 * @param Def.PinInt 0: INT1 pin, 1: INT2 pin
 * @param Def.Mode 0: Freefall, 1: Motion
 * @param Def.Sensibility 1 is the highest, 127 is the lowest sensibility
 */
static void EnableFfMtInt(tFreefallMotionDef Def)
{
	// Enter stand-by mode
	Standby();
	Delay_ms(1);

	EnableIntRegister(SRC_FF_MT_MASK, Def.PinINT);
	Delay_ms(1);

	// Enable single pulse detection on each axis
	if (Def.Mode)
		RegisterSet(MMA865X_FF_MT_CFG, 0xF8); // Motion detection - XYZ events enabled
	else
		RegisterSet(MMA865X_FF_MT_CFG, 0xB8); // Freefall detection - XYZ events enabled

	Delay_ms(1);

	RegisterSet(MMA865X_FF_MT_THS, Def.Sensibility_U8);
	Delay_ms(1);

	// Back to active mode
	Active();
}

/**
 * DisableFfMtInt
 *
 * Disable Freefall/Motion interrupt
 */
static void DisableFfMtInt()
{
	// Enter stand-by mode
	Standby();
	Delay_ms(1);

	DisableIntRegister(SRC_FF_MT_MASK);
	Delay_ms(1);

	// Disable Freefall/Motion detection
	RegisterSet(MMA865X_FF_MT_CFG, 0);
	Delay_ms(1);

	// Back to active mode
	Active();
}

/**
 * EnableTapInt
 *
 * Enable tap interrupt and attach interrupt to output pin
 *
 * @param Def.PinInt 0: INT1 pin, 1: INT2 pin
 * @param Def.Mode 0: Tap, 1: DoubleTap_B
 * @param Def.Sensibility 1 is the highest, 127 is the lowest sensibility
 */
static void EnableTapInt(tPulseDef Def)
{
	uint8_t Config_U8;

	// Enter stand-by mode
	Standby();
	Delay_ms(1);

	EnableIntRegister(SRC_PULSE_MASK, Def.PinINT);
	Delay_ms(1);

	if (Def.Mode)	// Double Tap
		Config_U8 = 0x2A;
	else
		Config_U8 = 0x15;

	// Enable single pulse detection on each axis
	RegisterSet(MMA865X_PULSE_CFG, Config_U8);
	Delay_ms(1);

	RegisterSet(MMA865X_PULSE_THSX, Def.Sensibility_U8);
	Delay_ms(1);

	RegisterSet(MMA865X_PULSE_THSY, Def.Sensibility_U8);
	Delay_ms(1);

	RegisterSet(MMA865X_PULSE_THSZ, Def.Sensibility_U8);
	Delay_ms(1);

	// Back to active mode
	Active();
}

/**
 * DisableTapInt
 *
 * Disable tap interrupt
 */
static void DisableTapInt()
{
	// Enter stand-by mode
	Standby();
	Delay_ms(1);

	DisableIntRegister(SRC_PULSE_MASK);
	Delay_ms(1);

	// Disable Pulse detection
	RegisterSet(MMA865X_PULSE_CFG, 0);
	Delay_ms(1);

	// Back to active mode
	Active();
}

/**
 * EnableLndPrlInt
 *
 * Enable portrait-landscape orientation interrupt and attach interrupt to output pin
 *
 * @param Def.PinInt 0: INT1 pin, 1: INT2
 */
static void EnableLndPrtInt(tLandscapePortaitDef Def)
{
	// Enter stand-by mode
	Standby();
	Delay_ms(1);

	EnableIntRegister(SRC_LNDPRT_MASK, Def.PinINT);
	Delay_ms(1);

	// Enable Portrait/Ladscape orientation detection
	RegisterSet(MMA865X_PL_CFG, PL_EN_MASK);
	Delay_ms(1);

	// Back to active mode
	Active();
}

/**
 * DisableLndPrlInt
 *
 * Disable landscape-portrait orientation interrupt
 */
static void DisableLndPrtInt()
{
	// Enter stand-by mode
	Standby();
	Delay_ms(1);

	DisableIntRegister(SRC_LNDPRT_MASK);
	Delay_ms(1);

	// Disable Portrait/Ladscape orientation detection
	RegisterSet(MMA865X_PL_CFG, 0);
	Delay_ms(1);

	// Back to active mode
	Active();
}


