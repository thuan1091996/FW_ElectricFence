/*
 * MC36xx.c
 *
 *  Created on: Oct 27, 2021
 *      Author: quocdt
 */

#include "MC36xx.h"
#include "i2c.h"
//#include "../HAL.h"
//#include "../../../../app/appdef.h"

//#ifdef ACL_MODULE

/*****************************************************************************************************************/
/*													GLOBAL VARIABLES											 */
/*****************************************************************************************************************/
//extern tAccelerometer_HAL Accelerometer_HAL_T;
//
//static tGPIODef GPIO_Wakeup_T = PIN_ACL_EXTI();

/*****************************************************************************************************************/
/*													PRIVATE FUNCTION											 */
/*****************************************************************************************************************/


/*****************************************************************************************************************/
/*											HARDWARE ABSTRACT LAYER									 			 */
/*****************************************************************************************************************/
//void __attribute__((weak)) HAL_Accelerometer_Interaction(tAccelerometer_HAL *HAL)
//{
////
////	HAL->AccelerometerType_E = MMA8652;
////
//	HAL->SENSOR_SERVICES_T.Initialize			= &AccelerometerInit;
//	HAL->SENSOR_SERVICES_T.ActiveMode			= &Active;
//	HAL->SENSOR_SERVICES_T.StandByMode			= &Standby;
//	HAL->SENSOR_SERVICES_T.Reset				= &Reset;
////	HAL->SENSOR_SERVICES_T.Read					= &Read;
////	HAL->SENSOR_SERVICES_T.Write				= &Write;
////	HAL->SENSOR_SERVICES_T.ReadAxis				= &ReadAxis;
//	HAL->SENSOR_SERVICES_T.Sleeping				= &Sleeping;
////	HAL->SENSOR_SERVICES_T.Wakeup				= &Wakeup;
//	HAL->SENSOR_SERVICES_T.EnableInterrupt		= &EnableInterrupt;
////	HAL->SENSOR_SERVICES_T.DisableInterrupt		= &DisableInterrupt;
////	HAL->SENSOR_SERVICES_T.GetAndClearSourceINT	= &GetAndClearSourceINT;
//}

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

static void MC3635_Setting();

MC3635_status_t  MC3635_SetMode ( MC3635_mode_c_mctrl_t eMode, MC3635_power_mode_t ePowerMode, MC3635_sample_rate_t eODR );

static void SniffSetThreshold (uint8_t Threshold_U8, uint8_t DetectCount_U8, MC3635_sniffth_c_sniff_and_or_t logic, MC3635_sniffth_c_sniff_mode_t mode);

MC3635_status_t  MC3635_SetFIFO (uint8_t NumberOfSamples, MC3635_fifo_c_fifo_mode_t FIFO_Mode );

MC3635_status_t  MC3635_EnableFIFO ( MC3635_fifo_c_fifo_en_t FIFO_Enable );

/*****************************************************************************************************************/
/*													FUNCTION IMPLEMENT									 	 	 */
/*****************************************************************************************************************/

/**
 * Init
 *
 * Initialize MMA8652 sensor
 */
bool MC36xx_Init()
{
	if (!HAL_I2C_IsDeviceReady(&hi2c1, MC3635_ADDRESS_LOW<<1, 3, 100))
	{


	//Place MMA865X in standby mode
		Standby();
		//HAL_Delay(10);

		RegisterSet(MC3635_FREG_1, FREG_1_I2C_EN_ENABLED);
		//HAL_Delay(10);

		RegisterSet(MC3635_INIT_1, INIT_1_INIT_1_FIXED_VALUE);
		//HAL_Delay(10);

		RegisterSet(MC3635_INIT_2, INIT_2_INT_2_FIXED_VALUE);
		//HAL_Delay(10);

		RegisterSet(MC3635_INIT_3, INIT_3_INT_3_FIXED_VALUE);
		//HAL_Delay(10);

		uint8_t int_source=0;
		GetAndClearSourceINT(&int_source);

		MC3635_Setting();

		EnableInterrupt();

		Active();

		HAL_Delay(2000);

		GetAndClearSourceINT(&int_source);
		if(int_source == 0x04)
		{
			return true;
		}
	}
	return false;
}

/**
 * Stanby
 *
 * Enter stanby mode, only can bring back to the active mode by calling active mode
 */
static void Standby()
{
	uint8_t cmd = RegisterGet(MC3635_MODE_C);
	cmd &= ~MODE_C_MCTRL_MASK;
	cmd |= MODE_C_MCTRL_STANDBY;

	RegisterSet(MC3635_MODE_C, cmd);
}

static void MC3635_Setting()
{
	uint8_t ptr;

	Standby();

	RegisterSet(MC3635_RANGE_C, RANGE_C_RANGE_2G | RANGE_C_RES_14_BITS);
	//HAL_Delay(1);

	ptr = RegisterGet(MC3635_MODE_C);
	ptr &= ~(MODE_C_X_AXIS_PD_MASK | MODE_C_Y_AXIS_PD_MASK | MODE_C_Z_AXIS_PD_MASK);
	ptr |= MODE_C_X_AXIS_PD_ENABLED | MODE_C_Y_AXIS_PD_ENABLED | MODE_C_Z_AXIS_PD_ENABLED;

	RegisterSet(MC3635_MODE_C, ptr);
	//HAL_Delay(1);
	RegisterSet(MC3635_RATE_1, ODR_7);

	RegisterSet(MC3635_FREG_1, FREG_1_I2C_EN_ENABLED);
	//HAL_Delay(1);

	RegisterSet(MC3635_FREG_2, FREG_2_I2CINT_WRCLRE_DISABLED);
	//HAL_Delay(1);

	SniffSetThreshold(1, 1,SNIFFTH_C_SNIFF_AND_OR_OR_ENABLED,SNIFFTH_C_SNIFF_MODE_C2P_ENABLED);
	//HAL_Delay(1);
}

static void Active()
{
	Standby();
	//HAL_Delay(1);

	MC3635_SetMode(MODE_C_MCTRL_SNIFF,ULTRA_LOW_POWER_MODE,ODR_7);
	//HAL_Delay(1);
}

static void Sleeping(void *ptr)
{
	Standby();
	//HAL_Delay(1);

	uint8_t cmd = RegisterGet(MC3635_MODE_C);
	cmd &= ~MODE_C_MCTRL_MASK;
	cmd |= MODE_C_MCTRL_SLEEP;

	RegisterSet(MC3635_MODE_C, cmd);
	//HAL_Delay(10);
}

/**
 * Reset
 *
 * Software reset device
 */
static void Reset()
{
	Standby();
	//HAL_Delay(1);

	RegisterSet(MC3635_RESET, RESET_RESET_FORCE_POWER_ON_RESET);
	//HAL_Delay(1);

	Active();
}

static void SniffSetThreshold (uint8_t Threshold_U8, uint8_t DetectCount_U8, MC3635_sniffth_c_sniff_and_or_t logic, MC3635_sniffth_c_sniff_mode_t mode)
{
	//0<= Threshold <= 63
	//0< DetectCount_U8 <= 62
	Standby();

	//HAL_Delay(1);

	RegisterSet(MC3635_SNIFFCF_C, SNIFFCF_C_SNIFF_THADR_SNIFF_THRESHOLD_X_AXIS);
	RegisterSet(MC3635_SNIFFTH_C, Threshold_U8);

	RegisterSet(MC3635_SNIFFCF_C, SNIFFCF_C_SNIFF_THADR_SNIFF_DETECTION_X_AXIS);
	RegisterSet(MC3635_SNIFFTH_C, DetectCount_U8);

	RegisterSet(MC3635_SNIFFCF_C, SNIFFCF_C_SNIFF_THADR_SNIFF_THRESHOLD_Y_AXIS);
	RegisterSet(MC3635_SNIFFTH_C, Threshold_U8);

	RegisterSet(MC3635_SNIFFCF_C, SNIFFCF_C_SNIFF_THADR_SNIFF_DETECTION_Y_AXIS);
	RegisterSet(MC3635_SNIFFTH_C, DetectCount_U8);

	RegisterSet(MC3635_SNIFFCF_C, SNIFFCF_C_SNIFF_THADR_SNIFF_THRESHOLD_Z_AXIS);
	RegisterSet(MC3635_SNIFFTH_C, Threshold_U8);

	RegisterSet(MC3635_SNIFFCF_C, SNIFFCF_C_SNIFF_THADR_SNIFF_DETECTION_Z_AXIS);
	RegisterSet(MC3635_SNIFFTH_C, logic|mode|DetectCount_U8);
}

static void MC3635_Conf_INTR(MC3635_intr_c_ipp_t INTN_Mode, MC3635_intr_c_iah_t INTN_Level )
{
	uint8_t ptr;
	ptr = RegisterGet(MC3635_INTR_C);

	ptr &= ~(INTR_C_IAH_MASK | INTR_C_IPP_MASK);
	ptr |= INTN_Mode| INTN_Level;

	RegisterSet(MC3635_INTR_C, ptr);
	//HAL_Delay(1);
}

static void MC3635_INTR_SET(uint8_t Intn_U8)
{
	uint8_t ptr;
	ptr = RegisterGet(MC3635_INTR_C);

	ptr &= ~( INTR_C_INT_WAKE_MASK | INTR_C_INT_ACQ_MASK | INTR_C_INT_FIFO_EMPTY_MASK | INTR_C_INT_FIFO_FULL_MASK | INTR_C_INT_FIFO_THRESH_MASK | INTR_C_INT_SWAKE_MASK );
	ptr |= Intn_U8;

	RegisterSet(MC3635_INTR_C, ptr);
}

MC3635_status_t  MC3635_SetMode ( MC3635_mode_c_mctrl_t eMode, MC3635_power_mode_t ePowerMode, MC3635_sample_rate_t eODR )
{
    uint8_t         reg =0, para = 0;

    /* CHECK INVALID MODES  */
    // TRIG, STANDBY and SLEEP mode have their own function, do NOT use this one!
    if ( ( eMode == MODE_C_MCTRL_STANDBY ) || ( eMode == MODE_C_MCTRL_SLEEP ) || ( eMode == MODE_C_MCTRL_TRIG ) )
    {
        return MC3635_FAILURE;
    }

    /* POWER MODE   */
    // Get the register data
    reg  =   MC3635_PMCR;
//    aux     =   RegisterSet ( myI2Cparameters, &reg, 1, I2C_NO_STOP_BIT  );
    para     =   RegisterGet  ( reg );

    if ( eMode == MODE_C_MCTRL_SNIFF )
    {
        para &=  ~PMCR_SPM_MASK;
        para |=   ( ePowerMode << 4U );
    }
    else
    {
        para &=  ~PMCR_CSPM_MASK;
        para |=   ( ePowerMode << 0U );
    }

    // Update the register
    RegisterSet ( reg, para );

    /*  SAMPLE RATE FOR THE GIVEN MODE */
    // SNIFF mode
    if ( eMode == MODE_C_MCTRL_SNIFF )
    {
        reg  =   MC3635_SNIFF_C;

        switch ( ePowerMode )
        {
        case ULTRA_LOW_POWER_MODE:
            if ( ( eODR == ODR_13 ) || ( eODR == ODR_14 ) )
            {
                return MC3635_FAILURE;
            }
            break;

        default:
        case LOW_POWER_MODE:
            if ( ( eODR == ODR_12 ) || ( eODR == ODR_13 ) || ( eODR == ODR_14 ) )
            {
                return MC3635_FAILURE;
            }
            break;

        case PRECISION:
            if ( ( eODR == ODR_9 ) || ( eODR == ODR_10 ) || ( eODR == ODR_11 ) || ( eODR == ODR_12 ) || ( eODR == ODR_13 ) || ( eODR == ODR_14 ) )
            {
                return MC3635_FAILURE;
            }
            break;
        }
    }
    // CWAKE and SWAKE mode
    else
    {
        reg  =   MC3635_RATE_1;

        if ( ( eODR == ODR_0 ) || ( eODR == ODR_1 ) || ( eODR == ODR_2 ) || ( eODR == ODR_3 ) || ( eODR == ODR_4 ) || ( eODR == ODR_13 ) || ( eODR == ODR_14 ) )
        {
            return MC3635_FAILURE;
        }
        else
        {
            switch ( ePowerMode )
            {
            case ULTRA_LOW_POWER_MODE:
                if ( eODR == ODR_5 )
                {
                    return MC3635_FAILURE;
                }
                break;

            default:
            case LOW_POWER_MODE:
                if ( eODR == ODR_12 )
                {
                    return MC3635_FAILURE;
                }
                break;

            case PRECISION:
                if ( ( eODR == ODR_9 ) || ( eODR == ODR_10 ) || ( eODR == ODR_11 ) || ( eODR == ODR_12 ) )
                {
                    return MC3635_FAILURE;
                }
                break;
            }
        }
    }


    // Check if Specific setup steps are required
    if ( eODR == ODR_15 )
    {
        // Previous steps ( Step 1, Step 2 and Step 3 ) have to be implemented by the user before calling this function
        // Step 4 is implemented at the begging of this function.

        // Step 5: Point to set wake/sniff settings
        reg  =   MC3635_RATE_1;
        para  =   0x10;
        RegisterSet(reg, para);

        if ( reg == MC3635_SNIFF_C )
        {
            // Step 6: Rate 15 setup 1 for sniff
            reg  =   MC3635_TRIGC;
            para  =   0x30;
            RegisterSet ( reg, para);

            // Step 7: Rate 15 setup 2 for sniff
            reg  =   MC3635_RATE_1;
            para  =   0x30;
            RegisterSet ( reg, para);

            // Step 8: Rate 15 setup 3 for sniff
            reg  =   MC3635_TRIGC;
            para  =   0x01;
            RegisterSet ( reg, para);

            // Step 9: Point to value 1
            reg  =   MC3635_RATE_1;
            para  =   0x60;
            RegisterSet ( reg, para);

            // Step 10: Write value 1
            reg  =   MC3635_TRIGC;
            switch ( ePowerMode )
            {
                case ULTRA_LOW_POWER_MODE:
                    para  =   0x52;
                    break;

                default:
                case LOW_POWER_MODE:
                    para  =   0x72;
                    break;

                case PRECISION:
                    para  =   0x32;
                    break;
            }

            RegisterSet ( reg, para);

            // Step 11: Point to value 2
            reg  =   MC3635_RATE_1;
            para  =   0x70;
            RegisterSet ( reg, para);
        }
        else
        {
            // Step 6: Rate 15 setup 1 for the given device mode
            reg  =   MC3635_TRIGC;
            para  =   0x03;
            RegisterSet ( reg, para);

            // Step 7: Rate 15 setup 2 for the given device mode
            reg  =   MC3635_RATE_1;
            para  =   0x20;
            RegisterSet ( reg, para);

            // Step 8: Rate 15 setup 3 for the given device mode
            reg  =   MC3635_TRIGC;
            para  =   0x01;
            RegisterSet ( reg, para);

            // Step 9: Point to value 1
            reg  =   MC3635_RATE_1;
            para  =   0x40;
            RegisterSet ( reg, para);

            // Step 10: Write value 1
            reg  =   MC3635_TRIGC;
            switch ( ePowerMode )
            {
                case ULTRA_LOW_POWER_MODE:
                    para  =   0x52;
                    break;

                default:
                case LOW_POWER_MODE:
                    para  =   0x72;
                    break;

                case PRECISION:
                    para  =   0x32;
                    break;
            }

            RegisterSet ( reg, para);

            // Step 11: Point to value 2
            reg  =   MC3635_RATE_1;
            para  =   0x50;
            RegisterSet ( reg, para);
        }

        // Step 12: Write value 2
        reg  =   MC3635_TRIGC;
        switch ( ePowerMode )
        {
            case ULTRA_LOW_POWER_MODE:
                para  =   0x01;
                break;

            default:
            case LOW_POWER_MODE:
                para  =   0x02;
                break;

            case PRECISION:
                para  =   0x12;
                break;
        }
        RegisterSet ( reg, para);

        // Step 13: Apply the values
        reg  =   MC3635_RATE_1;
        para  =   0x0F;
        RegisterSet ( reg, para);
    }
    else
    {
        // Get the register data
        para     =   RegisterGet  ( reg );


        // Update the register data for the specific mode
        if ( reg == MC3635_SNIFF_C )
        {
            para &=  ~SNIFF_C_SNIFF_SR_MASK;
            para |=   eODR;
        }
        else
        {
            para &=  ~RATE_1_RR_MASK;
            para |=   eODR;
        }

        RegisterSet ( reg, para);
    }


    /* DEVICE MODE */
    // Step 14: Go to the given device mode
    reg  =   MC3635_MODE_C;
    para     =   RegisterGet  ( reg );

    para &=  ~MODE_C_MCTRL_MASK;
    para |=   eMode;
    RegisterSet ( reg, para);

    return MC3635_SUCCESS;
}

static void EnableInterrupt(void)
{
	Standby();

	MC3635_Conf_INTR(INTR_C_IPP_PUSH_PULL_MODE, INTR_C_IAH_ACTIVE_HIGH);
	//HAL_Delay(1);

	MC3635_INTR_SET(INTR_C_INT_WAKE_ENABLED);
	//HAL_Delay(1);
}

static void DisableInterrupt(void)
{
	Standby();

	MC3635_INTR_SET(0x00);
	//HAL_Delay(1);
}

void GetAndClearSourceINT(uint8_t* Source_U8P)
{
	*Source_U8P = RegisterGet(MC3635_STATUS_2);
}

MC3635_status_t  MC3635_SetFIFO (uint8_t NumberOfSamples, MC3635_fifo_c_fifo_mode_t FIFO_Mode )
{
    uint8_t para = 0;
    // Check FIFO number of samples
    if ( ( NumberOfSamples < 1 ) || ( NumberOfSamples > 31 ) )
    {
        return   MC3635_FAILURE;
    }

    // Get the register data
    para     =   RegisterGet(MC3635_FIFO_C);

    // Update the register data
    para	    &=  ~( FIFO_C_FIFO_TH_MASK | FIFO_C_FIFO_MODE_MASK );
    para		|=   ( NumberOfSamples | FIFO_Mode );
    RegisterSet(MC3635_FIFO_C, para);
}

MC3635_status_t  MC3635_EnableFIFO ( MC3635_fifo_c_fifo_en_t FIFO_Enable )
{
    uint8_t para = 0;

    // Get the register data
    para     =   RegisterGet(MC3635_FIFO_C);

    // Update the register data
    para	 &=  ~FIFO_C_FIFO_EN_MASK;
    para	 |=   FIFO_Enable;
    RegisterSet(MC3635_FIFO_C, para);
}

void ReadAxis(uint16_t* x, uint16_t* y, uint16_t* z)
{
	uint8_t ret[6];
	Read(MC3635_XOUT_LSB, ret, sizeof(ret));

	*x |= *(uint16_t *)ret;
	*y |= *(uint16_t *)(ret + 2);
	*z |= *(uint16_t *)(ret + 4);
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

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MC3635_ADDRESS_LOW<<1, Register_U8, 1, Data_U8P, Length_U8, 1000);

//	Accelerometer_HAL_T.CORE_SERVICES_T.Driver_TP->TxSend(&I2CPackage_T, NULL);
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
static void Write(uint8_t Register_U8, uint8_t* Data_U8P, uint8_t Length_U8)
{

	if(HAL_OK == HAL_I2C_Mem_Write(&hi2c1, MC3635_ADDRESS_LOW<<1, Register_U8, 1, Data_U8P, Length_U8, 1000))
	{
	}

//	Accelerometer_HAL_T.CORE_SERVICES_T.Driver_TP->TxSend(&I2CPackage_T, NULL);
}

/**
 * RegisterGet
 *
 * Get single byte from register
 *
 * @param Register adddress
 */
uint8_t RegisterGet(uint8_t Register_U8)
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
void RegisterSet(uint8_t Register_U8, uint8_t Data_U8)
{
	Write(Register_U8, &Data_U8, 1);
}

