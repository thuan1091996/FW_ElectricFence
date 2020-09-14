/*
 * Voltage.c
 *
 *  Created on: Dec 30, 2019
 *      Author: DFMPC-01201117
 */

#include <HAL/hardwares/Analog.h>
#include "../HAL.h"

/*****************************************************************************************************************/
/*											   PRIVATE VARIABLES												 */
/*****************************************************************************************************************/
static tGPIODef ADC_GPIO_EN_T = ADC_ENABLE_PIN();
static tGPIODef ADC_GPIO_CHL_T = ADC_CHANNEL_PIN();

/*****************************************************************************************************************/
/*												GLOBAL VARIABLES												 */
/*****************************************************************************************************************/
extern tADC_HAL ADC_HAL_T;


/*****************************************************************************************************************/
/*											   FUNCTION IMPLEMENT										 	 	 */
/*****************************************************************************************************************/
static void Initialize(void)
{
	ADC_HAL_T.Enable_B = 1;
	ADC_HAL_T.DeviceStatus_E = Activated;

	ADC_HAL_T.CORE_SERVICES_T.GpioHAL()->Initialize(&ADC_GPIO_EN_T, RESET);

#ifndef APP_ORANGE
	ADC_HAL_T.CORE_SERVICES_T.Initialize(&ADC_GPIO_CHL_T);
#endif
}

static float GetData()
{
	uint16_t ADCData = 0;
	float Vol = 0.0;

#ifdef APP_ORANGE
	uint16_t VREFINT_CAL = (*(uint16_t*)0x1FF80078);
	uint16_t VREFINT_DATA = ADC_HAL_T.CORE_SERVICES_T.ReadDataChannel(NULL);
#endif

	ADC_HAL_T.CORE_SERVICES_T.GpioHAL()->Write(&ADC_GPIO_EN_T, SET);
	ADCData = ADC_HAL_T.CORE_SERVICES_T.ReadDataChannel(&ADC_GPIO_CHL_T);
	ADC_HAL_T.CORE_SERVICES_T.GpioHAL()->Write(&ADC_GPIO_EN_T, RESET);

	//TODO: check and convert data raw to battery voltage

#ifdef APP_ORANGE
	Vol = ((float) ADCData / 4095) * (3.0 * VREFINT_CAL / VREFINT_DATA) * (400.0 / 100.0) + 0.004; // todo: vref
	Vol *= 1.05;
#else
	#ifdef AAAA
		Vol = ((float) ADCData / 4095) * 2.5 * (2.789 - (0.04 * ((2291 - ADCData) / 23) )); //pin 3.7 vol
	#else
		Vol = ((float) ADCData / 4095) * 2.5 * (3.9595 - (0.0123 * ((2731 - ADCData) / 33) )); // 4 pin AA
	#endif
#endif

	return Vol;
}

static void Sleeping(void *Def)
{
	ADC_HAL_T.CORE_SERVICES_T.GpioHAL()->Initialize(&ADC_GPIO_EN_T, RESET);
}

static void Wakeup(void *Def)
{
	;
}

/*****************************************************************************************************************/
/*											 HARDWARE ABSTRACT LAYER								 			 */
/*****************************************************************************************************************/
void __attribute__((weak)) HAL_ADC_Interaction(tADC_HAL *HAL)
{
	HAL->SENSOR_SERVICES_T.Initialize = &Initialize;
	HAL->SENSOR_SERVICES_T.GetData	  = &GetData;
	HAL->SENSOR_SERVICES_T.Sleeping	  = &Sleeping;
	HAL->SENSOR_SERVICES_T.Wakeup	  = &Wakeup;

	Initialize();
}
