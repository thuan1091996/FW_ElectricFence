/*
 * Voltage.h
 *
 *  Created on: Dec 30, 2019
 *      Author: DFMPC-01201117
 */

#ifndef IOTLIB_HAL_HARDWARES_ANALOG_H_
#define IOTLIB_HAL_HARDWARES_ANALOG_H_

#include "../../../common/common.h"

/*****************************************************************************************************************/
/*												 MACROS DEFINITION												 */
/*****************************************************************************************************************/
/*TODO: check gpio init*/

#define ADC_ENABLE_PIN() { 					\
		.pin	= PA_1,						\
		.mode	= PIN_OUTPUT,				\
		.pull	= PIN_NO_PULL,				\
}


#define ADC_CHANNEL_PIN() { 				\
		.pin	= PA_3,						\
		.mode	= PIN_ANALOGIC,				\
		.pull	= PIN_NO_PULL,				\
}
/*****************************************************************************************************************/
/*													DATA-STRUCTS												 */
/*****************************************************************************************************************/


#endif /* IOTLIB_HAL_HARDWARES_ANALOG_H_ */
