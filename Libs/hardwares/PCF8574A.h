/*
 * PCF8574.h
 *
 *  Created on: Aug 13, 2021
 *      Author: ChienNguyen
 */

#ifndef IOTLIB_HAL_HARDWARES_PCF8574A_H_
#define IOTLIB_HAL_HARDWARES_PCF8574A_H_

#include "stdint.h"
/*****************************************************************************************************************/
/*													CONSTANTS DEFINITION										 */
/*****************************************************************************************************************/

#define PCF8574A_CLK_kHZ	100

#define A2 0
#define A1 0
#define A0 0

#if (A2 == 0) & (A1 == 0) & (A0 == 0)
#define PCF8574A_ADR			0x71
#else
#error "Ref docs"
#endif

#define MODE_INPUT		0
#define MODE_OUTPUT		1

#define P0	(1 << 0)
#define P1	(1 << 1)
#define P2	(1 << 2)
#define P3	(1 << 3)

#define P4	(1 << 4)
#define P5	(1 << 5)
#define P6	(1 << 6)
#define P7	(1 << 7)

#define IS_PCF8574A_PIN(PIN)		((PIN >= 0) && (PIN <= 7))

#ifndef assert_param
#define assert_param(expr)
#endif

typedef enum
{
	INPUT,
	OUTPUT,
	EXT_IRQ
} eGPIOMode;
/*****************************************************************************************************************/
/*													COMMANDS LIST												 */
/*****************************************************************************************************************/


#endif /* IOTLIB_HAL_HARDWARES_PCF8574A_H_ */
