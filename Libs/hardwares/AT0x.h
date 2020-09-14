/*
 * AT0x.h
 * x = 4 or 8
 *
 *  Created on: Sep 19, 2019
 *      Author: DFMPC-01201117
 */

#ifndef SRC_TEMPLATES_HARDWAREABSTRACTLAYER_AT0X_H_
#define SRC_TEMPLATES_HARDWAREABSTRACTLAYER_AT0X_H_



/*****************************************************************************************************************/
/*												  CONSTANTS DEFINITION											 */
/*****************************************************************************************************************/

#define cCMDFlash					(0xA0)

#ifdef FLASH_AT04
	#define cFlashSize_U16          	((uint16_t)512)
	#define cPageSize_U16				((uint16_t)16)
#else
	#define cFlashSize_U16          	((uint16_t)1024)
	#define cPageSize_U16				((uint16_t)8)
#endif

#define cPageNum_U16				((uint16_t)cFlashSize_U16 / cPageSize_U16)

/*****************************************************************************************************************/
/*													MACROS DEFINITION											 */
/*****************************************************************************************************************/

#define mFlashAddress(MemeoryAddress)		(cCMDFlash | ((MemeoryAddress >> 7) & 0x06)) //bit 10, 9 of memory address is bit 2, 1 of device address
#define mMemoryAddress(MemeoryAddress)		(MemeoryAddress & 0xFF)

/*****************************************************************************************************************/
/*													   DATA-STRUCTS												 */
/*****************************************************************************************************************/

#endif /* SRC_TEMPLATES_HARDWAREABSTRACTLAYER_AT0X_H_ */
