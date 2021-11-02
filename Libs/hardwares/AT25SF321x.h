/*
 * AT25SF321x.h
 *
 *  Created on: Aug 11, 2021
 *      Author: DFMPC-
 */

#ifndef IOTLIB_HAL_HARDWARES_AT25SF321X_H_
#define IOTLIB_HAL_HARDWARES_AT25SF321X_H_

/*****************************************************************************************************************/
/*												  INTEGRATION													 */
/*****************************************************************************************************************/


/*****************************************************************************************************************/
/*												  MACRO UTILITIES												 */
/*****************************************************************************************************************/
#if QSPI
#define mSPIFlashAdd(MemoryAddress,Mode)			((((uint32_t)MemoryAddress)<<8) |(Mode))
#define INIT_BUSTXFER(o,ad,da,id)					{o-1,ad-1,da-1,id-1}
#endif

/*****************************************************************************************************************/
/*												  CONSTANTS DEFINITION											 */
/*****************************************************************************************************************/
#define MANUFACTURE_DEVICE_ID			0x0001871F

#define AT25SF321X_START_MEM 			0x000000
#define AT25SF321X_END_MEM				0x3FFFFF
#define AT25SF321X_PAGE_SIZE     		256

#define MASK_BLK_256B					((0xFFFFFFFF) << 8)
#define MASK_BLK_4K						((0xFFFFFFFF) << 12)
#define MASK_BLK_32K					((0xFFFFFFFF) << 15)
#define MASK_BLK_64K					((0xFFFFFFFF) << 16)

#define cBLOCK_256B						(AT25SF321X_PAGE_SIZE)
#define cBLOCK_4KB						(1*4*1024)
#define cBLOCK_32KB						(2*4*4*1024)
#define cBLOCK_64KB						(4*4*4*1024)

#define INSTRUCTION_SIZE				1
#define ADDRESS_SIZE					3
#define DATA_SIZE						8
#define DUMMY_SIZE						x
#define INITCMD_SIZE					x

#define WRAP_BYTE						0x10
#define WRAP_HWORD						0x20
#define WRAP_WORD						0x40
#define WRAP_DWORD						0x70

//Time unit ms
#define T_chpe							30*1000				//Chip erase time
#define T_wrsr							30					//Write status Register time

#define T_blk4e							300					//Block X Erase time
#define T_blk32e						1600
#define T_blk64e						2000

#define T_bp1							0.050
#define T_bp2							0.012
#define T_pp							3.4

/*****************************************************************************************************************/
/*													COMMANDS 										 			 */
/*****************************************************************************************************************/
#define OPC_STATUS_REG_1					0x05
#define OPC_STATUS_REG_2					0x35
#define OPC_STATUS_REG_3					0x15

#define CMD_WRITE_STATUS_REG1				0x01
#define CMD_WRITE_STATUS_REG2				0x31
#define CMD_WRITE_STATUS_REG3				0x11

#define CMD_WRITE_EN						0x06
#define CMD_WRITE_DIS						0x04

#define CMD_BLOCK_ERASE_4K					0x20
#define CMD_BLOCK_ERASE_32K					0x52
#define CMD_BLOCK_ERASE_64K					0xD8

#define CMD_CHIP_ERASE						0x60

#define CMD_NORMAL_READ						0x03		//@Ref[1] Standard read
#define CMD_FAST_READ						0x0B		//

#define CMD_DUAL_O_FAST_READ				0x3B		//@Ref[2] Dual fast read
#define CMD_DUAL_IO_FAST_READ				0xBB		//
#define CMD_QUAD_O_FAST_READ				0x6B
#define CMD_QUAD_IO_FAST_READ				0xEB

#define OP_WITH_CONTINUOUS					0x20		//@Ref[3] Option fast read
#define OP_NO_CONTINUOUS					0x10

#define CMD_WORD_FASTREAD					0xE7
#define CMD_SET_BRUST						0x77

#define CMD_BYTEPAGE_PROGRAM				0x02		//TODO

#define CMD_READ_MANUFACTURE_DEV_ID			0x9F
#define CMD_READ_UID						0x4B

#define CMD_DEEP_PWR_DOWN					0xB9
#define CMD_RELEASE_DEEP_PWR_DOWN			0xAB

/*****************************************************************************************************************/
/*													   DEVICE CONFIGURE										     */
/*****************************************************************************************************************/
#define MOD_STANDARD_READ					CMD_NORMAL_READ							//Choose between NORMAL READ And FAST READ @Ref[1]

#define MOD_BYTE_FAST_READ					CMD_DUAL_IO_FAST_READ					//Choose at @Ref[2]
#define OP_BYTE_FREAD_CONTINUOUS			OP_WITH_CONTINUOUS						//Choose at @Ref[3]

#define MOD_WORD_FAST_READ					CMD_WORD_FASTREAD
#define OP_WORD_FREAD_CONTINUOUS			OP_NO_CONTINUOUS						//Choose at @Ref[3]


/*****************************************************************************************************************/
/*													   DATA-STRUCTS												 */
/*****************************************************************************************************************/
typedef struct
{
	uint8_t ReadyFlag_B				:1;
	uint8_t WEL_B					:1;							// if true accept to program/ erase flash
	uint8_t Blk0Protection_B		:1;
	uint8_t Blk1Protection_B		:1;

	uint8_t Blk2Protection_B		:1;
	uint8_t Blk3Protection_B		:1;
	uint8_t Blk4Protection_B		:1;
	uint8_t SRP0Protection_B		:1;
} tStatusRegister1;

typedef struct
{
	uint8_t Res						:4;

	uint8_t DeviceStrength_U8		:2;
	uint8_t Res2					:2;
} tStatusRegister3;

typedef struct
{
	uint8_t SRP0Protection_B			:1;
	uint8_t QuadEnable_B				:1;
	uint8_t ProgSuspendStatus_B			:1;
	uint8_t LockSecurityReg1_B			:1;

	uint8_t LockSecurityReg2_B			:1;
	uint8_t LockSecurityReg3_B			:1;
	uint8_t CompleteBlkProtect_B		:1;
	uint8_t EraseSuspend_B				:1;
} tStatusRegister2;

typedef union {
	tStatusRegister1 Reg1_T;
	tStatusRegister2 Reg2_T;
	tStatusRegister3 Reg3_T;
	uint8_t Val_U8;
} tStatusRegister;

typedef struct {
	uint8_t OP	:2;
	uint8_t AD	:2;
	uint8_t DA	:2;
	uint8_t IDM	:2;
} tBusTxfer;

typedef struct {
	uint32_t Address_U32;
	uint8_t *Data_U8P;
	uint32_t Length_U32;
}tMemoryData;

int8_t AT25SF321X_ReadManufactureAndDeviceID(uint32_t* Value);
int8_t AT25SF321X_WriteEnable();
int8_t AT25SF321X_WriteDisable();
int8_t AT25SF321X_ReadUID(uint64_t* Value);
int8_t AT25SF321X_Flash_Read(tMemoryData MemoryData);
int8_t AT25SF321X_Flash_WriteSector(tMemoryData MemoryData);
int8_t AT25SF321X_Flash_Erase(uint32_t Address_U32, uint32_t Length_U32);
int8_t AT25SF321X_DeepPowerDown();
int8_t AT25SF321X_ResumeStandbyMode();
#endif /* IOTLIB_HAL_HARDWARES_AT25SF321X_H_ */

