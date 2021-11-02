/*
 * AT25SF321x.c
 *
 *  Created on: Aug 11, 2021
 *      Author: DFMPC-
 */

#include "AT25SF321x.h"

/*****************************************************************************************************************/
/*														  MACRO													 */
/*****************************************************************************************************************/
#define mFlashEnable()							Flash_HAL_TP->GPIO_HAL_TP->Write(&GPIO_PWR, SET)
#define mFlashDisable()							Flash_HAL_TP->GPIO_HAL_TP->Write(&GPIO_PWR, RESET)

#define SPI_CommandConfig(IntructionSize, AddressSize, DataSize, DummySize, BusTxfer)\
				UNUSED(IntructionSize); UNUSED(AddressSize); UNUSED(DataSize); UNUSED(DummySize); UNUSED(BusTxfer);

/*****************************************************************************************************************/
/*													GLOBAL VARIABLES											 */
/*****************************************************************************************************************/


/*****************************************************************************************************************/
/*													PRIVATE VARIABLES											 */
/*****************************************************************************************************************/

/*****************************************************************************************************************/
/*													PRIVATE FUNCTION											 */
/*****************************************************************************************************************/
static int8_t SPI_Transmit(void* TxData_P, uint32_t Length_U32);
static int8_t SPI_Receive(void* RxData_P, uint32_t Length_U32);
static int8_t SPI_TransmitReceive(void* TxData_P, void* RxData_P,uint32_t Length_U32);

static int8_t AT25SF3321x_EraseBlock(uint8_t Opcode, uint32_t DevAddress_U32, uint8_t NbBlock);

/*****************************************************************************************************************/
/*												HARDWARE ABSTRACT LAYER									 		 */
/*****************************************************************************************************************/
static void Read(tMemoryData);
static void Write(tMemoryData);
static bool Erase(tMemoryData);
static void ReadUID(void* Value);
static void Sleeping(void* Def);
static void Wakeup(void* Def);


/*****************************************************************************************************************/
/*												  FUNCTION IMPLEMENT									 	 	 */
/*****************************************************************************************************************/
void Flash_UnitTest()
{
	bool result = 0;
	uint8_t wdata[cBLOCK_4KB];
	uint8_t rdata[2*cBLOCK_4KB];
	uint8_t expectData[2*cBLOCK_4KB];

	tMemoryData TestRData_T = {
		.Address_U32 = 0,
		.Data_U8P = (void*)&rdata,
		.Length_U32 = sizeof(rdata),
	};

	tMemoryData TestWData_T = {
		.Address_U32 = 0,
		.Data_U8P = (void*)&wdata,
		.Length_U32 = sizeof(wdata),
	};

	TestWData_T.Address_U32 = 0;
	for (int i = 0; i < cBLOCK_4KB; i++)
	{
		wdata[i] = i % 256;
		expectData[i + TestWData_T.Address_U32] = wdata[i];
	}
	Write(TestWData_T);

	TestWData_T.Address_U32 = 4096;
	for (int i = 0; i < cBLOCK_4KB; i++)
	{
		wdata[i] = 255 - i % 256;
		expectData[i + TestWData_T.Address_U32] = wdata[i];
	}
	Write(TestWData_T);

	TestWData_T.Address_U32 = 2048;
	for (int i = 0; i < cBLOCK_4KB; i++)
	{
		wdata[i] = 1;
		expectData[i + TestWData_T.Address_U32] = wdata[i];
	}
	Write(TestWData_T);

	Read(TestRData_T);
	result = !memcmp(expectData, rdata, sizeof(rdata));

	TestWData_T.Address_U32 = 2048;
	TestWData_T.Length_U32  = 6144;
	Erase(TestWData_T);

	for (int i = 0; i < TestWData_T.Length_U32; i++)
		expectData[i + TestWData_T.Address_U32] = 0xFF;

	Read(TestRData_T);
	result = !memcmp(expectData, rdata, sizeof(rdata));

	while(1);
}

void Flash_Initialize()
{
	Flash_HAL_TP->GPIO_HAL_TP->Flash_Initialize(&GPIO_PWR, SET);

	uint32_t TempData_U32 = {0x01234567};

	tMemoryData TestData_T = {
		.Address_U32 = STORAGE_ADDR_MEM,
		.Data_U8P = (void*)&TempData_U32,
		.Length_U32 = sizeof(TempData_U32),
	};

	Write(TestData_T);
	TempData_U32 = 0x0;
	Read(TestData_T);

	Flash_HAL_TP->Enable_B = (0x01234567 == TempData_U32);

	uint32_t Value = 0;

	if (!AT25SF321X_ReadManufactureAndDeviceID(&Value))
		Flash_HAL_TP->Enable_B = (Value == MANUFACTURE_DEVICE_ID);
}

static void Read(tMemoryData MemoryData)
{
	AT25SF321X_Flash_Read(MemoryData);
}

static void Write(tMemoryData Input)
{
	int8_t Result_S8 = -1;
	uint16_t index = 0;
	uint16_t Length = 0;
//	uint8_t* MemAllocated = malloc(cBLOCK_4KB);
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

//	free(MemAllocated);
}

static bool Erase(tMemoryData Input)
{
	return (0 == AT25SF321X_Flash_Erase(Input.Address_U32, Input.Length_U32));
}

static void ReadUID(void* Value)
{
	if (0 != AT25SF321X_ReadUID(Value))
		memset(Value, 0 , sizeof(uint64_t));
}

static void Sleeping(void* Def)
{
//	AT25SF321X_DeepPowerDown();
	mFlashDisable();
}

static void Wakeup(void* Def)
{
	mFlashEnable();
//	AT25SF321X_ResumeStandbyMode();
}

/*****************************************************************************************************************/
/*													PRIVATE FUNCTION											 */
/*****************************************************************************************************************/
static int8_t SPI_TransmitReceive(void* TxData_P,void* RxData_P, uint32_t Length_U32)
{
	((tSPI*)Flash_HAL_TP->CORE_SERVICES_T.Driver_TP)->TransmitReceive(TxData_P, RxData_P, Length_U32);
	return ((tSPI*)Flash_HAL_TP->CORE_SERVICES_T.Driver_TP)->State_U8;
}

static int8_t SPI_Transmit(void* TxData_P, uint32_t Length_U32)
{
	((tSPI*)Flash_HAL_TP->CORE_SERVICES_T.Driver_TP)->TxTransmit(TxData_P, Length_U32);
	return ((tSPI*)Flash_HAL_TP->CORE_SERVICES_T.Driver_TP)->State_U8;
}

static int8_t SPI_Receive(void* RxData_P, uint32_t Length_U32)
{
	((tSPI*)Flash_HAL_TP->CORE_SERVICES_T.Driver_TP)->RxReceive( RxData_P, Length_U32);
	return ((tSPI*)Flash_HAL_TP->CORE_SERVICES_T.Driver_TP)->State_U8;
}

/**
 * @Description: Read Status Register to determine the device's read/busy status. The Register can be read at any time.
 * @Input:  RegStatus: pointer to memory/variable address contain value status register
 *			OpcodeReg: Instruction-dependent information, such as address
 *
 * @return  if executed successfully return 0 else return -1
 */

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
	uint64_t StartTime = GetCurrentTime_ms();
	tStatusRegister StatusRegister_T = {0};

	do
	{
		Result_S8 = AT25SF321X_ReadStatusReg(Opcode, &StatusRegister_T);
	}
	while (!Result_S8 && CheckProgress(&StatusRegister_T) && CheckTimeout(StartTime, Timeout));

	if (Register)
		*Register = StatusRegister_T;

	return Result_S8;
}

/**
 * @Description: To modifies the block protection, security register lock down, Quad enable and status
 * register protection.
 *
 * @Input:  RegStatus: pointer to memory/variable address contain value status register
 *			OpcodeReg: Instruction-dependent information, such as address
 *
 * @return  if executed successfully return 0 else return -1
 */

static int8_t AT25SF321X_WriteStatusReg(uint8_t OpcodeReg, tStatusRegister RegValue)
{
	int8_t Result_U8 = -1;
	uint8_t Command[] = {
			OpcodeReg,
			RegValue.Val_U8
	};
	Result_U8 = SPI_Transmit(&Command, sizeof(Command));

	int8_t CheckProgress(tStatusRegister* Status) { return Status->Reg1_T.WEL_B;}

	//Recommended: The status register be polled to determine if device has finished execution
	Result_U8 = PollingReadStatusRegister(OPC_STATUS_REG_1, NULL, &CheckProgress, T_wrsr);

	return Result_U8;
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

int8_t AT25SF321X_WriteDisable()
{
	int8_t Result_U8 = -1;
	uint8_t Command = CMD_WRITE_DIS;
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

int8_t AT25SF321X_ReadUID(uint64_t* Value)
{
	int8_t Result_U8 = -1;
	uint8_t Command[] = {
			CMD_READ_UID,
			0,1,2,3,								//Dummy
			8,8,8,8,8,8,8,8
	};

	Result_U8 = SPI_TransmitReceive(Command, Command, sizeof(Command));
	memcpy(Value, &Command[5], sizeof(uint64_t));

	return Result_U8;
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


#if QSPI
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,1,1,0);
	SPI_CommandConfig(1, 3, 1, 1, BusTxfer);
#endif

//	uint8_t* MemAllocated = malloc(sizeof(Instruction) + Length);
	uint8_t MemAllocated[sizeof(Instruction) + Length];
	memcpy(MemAllocated, Instruction, sizeof(Instruction));
	memcpy(MemAllocated + sizeof(Instruction), MemoryData.Data_U8P, Length);

	Result_S16 = SPI_Transmit(MemAllocated, sizeof(Instruction) + Length);
//	free(MemAllocated);

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

//	uint8_t* MemAllocated = malloc(sizeof(Instruction) + MemoryData.Length_U32);
	uint8_t MemAllocated[sizeof(Instruction) + MemoryData.Length_U32];
	memcpy(MemAllocated, Instruction, sizeof(Instruction));
	memset(MemAllocated + sizeof(Instruction), 0, MemoryData.Length_U32);

	Result_S8 = SPI_TransmitReceive(MemAllocated, MemAllocated, sizeof(Instruction) + MemoryData.Length_U32);
	memcpy(MemoryData.Data_U8P, MemAllocated + sizeof(Instruction), MemoryData.Length_U32);
//	free(MemAllocated);

	return Result_S8;
}


/**
 * @Description
 * This function read data form Flash with specific address and length.Function support four read mode can be using depend on configure
 * command MOD_BYTE_FAST_READ, it's:
 * 				+ CMD_DUAL_O_FAST_READ
 * 				+ CMD_DUAL_IO_FAST_READ
 * 				+ CMD_QUAD_O_FAST_READ
 * 				+ CMD_QUAD_IO_FAST_READ
 * with option use/not use continuous read.
 *				+OP_BYTE_FREAD_CONTINUOUS
 *	MOD_BYTE_FAST_READ &  OP_BYTE_FREAD_CONTINUOUS is configured ad header file
 * Ref Docs
 *
 * @param  Address in range 0 - 4MB
 * @param  Data: Pointer to memory contain data read
 * @param  Length: to read
 * @return execute success or not
 */


static int8_t AT25SF321X_ByteFastRead(uint32_t Address, uint8_t* Data, uint32_t Length)
{
	int8_t  Result_S8 = -1;
	uint16_t Dummy = 0;
	uint8_t InitCmd = OP_BYTE_FREAD_CONTINUOUS;

	uint8_t Instruction[] = { MOD_BYTE_FAST_READ,
							  Address >> 16,
							  Address >> 8,
							  Address >> 0,
#if (MOD_BYTE_FAST_READ == CMD_DUAL_O_FAST_READ)
			  || (MOD_BYTE_FAST_READ == CMD_QUAD_O_FAST_READ)
							  Dummy
#elif (MOD_BYTE_FAST_READ == CMD_DUAL_IO_FAST_READ)
							  InitCmd
#elif (MOD_BYTE_FAST_READ == CMD_QUAD_IO_FAST_READ)
			  	  	  	  	  InitCmd,
							  Dummy >> 8,
							  Dummy
#endif
							};

#if QSPI

#if MOD_BYTE_FAST_READ == CMD_DUAL_O_FAST_READ
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,1,2,1);
	SPI_CommandConfig(1, 3, 1, 1, BusTxfer);
#elif MOD_BYTE_FAST_READ == CMD_DUAL_IO_FAST_READ
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,2,2,2);
	SPI_CommandConfig(1, 3, 1, 1, BusTxfer);
#elif MOD_BYTE_FAST_READ == CMD_QUAD_O_FAST_READ
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,1,4,1);
	SPI_CommandConfig(1, 3, 1, 1, BusTxfer);			//1 init
#elif MOD_BYTE_FAST_READ == CMD_QUAD_IO_FAST_READ
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,4,4,4);
	SPI_CommandConfig(1, 3, 1, 3, BusTxfer);			//3: dummy+init
#else
#error
#endif

#endif

	Result_S8 = SPI_TransmitReceive(&Instruction, Data, Length + sizeof(Instruction));

	return Result_S8;
}


int8_t AT25SF321X_SetBrust(uint8_t WLength)
{
	int8_t  Result_S8 = -1;
	uint8_t Dummy[4] = {0,0,0, WLength};

	uint8_t Instruction[] = { CMD_SET_BRUST,
							  Dummy[0],
							  Dummy[1],
							  Dummy[2],
							  Dummy[3],
							};

#if QSPI
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,1,1,4);
	SPI_CommandConfig(1, 0, 0, 4, BusTxfer);			//4: dummy
#endif
	Result_S8 = SPI_Transmit(&Instruction, sizeof(Instruction));

	return Result_S8;
}


/**
 * @Description
 * This function read data form Flash with specific address and length.Function support four read mode can be using depend on configure
 * command MOD_WORD_FAST_READ, it's:
 * 				+ CMD_QUAD_FASTREAD
 * with option use/not use continuous read.
 *				+OP_WORD_FREAD_CONTINUOUS
 *	MOD_WORD_FAST_READ &  OP_WORD_FREAD_CONTINUOUS is configured ad header file
 * Ref Docs
 *
 * @param  Address in range 0 - 4MB
 * @param  Data: Pointer to memory contain data read
 * @param  Length: to read
 * @return execute success or not
 */
static int8_t AT25SF321X_WordFastRead(uint32_t Address,uint8_t WLength, void* Data, uint32_t Length)
{
	int8_t  Result_S8 = -1;
	uint16_t Dummy = 0;
	uint8_t InitCmd = OP_WORD_FREAD_CONTINUOUS;

	if ((Address & 0x01) || AT25SF321X_SetBrust(WRAP_HWORD))
		return -1;

	tStatusRegister Reg2;
	AT25SF321X_WriteStatusReg(CMD_WRITE_STATUS_REG2, Reg2);

	uint8_t Instruction[] = { MOD_WORD_FAST_READ,
							  Address >> 16,
							  Address >> 8,
							  Address >> 0,
							  InitCmd,
							  Dummy
							};
#if QSPI
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,4,4,4);
	SPI_CommandConfig(1, 3, 2, 2, BusTxfer);			//3: dummy+init	,, check data size
#endif

	Result_S8 = SPI_TransmitReceive(&Instruction, Data, Length + sizeof(Instruction));

	return Result_S8;
}

static int8_t AT25SF3321X_EraseChip()
{
	uint8_t Result = -1;

	if (1 == AT25SF321X_WriteEnable())
	{
		uint8_t frame[] = {CMD_CHIP_ERASE};

		Result = SPI_Transmit(frame, sizeof(frame));

		int8_t CheckProgress(tStatusRegister* Status) { return Status->Reg1_T.ReadyFlag_B;};		//0 Device readly, stop polling

		//Recommended: The status register be polled to determine if device has finished execution
		//if not use polling check can be use delay with specific time, ref datasheet, page 49
		Result = PollingReadStatusRegister(OPC_STATUS_REG_1, NULL, &CheckProgress, T_chpe);
	}

	return Result;
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

/**
 * 	When put device into deep power mode down, all commands are ignored, but excepted resume for deep power down (0xAB)
 *
 */

int8_t AT25SF321X_DeepPowerDown()
{
	uint8_t Result_S8 = -1;
	uint8_t Instruction = CMD_DEEP_PWR_DOWN;

#if QSPI
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,1,1,1);
	SPI_CommandConfig(1, 0, 0, 0, BusTxfer);
#endif

	Result_S8 = SPI_Transmit(&Instruction, sizeof(Instruction));

	return Result_S8;
}

int8_t AT25SF321X_ResumeStandbyMode()
{
	uint8_t Result_S8 = -1;
	uint8_t Instruction = CMD_RELEASE_DEEP_PWR_DOWN;

#if QSPI
	tBusTxfer BusTxfer = INIT_BUSTXFER(1,1,1,1);
	SPI_CommandConfig(1, 0, 0, 0, BusTxfer);
#endif

	Result_S8 = SPI_Transmit(&Instruction, sizeof(Instruction));

	return Result_S8;
}


/*****************************************************************************************************************/
/*												  API_AT25SF321X											 	 */
/*****************************************************************************************************************/
/*
 * Warning: The bit WEL on RgS1 must be set before call this function
 */
int8_t AT25SF321X_Flash_Read(tMemoryData MemoryData)
{
	int16_t RResutl_S8 = -1;

	if (!MemoryData.Data_U8P || !MemoryData.Length_U32)
		return RResutl_S8;

	RResutl_S8 = AT25SF321X_ByteStandradRead(MemoryData);

	return RResutl_S8;
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

/**
 * Erase data on flash
 * @ Warning: The bit WEL on RgS1 must be set before call this function
 * @param Address: of flash
 * 		  Length to earase
 * @return  return 0 if executed successfully, else return error
 */
int8_t AT25SF321X_Flash_Erase(uint32_t Address_U32, uint32_t Length_U32)
{
	int8_t Result_S8 = -1;

	if (!Length_U32)
	{
		return Result_S8;
	}
	else if ((Address_U32 == AT25SF321X_START_MEM)	&& (Length_U32 >= (AT25SF321X_END_MEM + 1)))
	{
		return AT25SF3321X_EraseChip();
	}

	uint16_t index = 0;
	uint16_t LengthTemp = 0;
//	uint8_t* MemAllocated = malloc(cBLOCK_4KB);
	uint8_t MemAllocated[cBLOCK_4KB];
	tMemoryData RWData = {
							.Address_U32 = Address_U32 & MASK_BLK_4K,
							.Data_U8P	 = MemAllocated,
							.Length_U32	 = cBLOCK_4KB
						 };

	do
	{
		Result_S8 = -1;

		index = Address_U32 & (cBLOCK_4KB - 1);

		if ((index + Length_U32) > cBLOCK_4KB)
			LengthTemp = cBLOCK_4KB - index;
		else
			LengthTemp = Length_U32;

		if ((0 == index) && (cBLOCK_4KB == LengthTemp))
		{
			Result_S8 = AT25SF3321x_EraseBlock(CMD_BLOCK_ERASE_4K, RWData.Address_U32, 1);
		}
		else
		{
			Result_S8 = AT25SF321X_Flash_Read(RWData);

			for (uint16_t i = 0; i < LengthTemp; i++)
				RWData.Data_U8P[index + i] = 0xFF;

			Result_S8 = AT25SF3321x_EraseBlock(CMD_BLOCK_ERASE_4K, RWData.Address_U32, 1);
			Result_S8 = AT25SF321X_Flash_WriteSector(RWData);
		}

		if (Result_S8 == 0)
		{
			Address_U32 += LengthTemp;
			Length_U32  -= LengthTemp;

			RWData.Address_U32 += cBLOCK_4KB;
		}

	} while ((Result_S8 == 0) && Length_U32);

//	free(MemAllocated);

	return Result_S8;


/*	uint8_t Opcode = 0;
	uint32_t StartAddress_U32 = Address_U32;
	uint32_t EndAddress_U32 = Address_U32 + Length_U32 - 1;

	uint32_t MaskAddressBlk_TA[] = {MASK_BLK_64K, MASK_BLK_32K, MASK_BLK_4K};
	uint8_t BlockSizeIdx = ((Length_U32 < cBLOCK_64KB) ? ((Length_U32 < cBLOCK_32KB) ? 2 : 1) : 0);

	uint32_t MaskAddressBlk = MaskAddressBlk_TA[BlockSizeIdx++];
	uint32_t BeginPageAddr_U32 = StartAddress_U32 & MaskAddressBlk;
	uint32_t EndPageAddr_U32 = EndAddress_U32 & MaskAddressBlk;

	uint8_t  NbBlock_U8 = 0;

	void* DataRemained_P = 0;

	Opcode = (MaskAddressBlk == MASK_BLK_4K) ? CMD_BLOCK_ERASE_4K :
					((MaskAddressBlk == MASK_BLK_32K) ? CMD_BLOCK_ERASE_32K : CMD_BLOCK_ERASE_64K);


	NbBlock_U8 = (EndPageAddr_U32 - BeginPageAddr_U32) / ((~MaskAddressBlk) + 1);

	if (StartAddress_U32 == BeginPageAddr_U32)
	{
		Result_S8 = AT25SF3321x_EraseBlock(Opcode, BeginPageAddr_U32, NbBlock_U8);

		if(0 != Result_S8)
			return Result_S8;

	}
	else
	{
		Result_S8 = AT25SF3321x_EraseBlock(Opcode, BeginPageAddr_U32 + ((~MaskAddressBlk) + 1), NbBlock_U8 - 1);
		if(0 != Result_S8)
			return Result_S8;
	}

	if (MaskAddressBlk != cBLOCK_4KB)
	{
		Result_S8 = AT25SF321X_Flash_Erase(StartAddress_U32, StartAddress_U32 - BeginPageAddr_U32);
		if(0 != Result_S8)
			return Result_S8;

		Result_S8 = AT25SF321X_Flash_Erase(EndPageAddr_U32, EndAddress_U32 - EndPageAddr_U32);
		if(0 != Result_S8)
			return Result_S8;
	}
	else //Minimum block size
	{
		uint32_t RemainAddrBegin = 0;
		uint32_t RemainAddrEnd = 0;
		tMemoryData MemoryData;

		Remain data from  begin page to start erase
		RemainAddrBegin = BeginPageAddr_U32;
		RemainAddrEnd = StartAddress_U32;

		MemoryData.Address_U32 = RemainAddrBegin;
		MemoryData.Data_U8P	   = DataRemained_P;
		MemoryData.Length_U32  = RemainAddrEnd - RemainAddrBegin;
		Result_S8 = AT25SF321X_Flash_Read(MemoryData);
		if (0 != Result_S8)
			return Result_S8;

		Result_S8 = AT25SF3321x_EraseBlock(Opcode, BeginPageAddr_U32, 1);
		if (0 != Result_S8)
			return Result_S8;

		MemoryData.Address_U32 = RemainAddrBegin;
		MemoryData.Data_U8P	   = DataRemained_P;
		MemoryData.Length_U32  = RemainAddrEnd - RemainAddrBegin;
		Result_S8 = AT25SF321X_Flash_WriteSector(MemoryData);

		if (0 != Result_S8)
			return Result_S8;

		Remain data from stop erase to page next to end page
		RemainAddrBegin = EndAddress_U32;
		RemainAddrEnd = EndPageAddr_U32 + ((~MaskAddressBlk) + 1);

		MemoryData.Address_U32 = RemainAddrBegin;
		MemoryData.Data_U8P	   = DataRemained_P;
		MemoryData.Length_U32  = RemainAddrEnd - RemainAddrBegin;
		Result_S8 = AT25SF321X_Flash_Read(MemoryData);
		if (0 != Result_S8)
			return Result_S8;

		MemoryData.Address_U32 = RemainAddrBegin;
		MemoryData.Data_U8P	   = DataRemained_P;
		MemoryData.Length_U32  = RemainAddrEnd - RemainAddrBegin;
		Result_S8 = AT25SF3321x_EraseBlock(Opcode, EndPageAddr_U32, 1);
		if (0 != Result_S8)
			return Result_S8;

		MemoryData.Address_U32 = RemainAddrBegin;
		MemoryData.Data_U8P	   = DataRemained_P;
		MemoryData.Length_U32  = RemainAddrEnd - RemainAddrBegin;
		Result_S8 = AT25SF321X_Flash_WriteSector(MemoryData);
		if (0 != Result_S8)
			return Result_S8;
	}

	return Result_S8;
*/
}


