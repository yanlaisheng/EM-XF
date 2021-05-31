/******************************************************************************
 *	File	:	main.c
 *	Version :	0.1
 *	Date	:	2020/04/24
 *	Author	:	XFORCE
 *
 *	Demonstrate how to use ECM-XF control drive type slave device
 *
 *	1.	Initialize the EtherCAT network and configure DC parameter
 *	2.	Setup driver by SDO communicate
 *	3.	Enable CiA402 state machine control
 *	4.	Control EtherCAT state and initialize position
 *	5.	Use FIFO to do Cyclic PDO data exchange
 *
 * @copyright (C) 2020 NEXTW TECHNOLOGY CO., LTD.. All rights reserved.
 *
 ******************************************************************************/

#include "EcmUsrDriver.h"
#include "EcmDriver.h"
#include "platform.h"
#include "main.h"

static int Vel[TEST_SERVO_CNT] = {0};

/*     V
 *     |
 *     |   _________
 *     |  /         \
 *     | /           \
 *___T_|/             \ _________               _________
 *     |                         \             /
 *     |                          \           /
 *     |                           \_________/
 *     |
*/
int CalVel(int nSlv, int nAcc, int nDec, int nTimeIdx)
{
	if (nTimeIdx < 50)
	{
		Vel[nSlv] += nAcc;
	}
	else if (nTimeIdx >= 39950 && nTimeIdx < 40000)
	{
		Vel[nSlv] -= nDec;
	}
	else if (nTimeIdx >= 80000 && nTimeIdx < 80050)
	{
		Vel[nSlv] -= nDec;
	}
	else if (nTimeIdx >= 119950 && nTimeIdx < 120000)
	{
		Vel[nSlv] += nAcc;
	}
	return Vel[nSlv];
}
int SetPdoConfTbl(PDO_CONFIG_HEAD *pConfig, uint8_t u8PdoIdx, uint8_t u8TblIdx, uint16_t u16Idx, uint8_t u8SubIdx, uint8_t u8BitSize)
{
	pConfig->Table[u8PdoIdx][u8TblIdx].u16Idx = u16Idx;
	pConfig->Table[u8PdoIdx][u8TblIdx].u8SubIdx = u8SubIdx;
	pConfig->Table[u8PdoIdx][u8TblIdx].u8BitSize = u8BitSize;
	return 1;
	/*
	 * Use API 'ECM_EcatPdoConfigSet' to set PDOs.
	 * Structure of type PDO_CONFIG_HEAD:
	 *      Slave: slave address
	 *      SmaIdx: Sync Manager index. It should be RxPDO Assign(0x1c12) or TxPDO Assign(0x1c13).
	 *      PDOCnt: Number of PDO mappings. No more than 3 for this API.
	 *      MapIdx[n]: The n-th PDO mapping index.
	 *      ObjsCnt[n]: Number of PDO entries for n-th PDO mapping. No more than 8 for this API.
	 *      Table[m][n]: The n-th PDO entrie of m-th PDO mapping.
	 * This API does not work if the number of PDO mappings is more the 3,
	 * or the number of PDO entries of any PDO mapping is more than 8.
	 * For those cases, use SDO request to set PDO configures.
	 */
}
int SdoGetTarPos()
{
	int nret = 0;
	int32_t n32Pos = 0;
	// Send read request
	// Target position command: 0x607A
	// #define ECM_SDO_OP_WR 0
	// #define ECM_SDO_OP_RD 1
	nret = ECM_EcatSdoReq(ECM_SDO_OP_RD, 0, 0x607A, 0, sizeof(n32Pos), 7000000, (uint8_t *)&n32Pos);
	if (nret <= 0)
	{
		PRINTF("SdoGetTarPos Request error\r\n");
		return n32Pos;
	}
	// Get last request feedback
	nret = ECM_EcatSdoGet((uint8_t *)&n32Pos);
	if (nret <= 0)
	{
		PRINTF("SdoGetTarPos Get error\r\n");
		return n32Pos;
	}
	return n32Pos;
}
int main_ini(void)
{
	int i = 0, j = 0, loop = 0;
	int nret = 0;
	uint8_t Version;

	PDO_CONFIG_HEAD RxPDOConfig[6];
	PDO_CONFIG_HEAD TxPDOConfig[6];

	uint16_t u16PDOSize = 0;
	uint16_t u16PDOSizeRet = 0;
	uint8_t u8CmdMode = 0, u8State = 0, u8FifoCnt = 0, u8LastState = 0;

	uint8_t u8WkcErrCnt = 0, u8CrcErrCnt = 0;
	uint8_t u8WkcErrCntLast = 0, u8CrcErrCntLast = 0;
	int8_t SlaveCnt = 0;
	int nTimeIdx[TEST_SERVO_CNT] = {0};
	uint8_t RxData[TEST_SPI_DATA_SIZE] = {0};
	uint8_t TxData[TEST_SPI_DATA_SIZE] = {0};
	uint16_t u16LogStatus[TEST_SERVO_CNT] = {0};
	int32_t u32LogPos[TEST_SERVO_CNT] = {0};

	RXPDO_ST_DEF_T *pRxPDOData = (RXPDO_ST_DEF_T *)RxData;
	TXPDO_ST_DEF_T *pTxPDOData = (TXPDO_ST_DEF_T *)TxData;
	extern uint32_t g_u32CrcType;

	////////////////////////////////////////////////////////////////////////////////////////////
	//The first section of the example is system initialize
	//and set all the data to 0 to prevent error
	SYS_Init();
	memset(RxData, 0, sizeof(RxData));	   // Set RxData value into 0
	memset(TxData, 0, sizeof(TxData));	   // Set TxDate value into 0
	memset(Vel, 0, sizeof(Vel));		   // Set Velocity Value into 0
	memset(nTimeIdx, 0, sizeof(nTimeIdx)); // Set Time index into 0

	//Set CRC Type
	ECM_SetCRCTypet(g_u32CrcType);

	//get firmware version
	nret = ECM_GetFirmwareVersion(&Version);
	if (nret <= 0)
		return -1;
	PRINTF("ECM-XF Firmware version : 0x%X\r\n", Version);
	////////////////////////////////////////////////////////////////////////////////////////////

	//	ECAT initial example
	//TEST_CYCTIME=250000£¬¼´250us
	nret = ECM_EcatInit(0x0030, TEST_CYCTIME, TEST_CYCTIME / 2); //Ó¦¸ÃÊÇ0x0003°É£¿
	// 0x03 is a activation of Sync0
	//BECKHOFF Register Description 3.49.4 SYNC Out unit (0x0980 : 0x0981)
	// Bit  Description
	//  0   Sync out unit activation (0:Deactivated 1:Activated)
	//  1   Sync0 generation (0:Deactivated 1:Activated)
	//  2   Sync1 generation (0:Deactivated 1:Activated)
	// The DC sync mode usually use as following:
	// Command  Description
	//  0x00    Deactivate the sync (Free Run)
	//  0x03    Activate Sync0 (DC Sync0 mode)
	//  0x07    Activate both Sync0 and Sync1
	if (nret <= 0)
	{
		PRINTF("ECM_EcatInit fail. No EtherCAT Slave found.\r\n");
		return -1;
	}
	PRINTF("ECM_EcatInit done\r\n");
	SlaveCnt = ECM_EcatSlvCntGet();
	if (SlaveCnt < TEST_SERVO_CNT)
	{
		PRINTF("ERROR : Slave count %d < %d\r\n", SlaveCnt, TEST_SERVO_CNT);
		return -1;
	}
	PRINTF("Found %d slaves\r\n", SlaveCnt);
	uint16_t u16VenderID;
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		u16VenderID = ECM_GetSlaveVenderID(i);
		PRINTF("[%d]Vender ID : %d\r\n", i, u16VenderID);
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	//Set the state to Pre-OP and prepare to configure PDO
	nret = ECM_StateCheck(0xFF, EC_STATE_PRE_OP, 1000); // Set mode must be at PRE-OP state
	// Select all slaves command: 0xFF, there's only 128 slaves totally.
	// In here, the function ECM_StateCheck(); includes state setting and check
	if (nret == 0)
	{
		return -1;
	}

	PRINTF("All Slaves are in PRE-OP state\r\n");

	////////////////////////////////////////////////////////////////////////////////////////////
	//In the following section shows how to configure RxPDO and TxPDO
	PRINTF("Assign and configure PDO\r\n");
	i = 0;
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		RxPDOConfig[i].SmaIdx = RxPDO_ASSIGN_IDX;
		RxPDOConfig[i].PDOCnt = 1;
		RxPDOConfig[i].MapIdx[0] = RxPDO_MAP_IDX;
		RxPDOConfig[i].ObjsCnt[0] = 2;
		SetPdoConfTbl(&RxPDOConfig[i], 0, 0, 0x6040, 0, 16); //0x6040:0 - control word  // 16 bits = 2 bytes for TEST_RXPDO_SIZE
		// the 1st parameter is PDO_CONFIG_HEAD
		// the 2nd parameter is 0 for the first RxPDOConfig.PDOCnt = 1
		// the 3rd parameter is 0 for the first object as RxPDOConfig.ObjsCnt[0] = 2
		// the 4th parameter is a object index (0x6040: control word)
		// the 5th parameter is a object sub-index
		// the 6th parameter is the bit size for 4th parameter
		SetPdoConfTbl(&RxPDOConfig[i], 0, 1, 0x607A, 0, 32); //0x607A:0 - target position // 32 bits = 4 bytes for TEST_RXPDO_SIZE

		TxPDOConfig[i].SmaIdx = TxPDO_ASSIGN_IDX;
		TxPDOConfig[i].PDOCnt = 1;
		TxPDOConfig[i].MapIdx[0] = TxPDO_MAP_IDX;
		TxPDOConfig[i].ObjsCnt[0] = 2;
		SetPdoConfTbl(&TxPDOConfig[i], 0, 0, 0x6041, 0, 16); //0x6041:0 - status word // 16 bits = 2 bytes for TEST_TXPDO_SIZE
		SetPdoConfTbl(&TxPDOConfig[i], 0, 1, 0x6064, 0, 32); //0x6064:0 - actual position // 32 bits = 4 bytes for TEST_TXPDO_SIZE
	}
	// Register PDO configure table
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		//for RxPDO
		nret = ECM_EcatPdoConfigSet(i, &RxPDOConfig[i]);
		if (nret <= 0)
		{
			PRINTF("Set RX PDO error\r\n");
			return -1;
		}
		//for TxPDO
		nret = ECM_EcatPdoConfigSet(i, &TxPDOConfig[i]);
		if (nret <= 0)
		{
			PRINTF("Set TX PDO error\r\n");
			return -1;
		}
	}
	// Call ECM_EcatReconfig() after setting PDO configures
	nret = ECM_EcatReconfig();
	if (nret < 0)
	{
		PRINTF("ECM_ECAT_CONFIG error : %d\r\n", nret);
		return -1;
	}
	PRINTF("\nList RxPDO :\r\n");
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		ECM_ShowPDOConfig(i, RxPDO_ASSIGN_IDX);
	}
	PRINTF("\nList TxPDO :\r\n");
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		ECM_ShowPDOConfig(i, TxPDO_ASSIGN_IDX);
	}
	//Get the FIFO RxPDO size
	u16PDOSize = ECM_FifoRxPdoSizeGet(); //TEST_RXPDO_SIZE = 16 bits + 32 bits = 6 bytes
	if (u16PDOSize != TEST_RXPDO_BUF_SIZE)
	{ //TEST_RXPDO_BUF_SIZE = TEST_RXPDO_SIZE*TEST_SERVO_CNT=6*4
		PRINTF("ERROR RxPdoSize(%d)\r\n", u16PDOSize);
		return -1;
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	// SDO is for slave parameter setting or return value from slave
	// In this section shows how to use SDO write and read for slave parameters
	// SDO example : use SDO write to set driver mode
	u8CmdMode = 8; // set driver in CSP mode 8: CSP, 9:CSV, 10:CST
	//	SDO write
	PRINTF("Set drive mode : CSP mode\r\n");
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		// Do write.
		nret = ECM_EcatSdoReq(ECM_SDO_OP_WR, i, 0x6060, 0, 1, 7000000, &u8CmdMode); //0x6060 : Mode of operation
		if (nret <= 0)
		{
			PRINTF("ECM_EcatSdoReq WRITE error\r\n");
			return -1;
		}
	}
	//	SDO read
	PRINTF("Check driver mode\r\n");
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		u8CmdMode = 0;
		// Send read request
		nret = ECM_EcatSdoReq(ECM_SDO_OP_RD, i, 0x6061, 0, 1, 7000000, &u8CmdMode); //0x6061 : Modes of operation display
		if (nret <= 0)
		{
			PRINTF("ECM_EcatSdoReq READ error\r\n");
		}
		// Get last request feedback
		nret = ECM_EcatSdoGet(&u8CmdMode);
		if (nret <= 0)
		{
			PRINTF("ECM_EcatSdoGet error\r\n");
		}
		PRINTF("Driver %d Mode : %d\r\n", i, u8CmdMode);
	}
	////////////////////////////////////////////////////////////////////////////////////////////
	//	Enable ECM-XF inside 402 state machine control
	//	NOTICE :Set the offset value and the expected state
	//	if the default value is not fit you need
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		nret = ECM_Drv402SM_Enable(i);
		if (nret == 0)
			return -1;
	}
	//Here only set the diver to servo on
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		ECM_Drv402SM_StateSet(i, CIA402_SW_OPERATIONENABLED);
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	// Change to SAFE-OP state
	nret = ECM_StateCheck(0xFF, EC_STATE_SAFE_OP, 1000);
	if (nret == 0)
	{
		return -1;
	}
	////////////////////////////////////////////////////////////////////////////////////////////
	//Before continuing to next section, the upper part configures PDO and set SDO parameters only
	//In this section shows the data exchange between XF and EtherCAT slave
	u16PDOSize = ECM_FifoTxPdoSizeGet(); //TEST_TXPDO_SIZE = 16 bits + 32 bits = 6 bytes //6 bytes * 4 slaves = 24 bytes
	PRINTF("TxPDOSize %d\r\n", u16PDOSize);
	u16PDOSize = ECM_FifoRxPdoSizeGet(); //TEST_RXPDO_SIZE = 16 bits + 32 bits = 6 bytes //6 bytes * 4 slaves = 24 bytes
	PRINTF("RxPDOSize %d\r\n", u16PDOSize);
	for (i = 0; i < 5; i++)
	{
		u16PDOSizeRet = u16PDOSize;
		nret = ECM_EcatPdoDataExchange(ECM_PDO_RD_OP, RxData, TxData, &u16PDOSizeRet); //Data exchange with slave directly
		if (nret == 0)
		{
			continue;
		}
		for (j = 0; j < TEST_SERVO_CNT; j++)
		{
			pRxPDOData[j].n32TarPos = pTxPDOData[j].n32AcuPos;
			if ((nret & ECM_PDO_RD_OP) == ECM_PDO_RD_OP)
			{
				PRINTF("[%d]PDOSize=%d, AcuPos=%ld, nret=%d\r\n", j, u16PDOSizeRet, pTxPDOData[j].n32AcuPos, nret);
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////
	// Change to OP state
	nret = ECM_StateCheck(0xFF, EC_STATE_OPERATIONAL, 1000);
	if (nret == 0)
	{
		return -1;
	}

	PRINTF("All Slaves are in OP state\r\n");
	PRINTF("Wait for initial position complete\r\n");

	////////////////////////////////////////////////////////////////////////////////////////////
	//	PDO example : use FIFO do cyclic data exchange
	//In this part shows using data exchange between XF and FIFO
	u8FifoCnt = 0;
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		pRxPDOData[i].n32TarPos = pTxPDOData[i].n32AcuPos;
		PRINTF("AcuPos %d = %ld\r\n", i, pTxPDOData[i].n32AcuPos);
	}
	while (1)
	{
		//When the FIFO is empty, continuing put data into FIFO
		if (u8FifoCnt < (PDO_FIFO_DEFAULT_CNT - 2))
		{
			for (i = 0; i < TEST_SERVO_CNT; i++)
			{
				pRxPDOData[i].n32TarPos += CalVel(i, 1, 1, nTimeIdx[i]++);

				if (nTimeIdx[i] >= 160000)
				{
					loop++;
					nTimeIdx[i] = 0;
				}
			}
			//Exchange data and also check FIFO count, Working counter Error and CRC error
			//Checking FIFO count to prevent PDO number is more than FIFO volume
			//Working counter error count is the error count between XF(Master) and EtherCAT slave(slave)
			//CRC Error count is the error count between SPI(Master) and XF(slave)
			nret = ECM_EcatPdoFifoDataExchange(PDO_FIFO_DEFAULT_CNT, RxData, TxData, u16PDOSize, &u8FifoCnt, &u8CrcErrCnt, &u8WkcErrCnt);
			if (nret > 0)
			{ // Get valid data
				for (i = 0; i < TEST_SERVO_CNT; i++)
				{
					u16LogStatus[i] = pTxPDOData[i].u16StaWord;
					u32LogPos[i] = pTxPDOData[i].n32AcuPos;
				}
			}
		}
		//If the FIFO is full, turn to the following process to prevent FIFO reject data and continuing checking FIFO count
		else
		{
			nret = ECM_InfoUpdate(&u8State, &u8FifoCnt, &u8CrcErrCnt, &u8WkcErrCnt);
			if (nret)
			{
				ECM_GetRetErrStatus(&u8State);
				if (u8LastState != u8State)
				{
					PRINTF("Err Status(0x%X)\r\n", u8State);
				}
				u8LastState = u8State;
			}
			if (j++ > 200)
			{
				j = 0;
				PRINTF("FIFO count:%d, ", u8FifoCnt);
				for (i = 0; i < TEST_SERVO_CNT; i++)
				{
					PRINTF("[%d] %ld,  ", i, u32LogPos[i]);
				}
				PRINTF("\r\n");
			}
		}
		//When Error from Working Counter or CRC checking occurs, show the error value
		if (u8WkcErrCntLast != u8WkcErrCnt)
		{
			PRINTF("\nu8WkcErrCnt %d\r\n", u8WkcErrCnt);
		}
		if (u8CrcErrCntLast != u8CrcErrCnt)
		{
			PRINTF("\nu8CrcErrCnt %d\r\n", u8CrcErrCnt);
		}
		u8WkcErrCntLast = u8WkcErrCnt;
		u8CrcErrCntLast = u8CrcErrCnt;
		if (loop > 1)
			break;
	}

	// wait for FIFO empty
	for (; u8FifoCnt != 0;)
	{
		nret = ECM_InfoUpdate(&u8State, &u8FifoCnt, &u8CrcErrCnt, &u8WkcErrCnt);
		PRINTF("FIFO Count %d\r\n", u8FifoCnt);
		UserDelay(200); //200usec
	}

	//set servo off
	for (i = 0; i < TEST_SERVO_CNT; i++)
	{
		ECM_Drv402SM_StateSet(i, CIA402_SW_READYTOSWITCHON);
		PRINTF("Slave %d: SERVO OFF\r\n", i);
	}
	UserDelay(1000000); //1s

	nret = ECM_StateCheck(0xFF, EC_STATE_INIT, 1000);
	if (nret == 0)
	{
		return -1;
	}

	PRINTF("All Slaves are in INIT state\r\n");
	PRINTF("Demo complete\r\n");
	return 0;
}
//	(C) COPYRIGHT 2020 NEXTW Technology Corp.
