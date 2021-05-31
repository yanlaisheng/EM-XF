/******************************************************************************
 *	File	:	EcmUsrDriver.c
 *	Version :	0.1
 *	Date	:	2020/04/24
 *	Author	:	XFORCE
 *
 *	ECM-XF basic driver example - Source file
 *
 *	Demonstrate how to implement API type user driver
 *
 * @copyright (C) 2020 NEXTW TECHNOLOGY CO., LTD.. All rights reserved.
 *
 ******************************************************************************/

#include "EcmUsrDriver.h"
#include "main.h"
#include "platform.h"

uint8_t u8TxBuf[PKG_MAX_SIZE] = {0}; //发送缓冲区1448
uint8_t u8RxBuf[PKG_MAX_SIZE] = {0}; //接收缓冲区1448

SPI_CMD_PACKAGE_T *pCmd = (SPI_CMD_PACKAGE_T *)u8TxBuf; //全局指针，发送命令指针，指向发送缓冲区
SPI_RET_PACKAGE_T *pRet = (SPI_RET_PACKAGE_T *)u8RxBuf; //全局指针，接收命令指针，指向接收缓冲区

uint8_t u8CmdIdx = 0; //命令索引号

/************************
 * 函数：SPI数据收发
 * 功能：SPI数据发送和接收
 * 调用UserSpiDataExchange函数，发送缓冲区u8TxBuf，接收缓冲区u8RxBuf，长度：默认144个字节
 * 输入参数：返回值索引指针、返回值命令指针
 * 输出：对返回数值进行CRC校验如果CRC校验正确，则返回1；
 * 如果CRC校验不正确，则检查是否有数据偏移，有偏移，则再次发送，并返回0
 */
int SpiDataExchange(uint8_t *RetIdx, uint8_t *RetCmd)
{
	int i;
	pCmd->Head.u32StartWord = ECM_START_WORD; //0xA1A2A3A4
	if (pCmd->Head.u8Cmd == ECM_CMD_CRC_TYPE_SET)
		pCmd->Crc = ECM_CRC_MAGIC_NUM; //0x12345678
	else
		pCmd->Crc = UserCalCrc((uint8_t *)pCmd, sizeof(pCmd->Head) + pCmd->Head.u16Size);

	pCmd->StopWord = ECM_STOP_WORD; //0x56575859
	UserSpiDataExchange(u8TxBuf, u8RxBuf, sizeof(SPI_CMD_PACKAGE_T));
	UserDelay(200); //200usec
	if ((pCmd->Head.u8Cmd == ECM_CMD_CRC_TYPE_SET && pRet->Crc == ECM_CRC_MAGIC_NUM) ||
		pRet->Crc == UserCalCrc((uint8_t *)pRet, sizeof(pRet->Head) + pRet->Head.u16Size))
	{
		//如果CRC校验正确
		if (RetIdx)
			*RetIdx = pRet->Head.u8Idx; //回应索引号
		if (RetCmd)
			*RetCmd = pRet->Head.u8Cmd; //回应命令码
		return 1;						//则返回1
	}
	// CRC Check Fail.
	for (i = 0; i < sizeof(SPI_CMD_PACKAGE_T) - 3; i++)
	{
		//在接收缓冲区中查找是否存在连续的0x59/0x58/0x57/0x56这4个数值
		if (u8RxBuf[i] == 0x59 && u8RxBuf[i + 1] == 0x58 && u8RxBuf[i + 2] == 0x57 && u8RxBuf[i + 3] == 0x56)
			break;
	}
	if (i >= sizeof(SPI_CMD_PACKAGE_T) - 4) //如果没找到，则报错
		PRINTF("CRC Error(%d)\r\n", pCmd->Head.u8Idx);
	else // correct data shift
	{
		//如果找到了，则说明有偏移，再进行收发
		PRINTF("shift (%d)\r\n", sizeof(SPI_CMD_PACKAGE_T) - i - 4);
		UserSpiDataExchange(u8TxBuf, u8RxBuf, i + 4);
		UserDelay(2000); //2ms
	}
	return 0;
}

/************************
 * 函数：获得硬件版本
 * 命令：ECM_CMD_FW_VERSION_GET
 * 输入参数：硬件版本地址指针
 * 输出：如果返回值的索引号等于发送命令的索引号，则得到正确的版本号，并返回1；
 * 否则，查询100次后，仍然得不到正确的版本号，则返回0
 */
int ECM_GetFirmwareVersion(uint8_t *pVersion)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_FW_VERSION_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++; //索引号，主要用途為識別用
	pCmd->Head.u8Ctrl = 0xF8;	   // clear all error
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				*pVersion = pRet->Head.u8Return;
				return 1;
			}
		}
	}
	return 0;
}

/************************
 * 函数：获得厂商码
 * 命令：ECM_CMD_ECAT_SLV_INFO_GET
 * 输入参数：从站站号
 * 输出：如果返回值的索引号等于发送命令的索引号，则得到正确的版本号，并返回1；
 * 否则，查询10次后，仍然得不到正确的版本号，则返回0
 */
uint16_t ECM_GetSlaveVenderID(uint8_t u8Slave)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SLV_INFO_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Param = u8Slave;

	// 命令参数[0]=從站資訊碼
	// 0 : 廠商碼
	// 1 : 廠商產品碼
	// 2 : 產品版本號
	// 3 : 產品名稱
	// 4 : 配置位置
	// 5 : 位置別名
	// 6 : 狀態
	// 7 : AL狀態
	// 8 : 輸出資料長度
	// 9 : 輸入資料長度
	pCmd->Head.u8Data[0] = 0; // Get Vender ID
	for (i = 0; i < 10; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return *((uint16_t *)pRet->Data);
			}
		}
	}
	return 0;
}

/************************
 * 函数：设置CRC校验类型
 * 命令：ECM_CMD_CRC_TYPE_SET
 */
int ECM_SetCRCTypet(uint8_t u8CRCType)
{
	pCmd->Head.u8Cmd = ECM_CMD_CRC_TYPE_SET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Param = u8CRCType;
	return SpiDataExchange(0, 0); //0,0表示只发送命令，不返回索引号和命令号
}

/************************
 * 函数：更新表头资料，即获取最新的ECM状态
 * 命令：ECM_CMD_INFO_UPDATE_OP
 * 输出：如果更新成功，则返回ECM状态、RxPDO待发送数、CRC错误数、WKC错误数，并返回1
 * 如果更新不成功，则返回0
 */
int ECM_InfoUpdate(uint8_t *pEcmStatus, uint8_t *pRxPDOFifoCnt, uint8_t *CrcErrCnt, uint8_t *WkcErrCnt)
{
	pCmd->Head.u8Cmd = ECM_CMD_INFO_UPDATE_OP;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Ctrl = 0;
	if (SpiDataExchange(0, 0))
	{
		if (pEcmStatus)
			*pEcmStatus = pRet->Head.u8Status;
		if (pRxPDOFifoCnt)
			*pRxPDOFifoCnt = pRet->Head.u8RxFifoCnt;
		if (CrcErrCnt)
			*CrcErrCnt = pRet->Head.u8CrcErrCnt;
		if (WkcErrCnt)
			*WkcErrCnt = pRet->Head.u8WkcErrCnt;
		return 1;
	}
	return 0;
}

/************************
 * 函数：判断ECM是否BUSY状态
 * 返回值1：BUSY
 */
int ECM_IsAsyncBusy()
{
	uint8_t u8RetStatus = 0;
	if (ECM_InfoUpdate(&u8RetStatus, 0, 0, 0))
	{
		//得到了最新的状态，则对状态进行判断
		if (u8RetStatus & ECM_STA_ASYNC_OP_BUSY_MASK)
		{
			return 1;
		}
	}
	else
	{
		return 1; //没有得到最新状态，则置1——BUSY
	}
	return 0;
}

/************************
 * 函数：返回ECAT状态
 */
int ECM_GetRetStatus(uint8_t *pStatus)
{
	*pStatus = pRet->Head.u8Status;
	return 1;
}

/************************
 * 函数：返回ECM错误状态
 */
int ECM_GetRetErrStatus(uint8_t *pErrStatus)
{
	*pErrStatus = pRet->Head.u8ErrorStatus;
	return 1;
}

/************************
 * 函数：延时nMS，等待同步完成
 * 如果在延时时间内，ECM不BUSY，则返回1
 * 否则如果超过延时时间，ECM仍然BUSY，则输出超时信息，且返回0
 */
int ECM_WaitAsyncDone(int nMS)
{
	int i = 0;
	for (i = 0; i < nMS; i++)
	{
		if (ECM_IsAsyncBusy())
		{
			UserDelay(1000); //1ms
		}
		else
		{
			return 1;
		}
	}
	printf("Wait done timeout\n");
	return 0;
}

/************************
 * 函数：ECM初始化
 * 命令：ECM_CMD_ECAT_INIT_OP，非即时命令
 * 初始化EtherCAT網路及從站，使用EtherCAT功能前必須先初始化EtherCAT網路
 * 输出：如果初始化不成功，则返回0
 */
int ECM_EcatInit(uint16_t DCAssignActivate, uint32_t CycTime, int32_t CycShift)
{
	int i = 0;
	uint8_t IdxCheck;
	/*
	数组指针强制转换成结构体指针
	这样转换编译器其实什么也不做，只是从语法上来说类型改变而已，用新的类型的方式来解释原来内存中的值
	这样做是让结构体按照自己的属性重新读取数组中的数据。
	字节对齐
	如果结构体中所有属性所占字节和数组的类型所占字节一样（如果数组类型是16位的，属性也是16位的），那就直接按照属性的顺序，将数组中的数据依次读取出来（是读取，不是复制，因为是指针强制转换）
	字节不对齐
	如果结构体属性所占字节数和数组的类型不一样，那样数据就会乱，结构体会按照属性的类型依次读取类型长度的数据，然后数据就会乱了
	大小端
	这个自然也是很有影响的，如果写的时候默认小端模式，而芯片中确实大端模式，那样数据就会直接读成倒的了，如数组{0x1234，0x5678} ，强制转换成结构体指针后，假设默认是小端模式，作者认为结构体中两个数据应该为{data1 = 0x1234, data2 = 0x5678} ，而芯片实际是大端模式，那CPU读取的数据就是{data1 = 0x7856, data2 = 0x3412} 
	*/
	EC_DCSYNC_H *pDcSyncCmd = (EC_DCSYNC_H *)pCmd->Data;

	pDcSyncCmd->Slave = ECM_INDEX; //#define ECM_INDEX 0xFF
	// The DC sync mode usually use as following:
	// Command  Description
	//  0x00    Deactivate the sync (Free Run)
	//  0x03    Activate Sync0 (DC Sync0 mode)
	//  0x07    Activate both Sync0 and Sync1
	pDcSyncCmd->AssignActivate = DCAssignActivate; //DC sync mode
	pDcSyncCmd->CyclTime0 = CycTime;			   //SYNC0週期時間(單位ns)
	pDcSyncCmd->CyclShift = CycShift;			   //週期時間偏移量(單位ns)
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_INIT_OP;	   //初始化网络命令，属于非即时命令
	pCmd->Head.u16Size = sizeof(EC_DCSYNC_H);
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck) //返回的索引号=发送的命令索引号，表示发送命令成功
			{
				break;
			}
		}
	}
	if (i >= 100)
	{
		printf("Timeout\n");
		return 0;
	}
	return ECM_WaitAsyncDone(1000); //延时1000ms，等待ECM空闲状态
}

/************************
 * 函数：ECM重新初始化
 * 命令：ECM_CMD_ECAT_RECONFIG_OP，非即时命令
 * 當重新規劃PDO後，需使用此命令使ECM-XF重新配置記憶體空間
 * 输出：如果重新初始化不成功，则返回0
 */
int ECM_EcatReconfig()
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_RECONFIG_OP;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				break;
			}
		}
	}
	if (i >= 100)
	{
		printf("Timeout\n");
		return 0;
	}
	return ECM_WaitAsyncDone(1000);
}

/************************
 * 函数：读取从站数量
 * 命令：ECM_CMD_ECAT_SLV_CNT_GET，即时命令
 * 输出：成功，则返回从站数量；如果不成功，则返回0
 */
int8_t ECM_EcatSlvCntGet()
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SLV_CNT_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return pRet->Head.u8Return;
			}
		}
	}
	return 0;
}

/************************
 * 函数：设定EtherCAT状态
 * 命令：ECM_CMD_ECAT_STATE_SET，非即时命令
 * 输入：从站号、设定状态
 * 输出：发送成功返回1；发送不成功返回0
 */
int ECM_EcatStateSet(uint8_t u8Slave, uint8_t u8State)
{
	if (!ECM_WaitAsyncDone(2000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_STATE_SET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Param = u8Slave;
	pCmd->Head.u8Data[0] = u8State;
	return SpiDataExchange(0, 0);
}

/************************
 * 函数：获取EtherCAT状态
 * 命令：ECM_CMD_ECAT_STATE_GET，非即时命令
 * 输入：从站号、返回状态变量指针
 * 输出：读取成功则返回从站状态，存入pu8State变量中，并且返回1；如果不成功返回0
 */
int ECM_EcatStateGet(uint8_t u8Slave, uint8_t *pu8State)
{
	int i = 0;
	uint8_t IdxCheck = 0;
	if (!ECM_WaitAsyncDone(2000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_STATE_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Param = u8Slave;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				*pu8State = pRet->Head.u8Return;
				return 1;
			}
		}
	}
	return 0;
}

/************************
 * 函数：EtherCAT状态检查
 * 状态设定后，再读取出来
 * 输入：从站号、设定的期望状态，超时时间
 * 输出：设定成功，返回1；如果不成功返回0
 */
int ECM_StateCheck(uint8_t u8Slave, uint8_t u8ExpectState, int TimeOutMS)
{
	uint8_t u8State;
	int i = 0;
	if (ECM_EcatStateSet(u8Slave, u8ExpectState))
	{
		for (i = 0; i < TimeOutMS; i++)
		{
			UserDelay(1000);
			if (ECM_EcatStateGet(u8Slave, &u8State))
			{
				if (u8State == u8ExpectState)
				{
					return 1;
				}
			}
		}
	}
	return 0;
}

/************************
 * 函数：配置從站PDO
 * 命令：ECM_CMD_ECAT_PDO_CONFIG_SET，非即时命令
 * 输入：从站号、从站PDO配置数据指针
 * 输出：配置成功，则返回1；如果不成功返回0
 */
int ECM_EcatPdoConfigSet(uint8_t Slave, PDO_CONFIG_HEAD *pConfigData)
{
	if (!ECM_WaitAsyncDone(1000)) //等等1000ms后，如果仍然BUSY，则返回0
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_CONFIG_SET;
	pCmd->Head.u16Size = sizeof(PDO_CONFIG_HEAD);
	pCmd->Head.u8Idx = u8CmdIdx++;
	pConfigData->Slave = Slave;
	memcpy(pCmd->Data, pConfigData, sizeof(PDO_CONFIG_HEAD));
	if (SpiDataExchange(0, 0))
	{
		if (ECM_WaitAsyncDone(1000)) //如果1000ms内空闲，则返回1
			return 1;
	}
	return 0;
}

/************************
 * 函数：請求執行PDO配置讀取
 * 命令：ECM_CMD_ECAT_PDO_CONFIG_REQ，非即时命令
 * 請求執行PDO配置讀取, 操作完成後使用ECM_CMD_ECAT_PDO_CONFIG_GET取回配置
 * 输入：从站号、PDO指定索引号
 * 输出：配置成功，则返回1；如果不成功返回0
 */
int ECM_EcatPdoConfigReq(uint8_t Slave, uint16_t SmaIdx)
{
	if (!ECM_WaitAsyncDone(3000))
		return 0;
	PDO_CONFIG_HEAD *pTxCmd = (PDO_CONFIG_HEAD *)pCmd->Data;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_CONFIG_REQ;
	pCmd->Head.u16Size = sizeof(PDO_CONFIG_HEAD);
	pCmd->Head.u8Idx = u8CmdIdx++;
	pTxCmd->Slave = Slave;
	pTxCmd->SmaIdx = SmaIdx;
	return SpiDataExchange(0, 0);
}

/************************
 * 函数：讀取從站PDO配置
 * 命令：ECM_CMD_ECAT_PDO_CONFIG_GET，非即时命令
 * 输入：读取后数据缓冲区指针
 * 输出：读取成功，则读取的数据存入pBuf，并且返回1；如果超时仍然不成功则返回0
 */
int ECM_EcatPdoConfigGet(PDO_CONFIG_HEAD *pBuf)
{
	int i;
	uint8_t IdxCheck;
	if (!ECM_WaitAsyncDone(3000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_CONFIG_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				memcpy(pBuf, pRet->Data, sizeof(PDO_CONFIG_HEAD));
				return 1;
			}
		}
	}
	return 0;
}

/************************
 * 函数：請求執行SDO讀寫命令
 * 命令：ECM_CMD_ECAT_SDO_REQ，非即时命令
 * 請求執行SDO讀寫命令，寫操作指令資料段為寫入資料,讀操作完成後通过ECM_CMD_ECAT_SDO_GET指令取回讀取資料
 * 输入：操作码OP（0——写；1——读），从站站号（0-127），COE对象索引号，COE对象子索引号，数据长度size（写操作有效），超时时间，数据指针（写操作有效，指令写入数据）
 * 输出：成功，则返回1；如果不成功返回0
 */
int ECM_EcatSdoReq(uint8_t OP,
				   uint8_t Slave,
				   uint16_t Index,
				   uint8_t SubIndex,
				   uint16_t size,
				   int Timeout,
				   uint8_t *Data)
{

	SDO_CMD_HEAD *pSdoCmd = (SDO_CMD_HEAD *)pCmd->Data;
	if (!ECM_WaitAsyncDone(1000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SDO_REQ;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Ctrl = 0;
	pSdoCmd->OP = OP;
	pSdoCmd->Slave = Slave;
	pSdoCmd->Index = Index;
	pSdoCmd->SubIndex = SubIndex;
	pSdoCmd->size = size;
	pSdoCmd->Timeout = Timeout;
	if (OP == ECM_SDO_OP_WR)
	{
		pCmd->Head.u16Size = 12 + size; //写操作，数据长度=12+指令数据有效长度
		memcpy(pSdoCmd->Data, Data, size);
	}
	else
	{
		pCmd->Head.u16Size = 12; //读操作，数据长度=12
	}
	return SpiDataExchange(0, 0);
}

/************************
 * 函数：讀回ECM_CMD_ECAT_SDO_REQ讀操作結果
 * 命令：ECM_CMD_ECAT_SDO_GET，即时命令
 * 讀回ECM_CMD_ECAT_SDO_REQ讀操作結果
 * 输入：数据缓冲区指针
 * 输出：成功，则将读取数据存入数据缓冲区，并返回读取的数据大小；如果不成功返回0
 */
int16_t ECM_EcatSdoGet(uint8_t *pBuf)
{
	int i;
	uint8_t IdxCheck;
	if (!ECM_WaitAsyncDone(1000))
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SDO_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Ctrl = 0;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				memcpy(pBuf, pRet->Data, pRet->Head.u16Size);
				return pRet->Head.u16Size;
			}
		}
	}
	return 0;
}

/************************
 * 函数：ECM-XF內部402狀態機控制位元組設定
 * 命令：ECM_CMD_402_CTL_SET，即时命令
 * 输入：从站站号
 * 输出：成功，则返回1；如果不成功返回0
 */
int ECM_Drv402SM_Enable(uint8_t SlvIdx)
{
	pCmd->Head.u8Cmd = ECM_CMD_402_CTL_SET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Param = SlvIdx;
	pCmd->Head.u8Data[0] = (CIA402_FSM_CTL_ENABLE_MASK | CIA402_FSM_CTL_FAULT_RST_MASK);
	pCmd->Head.u8Idx = u8CmdIdx++;
	return SpiDataExchange(0, 0);
}

/************************
 * 函数：切換指定從站402狀態
 * 命令：ECM_CMD_402_STATE_SET，即时命令
 * 输入：从站站号，设定状态
 * 输出：成功，则返回1；如果不成功返回0
#define CIA402_SW_NOTREADYTOSWITCHON 0x00
#define CIA402_SW_SWITCHEDONDISABLED 0x40
#define CIA402_SW_READYTOSWITCHON 0x21
#define CIA402_SW_SWITCHEDON 0x23
#define CIA402_SW_OPERATIONENABLED 0x27
#define CIA402_SW_QUICKSTOPACTIVE 0x07
#define CIA402_SW_FAULTREACTIONACTIVE 0x0F
#define CIA402_SW_FAULT 0x08
 */
int ECM_Drv402SM_StateSet(uint8_t SlvIdx, uint8_t State)
{
	pCmd->Head.u8Cmd = ECM_CMD_402_STATE_SET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Param = SlvIdx;
	pCmd->Head.u8Data[0] = State;
	pCmd->Head.u8Idx = u8CmdIdx++;
	return SpiDataExchange(0, 0);
}

/************************
 * 函数：读取指定從站402狀態
 * 命令：ECM_CMD_402_STATE_GET，即时命令
 * 输入：从站站号，读取的状态
 * 输出：成功，则保存读取的状态，并且返回1；如果不成功返回0
 */
int ECM_Drv402SM_StateGet(uint8_t SlvIdx, uint8_t *pState)
{
	int i;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_402_STATE_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Param = SlvIdx;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				*pState = pRet->Head.u8Return;
				return 1;
			}
		}
	}
	return 0;
}

/************************
 * 函数：状态检查，设定并读取指定從站402狀態
 * 输入：从站站号，设定的期望状态，超时时间
 * 输出：设定成功，则返回1；如果超时还不成功则返回0
 */
int ECM_Drv402SM_StateCheck(uint8_t SlvIdx, uint8_t ExceptState, int TimeOutMS)
{
	int i;
	uint8_t State;
	i = ECM_Drv402SM_StateSet(SlvIdx, ExceptState);
	if (i == 0)
	{
		return 0;
	}
	for (i = 0; i < TimeOutMS; i++)
	{
		UserDelay(1000);
		if (ECM_Drv402SM_StateGet(SlvIdx, &State))
		{
			if ((State & CIA402_SW_STATE_MASK) == ExceptState)
			{
				return 1;
			}
		}
	}
	printf("(%d) 0x%X 0x%X\n", SlvIdx, State, ExceptState);
	return 0;
}

/************************
 * 函数：讀取RxPDO資料長度，FIFO內一筆資料為一個PDO
 * 命令：ECM_CMD_FIFO_PACK_SIZE_GET，即时命令
 * 输出：读取成功，则返回RxPDO数据长度；如果不成功则返回0
 */
uint16_t ECM_FifoRxPdoSizeGet()
{
	int i;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_FIFO_PACK_SIZE_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Param = 0; //	0:	RX
							//	1:	TX
	for (i = 0; i < 10; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return *((uint16_t *)pRet->Data);
			}
		}
	}
	return 0;
}

/************************
 * 函数：讀取TxPDO資料長度，FIFO內一筆資料為一個PDO
 * 命令：ECM_CMD_FIFO_PACK_SIZE_GET，即时命令
 * 输出：读取成功，则返回TxPDO数据长度；如果不成功则返回0
 */
uint16_t ECM_FifoTxPdoSizeGet()
{
	int i;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_FIFO_PACK_SIZE_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Param = 1; //	0:	RX
							//	1:	TX
	for (i = 0; i < 10; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return *((uint16_t *)pRet->Data);
			}
		}
	}
	return 0;
}

/************************
 * 函数：存取PDO資料
 * 命令：ECM_CMD_ECAT_PDO_DATA_OP，即时命令
 * 输入：操作码OP，读写数据指针及数据大小
 * #define ECM_PDO_WR_OP 1
 * #define ECM_PDO_RD_OP 2
 * 输出：写入成功，则返回1；如果读取成功则返回数据及数据长度；如果不成功则返回0
 */
uint8_t ECM_EcatPdoDataExchange(uint8_t u8OP, uint8_t *pRxData, uint8_t *pTxData, uint16_t *pu16DataSize)
{
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_DATA_OP;
	pCmd->Head.u16Size = *pu16DataSize;
	pCmd->Head.u8Idx = u8CmdIdx++;
	if (u8OP)
	{
		pCmd->Head.u8Data[0] = (ECM_PDO_HALF_OP | u8OP);
	}
	else
	{
		pCmd->Head.u8Data[0] = 0; //Read and Write
	}
	if (u8OP & ECM_PDO_WR_OP)
	{
		memcpy(pCmd->Data, pRxData, pCmd->Head.u16Size);
	}
	if (SpiDataExchange(0, 0) == 0)
	{
		return 0;
	}
	if (pRet->Head.u8Cmd == ECM_CMD_ECAT_PDO_DATA_OP)
	{
		if (pRet->Head.u8Return & ECM_PDO_RD_OP)
		{
			memcpy(pTxData, pRet->Data, pRet->Head.u16Size);
			*pu16DataSize = pRet->Head.u16Size;
		}
		return pRet->Head.u8Return;
	}
	return ECM_PDO_WR_OP;
}

/************************
 * 函数：判断接收FIFO缓冲区是否满了
 * 输出：如果缓冲区满了，则返回1；如果缓冲区没满则返回0
 */
int ECM_EcatPdoFifoIsFull(uint8_t u8FifoThreshold)
{
	// Notice : FIFO count update has two times delay
	if (pRet->Head.u8RxFifoCnt >= u8FifoThreshold - 2)
	{
		return 1; // FIFO count threshold reached
	}
	else
	{
		return 0;
	}
}

/************************
 * 函数：對FIFO存取PDO資料
 * 命令：ECM_CMD_ECAT_PDO_DATA_FIFO_OP，即时命令
 * 输入：FIFO缓冲区阀值，读写数据指针及数据大小
 * #define ECM_PDO_WR_OP 1
 * #define ECM_PDO_RD_OP 2
 * 输出：写入成功，则返回1；
 * 如果读取成功则返回数据及数据长度；
 * 如果TxPDO FIFO empty，则返回0
 * 如果FIFO缓冲区满了，则返回-2；
 * 如果CRC错误，则返回-1
 * 如果读取到的数据数量为0，则返回-4
  	Exchange data and also check FIFO count, Working counter Error and CRC error
	Checking FIFO count to prevent PDO number is more than FIFO volume
	Working counter error count is the error count between XF(Master) and EtherCAT slave(slave)
	CRC Error count is the error count between SPI(Master) and XF(slave)
 */
int ECM_EcatPdoFifoDataExchange(uint8_t u8FifoThreshold, uint8_t *pRxData, uint8_t *pTxData, uint16_t u16DataSize, uint8_t *pu8RxPdoFifoCnt, uint8_t *CrcErrCnt, uint8_t *WkcErrCnt)
{
	// Notice : FIFO count update has two times delay
	if (pRet->Head.u8RxFifoCnt >= u8FifoThreshold - 2)
	{
		return -2; // FIFO count threshold reached
	}
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_DATA_FIFO_OP;
	pCmd->Head.u16Size = u16DataSize;
	pCmd->Head.u8Param = 1;
	pCmd->Head.u8Data[0] = (ECM_FIFO_WR | ECM_FIFO_RD);
	pCmd->Head.u8Idx = u8CmdIdx++;
	memcpy(pCmd->Data, pRxData, pCmd->Head.u16Size);
	if (SpiDataExchange(0, 0) == 0)
	{
		printf("CRC error\n");
		return -1; //CRC error
	}
	if (pu8RxPdoFifoCnt)
		*pu8RxPdoFifoCnt = pRet->Head.u8RxFifoCnt; //返回RxPDO FIFO缓冲区中待发送的数据数量
	if (CrcErrCnt)
		*CrcErrCnt = pRet->Head.u8CrcErrCnt; //返回CRC错误数量
	if (WkcErrCnt)
		*WkcErrCnt = pRet->Head.u8WkcErrCnt; //返回WKC错误数量
	if (pRet->Head.u8Cmd == ECM_CMD_ECAT_PDO_DATA_FIFO_OP)
	{
		if (pRet->Head.u8Return & ECM_FIFO_RD)
		{
			if (pRet->Head.u16Size)
			{
				memcpy(pTxData, pRet->Data, pRet->Head.u16Size); //存入读取到的数据
			}
			else
			{
				printf("zero size\n");
				return -4;
			}
			return pRet->Head.u16Size;
		}
		else
		{
			printf("TxPDO FIFO empty\n");
			return 0;
		}
	}
	else
	{
		return -3;
	}
	return -6;
}

/************************
 * 函数：請求執行EEPROM讀寫命令
 * 命令：ECM_EEPROM_REQ，非即时命令
 * 請求執行EEPROM讀寫命令, 寫操作指令資料段為寫入資料,讀操作完成後透ECM_EEPROM_GET指令取回讀取資料
 * 输入：操作码OP（0-读取，1-写入），从站站号slave（0-127），读写位置eeproma，写入数据data（写操作时有效），超时时间timeout（ns）
 * 输出：如果成功，则返回1；如果不成功则返回0
 */
int ECM_EcatEepromReq(
	uint16_t OP,
	uint16_t slave,
	uint16_t eeproma,
	uint16_t data,
	uint32_t timeout)
{
	ECM_EEPROM_REQ_T *pEepromReq = (ECM_EEPROM_REQ_T *)pCmd->Data;
	if (!ECM_WaitAsyncDone(1000))
		return 0;
	pCmd->Head.u8Cmd = ECM_EEPROM_REQ;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pEepromReq->OP = OP;
	pEepromReq->slave = slave;
	pEepromReq->eeproma = eeproma;
	pEepromReq->data = data;
	pEepromReq->timeout = timeout;
	pCmd->Head.u16Size = sizeof(ECM_EEPROM_REQ_T);
	return SpiDataExchange(0, 0);
}

/************************
 * 函数：讀回ECM_EEPROM_REQ讀操作結果
 * 命令：ECM_EEPROM_GET，即时命令
 * 請求執行EEPROM讀寫命令, 寫操作指令資料段為寫入資料,讀操作完成後透ECM_EEPROM_GET指令取回讀取資料
 * 输入：读取的数据指针
 * 输出：如果成功，则存入读取到的数据，并且返回1；如果不成功则返回0
 */
int ECM_EcatEepromGet(uint64_t *pu64Data)
{
	int i;
	uint8_t IdxCheck;
	if (!ECM_WaitAsyncDone(1000))
		return 0;
	pCmd->Head.u8Cmd = ECM_EEPROM_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				memcpy(pu64Data, pRet->Data, pRet->Head.u16Size);
				return 1;
			}
		}
	}
	return 0;
}

/************************
 * 函数：显示PDO配置信息
 * 输入：从站站号，PDO指定索引号
 * #define RxPDO_ASSIGN_IDX		0x1C12
 * #define TxPDO_ASSIGN_IDX		0x1C13
 * 输出：如果成功，则显示配置数据，并且返回1；如果不成功则返回0
 */
int ECM_ShowPDOConfig(int Slave, int SmaIdx)
{
	int i = 0, j = 0;
	PDO_CONFIG_HEAD PdoConfigBuf;
	int nret = ECM_EcatPdoConfigReq(Slave, SmaIdx);
	if (nret <= 0)
	{
		return 0;
	}
	nret = ECM_EcatPdoConfigGet(&PdoConfigBuf);
	if (nret <= 0)
	{
		return 0;
	}
	PRINTF("(%d) 0x%X : \r\n", Slave, SmaIdx);
	for (i = 0; i < PdoConfigBuf.PDOCnt; i++)
	{
		PRINTF("\tPDO%d - MapIdx(0x%X)\r\n", i, PdoConfigBuf.MapIdx[i]);
		for (j = 0; j < PdoConfigBuf.ObjsCnt[i]; j++)
		{
			PRINTF("\t\t0x%X\r\n", PdoConfigBuf.Table[i][j]);
		}
	}
	return 1;
}
//	(C) COPYRIGHT 2020 NEXTW Technology Corp.
