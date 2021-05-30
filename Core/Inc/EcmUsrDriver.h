/******************************************************************************
 *	File	:	EcmUsrDriver.h
 *	Version :	0.1
 *	Date	:	2020/04/24
 *	Author	:	XFORCE
 *
 *	ECM-XF basic driver example - Header file
 *
 *	Demonstrate how to implement API type user driver
 *
 * @copyright (C) 2020 NEXTW TECHNOLOGY CO., LTD.. All rights reserved.
 *
 ******************************************************************************/

#ifndef _ECM_USR_DRV_H_
#define _ECM_USR_DRV_H_

#include <stdio.h>
#include <string.h>
#include "EcmDriver.h"
#include "PdoDefine.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define TEST_CYCTIME 250000
#define TEST_SERVO_CNT 2
#define TEST_SPI_DATA_SIZE PKG_DATA_DEFAULT_SIZE //112
#define TEST_FIFO_CNT PDO_FIFO_DEFAULT_CNT		 //64
//6*5+12
#define TEST_RXPDO_SIZE 6
#define TEST_RXPDO_BUF_SIZE (TEST_RXPDO_SIZE * TEST_SERVO_CNT) //6*2

	typedef struct ECM_PACK spi_cmd_package_t
	{
		SPI_CMD_HEADER Head;			  //命令头，结构体，24个字节
		uint8_t Data[TEST_SPI_DATA_SIZE]; //数据112个字节，范围：32——1408个字节
		uint32_t Crc;
		uint32_t StopWord;
	} SPI_CMD_PACKAGE_T; //共144个字节
	typedef struct ECM_PACK spi_ret_package_t
	{
		SPI_RET_HEADER Head;
		uint8_t Data[TEST_SPI_DATA_SIZE];
		uint32_t Crc;
		uint32_t StopWord;
	} SPI_RET_PACKAGE_T; //返回数据结构体，同样144个字节

	int SpiDataExchange(uint8_t *RetIdx, uint8_t *RetCmd);
	int ECM_SetCRCTypet(uint8_t u8CRCType);
	uint16_t ECM_GetSlaveVenderID(uint8_t u8Slave);
	int ECM_GetFirmwareVersion(uint8_t *pVersion);
	int ECM_InfoUpdate(uint8_t *pEcmStatus, uint8_t *pRxPDOFifoCnt, uint8_t *CrcErrCnt, uint8_t *WkcErrCnt);
	int ECM_EcatInit(uint16_t DCAssignActivate, uint32_t CycTime, int32_t CycShift);
	int ECM_GetRetStatus(uint8_t *pStatus);
	int ECM_GetRetErrStatus(uint8_t *pErrStatus);

	int ECM_EcatReconfig();
	int8_t ECM_EcatSlvCntGet();
	int ECM_EcatStateSet(uint8_t u8Slave, uint8_t u8State);
	int ECM_EcatStateGet(uint8_t u8Slave, uint8_t *pu8State);
	int ECM_EcatPdoConfigSet(uint8_t Slave, PDO_CONFIG_HEAD *pConfigData);
	int ECM_EcatPdoConfigReq(uint8_t Slave, uint16_t SmaIdx);
	int ECM_EcatPdoConfigGet(PDO_CONFIG_HEAD *pBuf);
	int ECM_EcatSdoReq(uint8_t OP, uint8_t Slave, uint16_t Index, uint8_t SubIndex, uint16_t size, int Timeout, uint8_t *Data);
	int16_t ECM_EcatSdoGet(uint8_t *pBuf);
	int ECM_Drv402SM_Enable(uint8_t SlvIdx);
	int ECM_Drv402SM_StateSet(uint8_t SlvIdx, uint8_t State);
	int ECM_Drv402SM_StateGet(uint8_t SlvIdx, uint8_t *pState);
	int ECM_Drv402SM_StateCheck(uint8_t SlvIdx, uint8_t ExceptState, int TimeOutMS);
	uint16_t ECM_FifoRxPdoSizeGet();
	uint16_t ECM_FifoTxPdoSizeGet();
	uint8_t ECM_EcatPdoDataExchange(uint8_t u8OP, uint8_t *pRxData, uint8_t *pTxData, uint16_t *pu16DataSize);
	int ECM_EcatPdoFifoDataExchange(uint8_t u8FifoThreshold, uint8_t *pRxData, uint8_t *pTxData, uint16_t u16DataSize, uint8_t *pRxPDOFifoCnt, uint8_t *CrcErrCnt, uint8_t *WkcErrCnt);
	int ECM_EcatPdoFifoIsFull(uint8_t u8FifoThreshold);
	int ECM_EcatEepromReq(uint16_t OP, uint16_t slave, uint16_t eeproma, uint16_t data, uint32_t timeout);
	int ECM_EcatEepromGet(uint64_t *pu64Data);

	int ECM_ShowPDOConfig(int Slave, int SmaIdx);
	int ECM_StateCheck(uint8_t u8Slave, uint8_t u8ExpectState, int TimeOutMS);
#ifdef __cplusplus
}
#endif

#endif
