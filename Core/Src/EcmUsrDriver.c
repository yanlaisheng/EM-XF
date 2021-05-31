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

uint8_t u8TxBuf[PKG_MAX_SIZE] = {0}; //���ͻ�����1448
uint8_t u8RxBuf[PKG_MAX_SIZE] = {0}; //���ջ�����1448

SPI_CMD_PACKAGE_T *pCmd = (SPI_CMD_PACKAGE_T *)u8TxBuf; //ȫ��ָ�룬��������ָ�룬ָ���ͻ�����
SPI_RET_PACKAGE_T *pRet = (SPI_RET_PACKAGE_T *)u8RxBuf; //ȫ��ָ�룬��������ָ�룬ָ����ջ�����

uint8_t u8CmdIdx = 0; //����������

/************************
 * ������SPI�����շ�
 * ���ܣ�SPI���ݷ��ͺͽ���
 * ����UserSpiDataExchange���������ͻ�����u8TxBuf�����ջ�����u8RxBuf�����ȣ�Ĭ��144���ֽ�
 * �������������ֵ����ָ�롢����ֵ����ָ��
 * ������Է�����ֵ����CRCУ�����CRCУ����ȷ���򷵻�1��
 * ���CRCУ�鲻��ȷ�������Ƿ�������ƫ�ƣ���ƫ�ƣ����ٴη��ͣ�������0
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
		//���CRCУ����ȷ
		if (RetIdx)
			*RetIdx = pRet->Head.u8Idx; //��Ӧ������
		if (RetCmd)
			*RetCmd = pRet->Head.u8Cmd; //��Ӧ������
		return 1;						//�򷵻�1
	}
	// CRC Check Fail.
	for (i = 0; i < sizeof(SPI_CMD_PACKAGE_T) - 3; i++)
	{
		//�ڽ��ջ������в����Ƿ����������0x59/0x58/0x57/0x56��4����ֵ
		if (u8RxBuf[i] == 0x59 && u8RxBuf[i + 1] == 0x58 && u8RxBuf[i + 2] == 0x57 && u8RxBuf[i + 3] == 0x56)
			break;
	}
	if (i >= sizeof(SPI_CMD_PACKAGE_T) - 4) //���û�ҵ����򱨴�
		PRINTF("CRC Error(%d)\r\n", pCmd->Head.u8Idx);
	else // correct data shift
	{
		//����ҵ��ˣ���˵����ƫ�ƣ��ٽ����շ�
		PRINTF("shift (%d)\r\n", sizeof(SPI_CMD_PACKAGE_T) - i - 4);
		UserSpiDataExchange(u8TxBuf, u8RxBuf, i + 4);
		UserDelay(2000); //2ms
	}
	return 0;
}

/************************
 * ���������Ӳ���汾
 * ���ECM_CMD_FW_VERSION_GET
 * ���������Ӳ���汾��ַָ��
 * ������������ֵ�������ŵ��ڷ�������������ţ���õ���ȷ�İ汾�ţ�������1��
 * ���򣬲�ѯ100�κ���Ȼ�ò�����ȷ�İ汾�ţ��򷵻�0
 */
int ECM_GetFirmwareVersion(uint8_t *pVersion)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_FW_VERSION_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++; //�����ţ���Ҫ��;���R�e��
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
 * ��������ó�����
 * ���ECM_CMD_ECAT_SLV_INFO_GET
 * �����������վվ��
 * ������������ֵ�������ŵ��ڷ�������������ţ���õ���ȷ�İ汾�ţ�������1��
 * ���򣬲�ѯ10�κ���Ȼ�ò�����ȷ�İ汾�ţ��򷵻�0
 */
uint16_t ECM_GetSlaveVenderID(uint8_t u8Slave)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_SLV_INFO_GET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Param = u8Slave;

	// �������[0]=��վ�YӍ�a
	// 0 : �S�̴a
	// 1 : �S�̮aƷ�a
	// 2 : �aƷ�汾̖
	// 3 : �aƷ���Q
	// 4 : ����λ��
	// 5 : λ�Äe��
	// 6 : ��B
	// 7 : AL��B
	// 8 : ݔ���Y���L��
	// 9 : ݔ���Y���L��
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
 * ����������CRCУ������
 * ���ECM_CMD_CRC_TYPE_SET
 */
int ECM_SetCRCTypet(uint8_t u8CRCType)
{
	pCmd->Head.u8Cmd = ECM_CMD_CRC_TYPE_SET;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	pCmd->Head.u8Param = u8CRCType;
	return SpiDataExchange(0, 0); //0,0��ʾֻ������������������ź������
}

/************************
 * ���������±�ͷ���ϣ�����ȡ���µ�ECM״̬
 * ���ECM_CMD_INFO_UPDATE_OP
 * �����������³ɹ����򷵻�ECM״̬��RxPDO����������CRC��������WKC��������������1
 * ������²��ɹ����򷵻�0
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
 * �������ж�ECM�Ƿ�BUSY״̬
 * ����ֵ1��BUSY
 */
int ECM_IsAsyncBusy()
{
	uint8_t u8RetStatus = 0;
	if (ECM_InfoUpdate(&u8RetStatus, 0, 0, 0))
	{
		//�õ������µ�״̬�����״̬�����ж�
		if (u8RetStatus & ECM_STA_ASYNC_OP_BUSY_MASK)
		{
			return 1;
		}
	}
	else
	{
		return 1; //û�еõ�����״̬������1����BUSY
	}
	return 0;
}

/************************
 * ����������ECAT״̬
 */
int ECM_GetRetStatus(uint8_t *pStatus)
{
	*pStatus = pRet->Head.u8Status;
	return 1;
}

/************************
 * ����������ECM����״̬
 */
int ECM_GetRetErrStatus(uint8_t *pErrStatus)
{
	*pErrStatus = pRet->Head.u8ErrorStatus;
	return 1;
}

/************************
 * ��������ʱnMS���ȴ�ͬ�����
 * �������ʱʱ���ڣ�ECM��BUSY���򷵻�1
 * �������������ʱʱ�䣬ECM��ȻBUSY���������ʱ��Ϣ���ҷ���0
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
 * ������ECM��ʼ��
 * ���ECM_CMD_ECAT_INIT_OP���Ǽ�ʱ����
 * ��ʼ��EtherCAT�W·����վ��ʹ��EtherCAT����ǰ����ȳ�ʼ��EtherCAT�W·
 * ����������ʼ�����ɹ����򷵻�0
 */
int ECM_EcatInit(uint16_t DCAssignActivate, uint32_t CycTime, int32_t CycShift)
{
	int i = 0;
	uint8_t IdxCheck;
	/*
	����ָ��ǿ��ת���ɽṹ��ָ��
	����ת����������ʵʲôҲ������ֻ�Ǵ��﷨����˵���͸ı���ѣ����µ����͵ķ�ʽ������ԭ���ڴ��е�ֵ
	���������ýṹ�尴���Լ����������¶�ȡ�����е����ݡ�
	�ֽڶ���
	����ṹ��������������ռ�ֽں������������ռ�ֽ�һ�����������������16λ�ģ�����Ҳ��16λ�ģ����Ǿ�ֱ�Ӱ������Ե�˳�򣬽������е��������ζ�ȡ�������Ƕ�ȡ�����Ǹ��ƣ���Ϊ��ָ��ǿ��ת����
	�ֽڲ�����
	����ṹ��������ռ�ֽ�������������Ͳ�һ�����������ݾͻ��ң��ṹ��ᰴ�����Ե��������ζ�ȡ���ͳ��ȵ����ݣ�Ȼ�����ݾͻ�����
	��С��
	�����ȻҲ�Ǻ���Ӱ��ģ����д��ʱ��Ĭ��С��ģʽ����оƬ��ȷʵ���ģʽ���������ݾͻ�ֱ�Ӷ��ɵ����ˣ�������{0x1234��0x5678} ��ǿ��ת���ɽṹ��ָ��󣬼���Ĭ����С��ģʽ��������Ϊ�ṹ������������Ӧ��Ϊ{data1 = 0x1234, data2 = 0x5678} ����оƬʵ���Ǵ��ģʽ����CPU��ȡ�����ݾ���{data1 = 0x7856, data2 = 0x3412} 
	*/
	EC_DCSYNC_H *pDcSyncCmd = (EC_DCSYNC_H *)pCmd->Data;

	pDcSyncCmd->Slave = ECM_INDEX; //#define ECM_INDEX 0xFF
	// The DC sync mode usually use as following:
	// Command  Description
	//  0x00    Deactivate the sync (Free Run)
	//  0x03    Activate Sync0 (DC Sync0 mode)
	//  0x07    Activate both Sync0 and Sync1
	pDcSyncCmd->AssignActivate = DCAssignActivate; //DC sync mode
	pDcSyncCmd->CyclTime0 = CycTime;			   //SYNC0�L�ڕr�g(��λns)
	pDcSyncCmd->CyclShift = CycShift;			   //�L�ڕr�gƫ����(��λns)
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_INIT_OP;	   //��ʼ������������ڷǼ�ʱ����
	pCmd->Head.u16Size = sizeof(EC_DCSYNC_H);
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck) //���ص�������=���͵����������ţ���ʾ��������ɹ�
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
	return ECM_WaitAsyncDone(1000); //��ʱ1000ms���ȴ�ECM����״̬
}

/************************
 * ������ECM���³�ʼ��
 * ���ECM_CMD_ECAT_RECONFIG_OP���Ǽ�ʱ����
 * ������Ҏ��PDO�ᣬ��ʹ�ô�����ʹECM-XF��������ӛ���w���g
 * �����������³�ʼ�����ɹ����򷵻�0
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
 * ��������ȡ��վ����
 * ���ECM_CMD_ECAT_SLV_CNT_GET����ʱ����
 * ������ɹ����򷵻ش�վ������������ɹ����򷵻�0
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
 * �������趨EtherCAT״̬
 * ���ECM_CMD_ECAT_STATE_SET���Ǽ�ʱ����
 * ���룺��վ�š��趨״̬
 * ��������ͳɹ�����1�����Ͳ��ɹ�����0
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
 * ��������ȡEtherCAT״̬
 * ���ECM_CMD_ECAT_STATE_GET���Ǽ�ʱ����
 * ���룺��վ�š�����״̬����ָ��
 * �������ȡ�ɹ��򷵻ش�վ״̬������pu8State�����У����ҷ���1��������ɹ�����0
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
 * ������EtherCAT״̬���
 * ״̬�趨���ٶ�ȡ����
 * ���룺��վ�š��趨������״̬����ʱʱ��
 * ������趨�ɹ�������1��������ɹ�����0
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
 * ���������Ï�վPDO
 * ���ECM_CMD_ECAT_PDO_CONFIG_SET���Ǽ�ʱ����
 * ���룺��վ�š���վPDO��������ָ��
 * ��������óɹ����򷵻�1��������ɹ�����0
 */
int ECM_EcatPdoConfigSet(uint8_t Slave, PDO_CONFIG_HEAD *pConfigData)
{
	if (!ECM_WaitAsyncDone(1000)) //�ȵ�1000ms�������ȻBUSY���򷵻�0
		return 0;
	pCmd->Head.u8Cmd = ECM_CMD_ECAT_PDO_CONFIG_SET;
	pCmd->Head.u16Size = sizeof(PDO_CONFIG_HEAD);
	pCmd->Head.u8Idx = u8CmdIdx++;
	pConfigData->Slave = Slave;
	memcpy(pCmd->Data, pConfigData, sizeof(PDO_CONFIG_HEAD));
	if (SpiDataExchange(0, 0))
	{
		if (ECM_WaitAsyncDone(1000)) //���1000ms�ڿ��У��򷵻�1
			return 1;
	}
	return 0;
}

/************************
 * ������Ո�����PDO�����xȡ
 * ���ECM_CMD_ECAT_PDO_CONFIG_REQ���Ǽ�ʱ����
 * Ո�����PDO�����xȡ, ���������ʹ��ECM_CMD_ECAT_PDO_CONFIG_GETȡ������
 * ���룺��վ�š�PDOָ��������
 * ��������óɹ����򷵻�1��������ɹ�����0
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
 * �������xȡ��վPDO����
 * ���ECM_CMD_ECAT_PDO_CONFIG_GET���Ǽ�ʱ����
 * ���룺��ȡ�����ݻ�����ָ��
 * �������ȡ�ɹ������ȡ�����ݴ���pBuf�����ҷ���1�������ʱ��Ȼ���ɹ��򷵻�0
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
 * ������Ո�����SDO�x������
 * ���ECM_CMD_ECAT_SDO_REQ���Ǽ�ʱ����
 * Ո�����SDO�x�����������ָ���Y�϶Ξ錑���Y��,�x���������ͨ��ECM_CMD_ECAT_SDO_GETָ��ȡ���xȡ�Y��
 * ���룺������OP��0����д��1������������վվ�ţ�0-127����COE���������ţ�COE�����������ţ����ݳ���size��д������Ч������ʱʱ�䣬����ָ�루д������Ч��ָ��д�����ݣ�
 * ������ɹ����򷵻�1��������ɹ�����0
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
		pCmd->Head.u16Size = 12 + size; //д���������ݳ���=12+ָ��������Ч����
		memcpy(pSdoCmd->Data, Data, size);
	}
	else
	{
		pCmd->Head.u16Size = 12; //�����������ݳ���=12
	}
	return SpiDataExchange(0, 0);
}

/************************
 * �������x��ECM_CMD_ECAT_SDO_REQ�x�����Y��
 * ���ECM_CMD_ECAT_SDO_GET����ʱ����
 * �x��ECM_CMD_ECAT_SDO_REQ�x�����Y��
 * ���룺���ݻ�����ָ��
 * ������ɹ����򽫶�ȡ���ݴ������ݻ������������ض�ȡ�����ݴ�С��������ɹ�����0
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
 * ������ECM-XF�Ȳ�402��B�C����λԪ�M�O��
 * ���ECM_CMD_402_CTL_SET����ʱ����
 * ���룺��վվ��
 * ������ɹ����򷵻�1��������ɹ�����0
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
 * �������ГQָ����վ402��B
 * ���ECM_CMD_402_STATE_SET����ʱ����
 * ���룺��վվ�ţ��趨״̬
 * ������ɹ����򷵻�1��������ɹ�����0
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
 * ��������ȡָ����վ402��B
 * ���ECM_CMD_402_STATE_GET����ʱ����
 * ���룺��վվ�ţ���ȡ��״̬
 * ������ɹ����򱣴��ȡ��״̬�����ҷ���1��������ɹ�����0
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
 * ������״̬��飬�趨����ȡָ����վ402��B
 * ���룺��վվ�ţ��趨������״̬����ʱʱ��
 * ������趨�ɹ����򷵻�1�������ʱ�����ɹ��򷵻�0
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
 * �������xȡRxPDO�Y���L�ȣ�FIFO��һ�P�Y�Ϟ�һ��PDO
 * ���ECM_CMD_FIFO_PACK_SIZE_GET����ʱ����
 * �������ȡ�ɹ����򷵻�RxPDO���ݳ��ȣ�������ɹ��򷵻�0
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
 * �������xȡTxPDO�Y���L�ȣ�FIFO��һ�P�Y�Ϟ�һ��PDO
 * ���ECM_CMD_FIFO_PACK_SIZE_GET����ʱ����
 * �������ȡ�ɹ����򷵻�TxPDO���ݳ��ȣ�������ɹ��򷵻�0
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
 * ��������ȡPDO�Y��
 * ���ECM_CMD_ECAT_PDO_DATA_OP����ʱ����
 * ���룺������OP����д����ָ�뼰���ݴ�С
 * #define ECM_PDO_WR_OP 1
 * #define ECM_PDO_RD_OP 2
 * �����д��ɹ����򷵻�1�������ȡ�ɹ��򷵻����ݼ����ݳ��ȣ�������ɹ��򷵻�0
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
 * �������жϽ���FIFO�������Ƿ�����
 * �����������������ˣ��򷵻�1�����������û���򷵻�0
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
 * ��������FIFO��ȡPDO�Y��
 * ���ECM_CMD_ECAT_PDO_DATA_FIFO_OP����ʱ����
 * ���룺FIFO��������ֵ����д����ָ�뼰���ݴ�С
 * #define ECM_PDO_WR_OP 1
 * #define ECM_PDO_RD_OP 2
 * �����д��ɹ����򷵻�1��
 * �����ȡ�ɹ��򷵻����ݼ����ݳ��ȣ�
 * ���TxPDO FIFO empty���򷵻�0
 * ���FIFO���������ˣ��򷵻�-2��
 * ���CRC�����򷵻�-1
 * �����ȡ������������Ϊ0���򷵻�-4
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
		*pu8RxPdoFifoCnt = pRet->Head.u8RxFifoCnt; //����RxPDO FIFO�������д����͵���������
	if (CrcErrCnt)
		*CrcErrCnt = pRet->Head.u8CrcErrCnt; //����CRC��������
	if (WkcErrCnt)
		*WkcErrCnt = pRet->Head.u8WkcErrCnt; //����WKC��������
	if (pRet->Head.u8Cmd == ECM_CMD_ECAT_PDO_DATA_FIFO_OP)
	{
		if (pRet->Head.u8Return & ECM_FIFO_RD)
		{
			if (pRet->Head.u16Size)
			{
				memcpy(pTxData, pRet->Data, pRet->Head.u16Size); //�����ȡ��������
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
 * ������Ո�����EEPROM�x������
 * ���ECM_EEPROM_REQ���Ǽ�ʱ����
 * Ո�����EEPROM�x������, ������ָ���Y�϶Ξ錑���Y��,�x���������͸ECM_EEPROM_GETָ��ȡ���xȡ�Y��
 * ���룺������OP��0-��ȡ��1-д�룩����վվ��slave��0-127������дλ��eeproma��д������data��д����ʱ��Ч������ʱʱ��timeout��ns��
 * ���������ɹ����򷵻�1��������ɹ��򷵻�0
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
 * �������x��ECM_EEPROM_REQ�x�����Y��
 * ���ECM_EEPROM_GET����ʱ����
 * Ո�����EEPROM�x������, ������ָ���Y�϶Ξ錑���Y��,�x���������͸ECM_EEPROM_GETָ��ȡ���xȡ�Y��
 * ���룺��ȡ������ָ��
 * ���������ɹ���������ȡ�������ݣ����ҷ���1��������ɹ��򷵻�0
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
 * ��������ʾPDO������Ϣ
 * ���룺��վվ�ţ�PDOָ��������
 * #define RxPDO_ASSIGN_IDX		0x1C12
 * #define TxPDO_ASSIGN_IDX		0x1C13
 * ���������ɹ�������ʾ�������ݣ����ҷ���1��������ɹ��򷵻�0
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
