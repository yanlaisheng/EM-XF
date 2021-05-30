/******************************************************************************
 *	File	:	EcmDriver.h
 *	Version :	0.1
 *	Date	:	2020/04/24
 *	Author	:	XFORCE
 *
 *	ECM-XF main header
 *
 *	Define
 *	1. SPI package.
 *	2. Command and data structure
 *	3. the fields of the structure.
 *	4. the flags of the field.
 *
 * @copyright (C) 2020 NEXTW TECHNOLOGY CO., LTD.. All rights reserved.
 *
 ******************************************************************************/
#ifndef _ECM_DRV_H_
#define _ECM_DRV_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define ECM_PACK_BEGIN
#define ECM_PACK __attribute__((__packed__))
#define ECM_PACK_END

#define CIA402_SW_STATE_MASK 0x6F
#define CIA402_SW_NOTREADYTOSWITCHON 0x00
#define CIA402_SW_SWITCHEDONDISABLED 0x40
#define CIA402_SW_READYTOSWITCHON 0x21
#define CIA402_SW_SWITCHEDON 0x23
#define CIA402_SW_OPERATIONENABLED 0x27
#define CIA402_SW_QUICKSTOPACTIVE 0x07
#define CIA402_SW_FAULTREACTIONACTIVE 0x0F
#define CIA402_SW_FAULT 0x08

#define ECM_INDEX 0xFF

#define EC_STATE_NONE 0x00
#define EC_STATE_INIT 0x01
#define EC_STATE_PRE_OP 0x02
#define EC_STATE_BOOT 0x03
#define EC_STATE_SAFE_OP 0x04
#define EC_STATE_OPERATIONAL 0x08
#define EC_STATE_ACK 0x10
#define EC_STATE_ERROR 0x20

#define PDO_FIFO_SIZE 8192
#define PKG_DATA_MIN_SIZE 32
#define PKG_DATA_MAX_SIZE 1408
#define PKG_DATA_DEFAULT_SIZE 112
#define PACK_HEAD_SIZE 24
#define CRC_FIELD_SIZE 4
#define PACK_STOPWORD_SIZE 4
#define PKG_MIN_SIZE 64
#define PKG_MAX_SIZE 1448
#define PKG_DEFAULT_SIZE 144
#define SPI_FIFO_MAX_SIZE 1448
#define PDO_FIFO_MAX_CNT 64
#define PDO_FIFO_DEFAULT_CNT 64
#define TXPDO_EXDATA_SIZE 16

#define ECM_ERR_LINK_PIN_MASK 0x01
#define ECM_ERR_LINK_CHK_MASK 0x02
#define ECM_ERR_ECAT_STA_MASK 0x04
#define ECM_ERR_ASYNC_CMD_MASK 0x08
#define ECM_ERR_SPI_MASK 0x10
#define ECM_ERR_FIFO_MASK 0x20
#define ECM_ERR_CRC_MASK 0x40
#define ECM_ERR_CMD_MASK 0x80

#define ECM_CRC_TYPE_NONE 0
#define ECM_CRC_TYPE_8 1
#define ECM_CRC_TYPE_16 2
#define ECM_CRC_TYPE_32 3

#define ECM_STA_CRC_TYPE_MASK 0x03
#define ECM_STA_DC_STABLE 0x04
#define ECM_STA_EMAC_INIT_MASK 0x08
#define ECM_STA_ECAT_CONFIG_MASK 0x10
#define ECM_STA_NOP_CMD_MASK 0x20
#define ECM_STA_FIFO_EN_MASK 0x40
#define ECM_STA_ASYNC_OP_BUSY_MASK 0x80

#define ECM_CTL_UPDATE_EC_STATE_MASK 0x01
#define ECM_CTL_CLR_ASYNC_OP_ERR_MASK 0x08
#define ECM_CTL_CLR_SPI_ERR_MASK 0x10
#define ECM_CTL_CLR_FIFO_ERR_MASK 0x20
#define ECM_CTL_CLR_WKC_ERR_MASK 0x40
#define ECM_CTL_CLR_ERR_MASK 0x80
#define ECM_CTL_CLR_ALL_ERR_MASK 0xF8

#define ECM_CRC_MAGIC_NUM 0x12345678
#define ECM_START_WORD 0xA1A2A3A4
#define ECM_STOP_WORD 0x56575859

#define ECM_SDO_OP_WR 0
#define ECM_SDO_OP_RD 1

#define ECM_FIFO_WR 1
#define ECM_FIFO_RD 2

#define ECM_FIFO_RXPDO 0
#define ECM_FIFO_TXPDO 1

#define ECM_PDO_WR_OP 1
#define ECM_PDO_RD_OP 2
#define ECM_PDO_HALF_OP 4

#define ECM_STATE_OP_UPDATE 0
#define ECM_STATE_OP_RD 1
#define ECM_STATE_OP_WR 2
#define ECM_STATE_OP_WRandACK 3

#define CIA402_FSM_CTL_FAULT_RST_MASK 0x08	   //bit3
#define CIA402_FSM_CTL_ENABLE_MASK 0x10		   //bit4
#define CIA402_FSM_CTL_FAULT_AUTORST_MASK 0x80 //bit7

#define ECM_EEPROM_OP_RD 0
#define ECM_EEPROM_OP_WR 1

	enum
	{
		ECM_SLV_INFO_vid,
		ECM_SLV_INFO_pid,
		ECM_SLV_INFO_rev,
		ECM_SLV_INFO_name,
		ECM_SLV_INFO_configadr,
		ECM_SLV_INFO_aliasadr,
		ECM_SLV_INFO_state,
		ECM_SLV_INFO_ALstatuscode,
		ECM_SLV_INFO_Obytes,
		ECM_SLV_INFO_Ibytes,
		ECM_SLV_INFO_hasdc,
		ECM_SLV_INFO_DCcycle,
		ECM_SLV_INFO_DCshift,
		ECM_SLV_INFO_DCactive
	};
	enum
	{
		ECM_CMD_INFO_UPDATE_OP,
		ECM_CMD_ECAT_INIT_OP,
		ECM_CMD_ECAT_RECONFIG_OP,
		ECM_CMD_ECAT_INIT_DC_OP,
		ECM_CMD_ECAT_PDO_WC_GET,
		ECM_CMD_ECAT_PDO_DATA_FIFO_OP,
		ECM_CMD_ECAT_PDO_DATA_OP,
		ECM_CMD_ECAT_PDO_CONFIG_SET,
		ECM_CMD_ECAT_PDO_CONFIG_REQ,
		ECM_CMD_ECAT_PDO_CONFIG_GET,
		ECM_CMD_ECAT_SDO_REQ,
		ECM_CMD_ECAT_SDO_GET,
		ECM_CMD_ECAT_STATE_SET,
		ECM_CMD_ECAT_STATE_GET,
		ECM_CMD_ECAT_CYCTIME_SET,
		ECM_CMD_ECAT_SLV_INFO_GET,
		ECM_CMD_ECAT_SLV_CNT_GET,
		ECM_CMD_FIFO_CONFIG,
		ECM_CMD_FIFO_ENABLE_OP,
		ECM_CMD_FIFO_PACK_SIZE_GET,
		ECM_CMD_SPI_PACK_SIZE_GET,
		ECM_CMD_SPI_TIMEOUT_SET,
		ECM_CMD_SPI_TIMEOUT_GET,
		ECM_CMD_SPI_RECONFIG_OP,
		ECM_CMD_CRC_ERR_CNT_CLR,
		ECM_CMD_CRC_TYPE_SET,
		ECM_CMD_402_CONFIG_SET,
		ECM_CMD_402_STATE_SET,
		ECM_CMD_402_STATE_GET,
		ECM_CMD_402_CTL_SET,
		ECM_CMD_402_CTL_GET,
		ECM_GPIO_CONFIG_SET,
		ECM_GPIO_FUNC_OP,
		ECM_QEI_FUNC_OP,
		ECM_DAC_FUNC_OP,
		ECM_ADC_FUNC_OP,
		ECM_WDT_FUNC_OP,
		ECM_DCM_PARAM_OP,
		ECM_EEPROM_REQ,
		ECM_EEPROM_GET,
		ECM_CMD_EMAC_RESET_OP,
		ECM_CMD_ECAT_STATE_CHECK,
		ECM_CMD_GET_VALUE,
		ECM_CMD_SET_VALUE,
		ECM_CMD_GET_VAR,
		ECM_CMD_GET_TIMER_TIME,
		ECM_CMD_COMPARE_SET_CONFIG,
		ECM_CMD_COMPARE_GET_CONFIG,
		ECM_CMD_SET_PDO_MAP,
		ECM_CMD_GET_PDO_MAP,
		ECM_CMD_ECAT_DCSYNC,
		ECM_CMD_FIFO_CLR_OP,
		ECM_CMD_FIFO_SET_TX_CNT,
		ECM_CMD_FIFO_GET_TX_CNT,
		ECM_CMD_FIFO_SET_RX_CNT,
		ECM_CMD_FIFO_GET_RX_CNT,
		ECM_CMD_FW_VERSION_GET
	};
	ECM_PACK_BEGIN
	typedef struct ECM_PACK ec_dcsync_h
	{
		uint8_t Slave;
		uint8_t Paddle;
		uint16_t AssignActivate;
		uint32_t CyclTime0;
		uint32_t CyclTime1;
		int32_t CyclShift;
	} EC_DCSYNC_H;
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK sdo_write_t
	{
		uint8_t OP;
		uint8_t Slave;
		uint16_t Index;
		uint8_t SubIndex;
		uint8_t CA;
		uint16_t size;
		int Timeout;
		uint8_t Data[256];
	} SDO_CMD_HEAD;
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK obj_entry_t
	{
		uint8_t u8BitSize;
		uint8_t u8SubIdx;
		uint16_t u16Idx;
	} OBJ_ENTRY_T;
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK pdo_config_t
	{
		uint8_t Slave;
		uint8_t PDOCnt;
		uint16_t SmaIdx;
		uint16_t MapIdx[3];
		uint16_t ObjsCnt[3];
		OBJ_ENTRY_T Table[3][8];
	} PDO_CONFIG_HEAD;
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK ecm_eeprom_req_t
	{
		uint16_t OP;
		uint16_t slave;
		uint16_t eeproma;
		uint16_t data;
		uint32_t timeout;
	} ECM_EEPROM_REQ_T;
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK spi_cmd_header
	{
		uint32_t u32StartWord;	 //封包起始字0xA1A2A3A4
		uint8_t u8Data[4];		 //参数资料0-3
		uint16_t u16Reserve;	 //保留字
		uint8_t u8Ctrl;			 //控制字节
		uint8_t u8Idx;			 //索引号
		uint8_t u8Cmd;			 //命令码
		uint8_t u8Param;		 //命令参数
		uint16_t u16Size;		 //数据段长度
		uint32_t u32CompIntClr;	 //中断控制
		uint8_t u8GpioIntClr[2]; //中断清除0-1
		uint8_t uGpio[2];		 //GPIO输出0-1
	} SPI_CMD_HEADER;			 //SPI命令头，24个字节
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK spi_cmd_package
	{
		SPI_CMD_HEADER Head; //SPI命令头
		uint8_t Data[];		 //数据
	} SPI_CMD_PACKAGE;		 //SPI命令包
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK spi_ret_header
	{
		uint32_t u32StartWord;	  //封包起始字0xA1A2A3A4
		uint8_t u8CrcErrCnt;	  //CRC错误次数
		uint8_t u8WkcErrCnt;	  //WKC错误次数
		uint8_t u8TxFifoCnt;	  //TxPDO待接收数
		uint8_t u8RxFifoCnt;	  //RxPDO待发送数
		uint8_t u8EcState;		  //ECAT状态
		uint8_t u8ErrorStatus;	  //ECM错误状态
		uint8_t u8Status;		  //ECM状态
		uint8_t u8Idx;			  //回应索引号
		uint8_t u8Cmd;			  //回应命令码
		uint8_t u8Return;		  //返回值
		uint16_t u16Size;		  //数据段长度
		uint32_t u32CompIntFlag;  //中断标志
		uint8_t u8GpioIntFlag[2]; //GPIO中断标志0-1
		uint8_t u8Gpio[2];		  //GPIO输入0-1
	} SPI_RET_HEADER;			  //SPI响应头，24个字节
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK spi_ret_package
	{
		SPI_RET_HEADER Head; //SPI响应头
		uint8_t Data[];		 //数据
	} SPI_RET_PACKAGE;		 //SPI响应包
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK _cia402_api_param_
	{
		uint8_t Ctrl;
		uint8_t TargetState;
		uint8_t ContrlWordOffset;
		uint8_t StatusWordOffset;
	} CIA402_API_PARAM;
	ECM_PACK_END
	ECM_PACK_BEGIN
	typedef struct ECM_PACK _ecm_pdo_ext_data_
	{
		uint32_t u32CompIntFlag;
		uint8_t u16GpioIntFlag[2];
		uint16_t u16AdcData;
		uint32_t u32EncCnt;
		uint16_t u16WKC;
		uint8_t u8Idx;
		uint8_t u8Reserve;
	} ECM_PDO_EXT_DATA;
	ECM_PACK_END
#ifdef __cplusplus
}
#endif

#endif
//	(C) COPYRIGHT 2020 NEXTW Technology Corp.
