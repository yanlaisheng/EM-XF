/*
 * platform.c
 *
 *  Created on: 2020骞?3鏈?19鏃?
 *      Author: bhliong.tw
 */

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "platform.h"
#include "EcmUsrDriver.h"
#define CEIL_4(a) (((a) + 3) & ~0x3) //向上取整
#define CEIL_2(a) (((a) + 1) & ~0x1) //向上取整

extern TIM_HandleTypeDef htim2;
extern SPI_HandleTypeDef hspi1;
extern CRC_HandleTypeDef hcrc;

static __inline int ECMSPIReadWrite(uint8_t *rdata, uint8_t *wdata, int rwlength)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, wdata, rdata, rwlength, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	return 0;
}

void SYS_Init(void)
{
}

int UserSpiDataExchange(uint8_t *pTxBuf, uint8_t *pRxBuf, uint32_t u32PackSize)
{
	ECMSPIReadWrite(pRxBuf, pTxBuf, u32PackSize);
	return 1;
}

unsigned int
crc32(const void *buf, size_t size);
uint32_t g_u32CrcType = ECM_CRC_TYPE_32;

int UserDelay(uint32_t uSec)
{
	uint32_t curTicks, dlyTicks, dlyCnt, tmcnt, stratcnt;
	dlyTicks = uSec / 1000U;
	dlyCnt = uSec % 1000U * 84;

	curTicks = HAL_GetTick();
	while ((HAL_GetTick() - curTicks) < dlyTicks)
	{
		__NOP();
	}

	stratcnt = htim2.Instance->CNT;
	do
	{
		tmcnt = htim2.Instance->CNT;
	} while (tmcnt - stratcnt < dlyCnt);

	return 1;
}

uint32_t UserSetCrcType(uint32_t u32CrcType)
{
	if (u32CrcType <= 3)
		g_u32CrcType = u32CrcType;
	return g_u32CrcType;
}
uint32_t UserCalCrc(uint8_t *pu8Addr, uint32_t u32Size)
{
	if (g_u32CrcType == ECM_CRC_TYPE_NONE)
		return ECM_CRC_MAGIC_NUM;
	else if (g_u32CrcType == ECM_CRC_TYPE_32)
		return crc32(pu8Addr, CEIL_4(u32Size));

	return 0;
}
