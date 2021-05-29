#ifndef _ECM_PLATFORM_H_
#define _ECM_PLATFORM_H_

#ifdef __cplusplus
extern "C"
{
#endif

int UserSpiDataExchange(uint8_t *pTxBuf, uint8_t *pRxBuf, uint32_t u32PackSize);
void SYS_Init(void);
int UserDelay(uint32_t uSec);
uint32_t UserSetCrcType(uint32_t u32CrcType);
uint32_t UserCalCrc(uint8_t *pu8Addr, uint32_t u32Size);
#ifdef __cplusplus
}
#endif

#endif
