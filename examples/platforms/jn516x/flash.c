#include <openthread-core-config.h>
#include <openthread/config.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <utils/code_utils.h>
#include <utils/flash.h>
#include <openthread/platform/alarm-milli.h>

otError utilsFlashInit(void)
{
	return 0;
}
uint32_t utilsFlashGetSize(void)
{
	return 0;
}
otError utilsFlashErasePage(uint32_t aAddress)
{
	(void)aAddress;
	return 0;
}
otError utilsFlashStatusWait(uint32_t aTimeout)
{
	(void)aTimeout;
	return 0;
}
uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
	(void)aAddress;
	(void)aData;
	(void)aSize;
	return 0;
}
uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
	(void)aAddress;
	(void)aData;
	(void)aSize;
	return 0;
}
