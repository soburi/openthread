#include <openthread-core-config.h>
#include <openthread/config.h>

#include <stddef.h>
#include <stdint.h>
#include <openthread/platform/toolchain.h>
#include <openthread/platform/uart.h>

#include "platform-jn516x.h"


#if (UART_AS_SERIAL_TRANSPORT == 1)

otError otPlatUartFlush(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

void jn516xUartProcess(void)
{
}

void jn516xUartInit(void)
{
    // Intentionally empty.
}

void jn516xUartClearPendingData(void)
{
    // Intentionally empty.
}

void jn516xUartDeinit(void)
{
}

otError otPlatUartEnable(void)
{
	return 0;
}

otError otPlatUartDisable(void)
{
	return 0;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
	(void)aBuf;
	(void)aBufLength;
	return 0;
}

#endif // UART_AS_SERIAL_TRANSPORT == 1

/**
 * The UART driver weak functions definition.
 *
 */
OT_TOOL_WEAK void otPlatUartSendDone(void)
{
}

OT_TOOL_WEAK void otPlatUartReceived(const uint8_t *aBuf, uint16_t aBufLength)
{
	(void)aBuf;
	(void)aBufLength;
}
