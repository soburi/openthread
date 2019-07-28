#ifndef PLATFORM_JN516X_H_
#define PLATFORM_JN516X_H_

#include <jendefs.h>
#include <AppHardwareApi.h>
#include <dbg.h>

#define __WEAK __attribute__((weak))
#define __INLINE inline

struct otInstance;

extern void jn516xUartInit(void);
extern void jn516xUartDeinit(void);
extern void jn516xUartProcess(void);
extern void jn516xUartClearPendingData(void);
extern void jn516xLogInit(void);
extern void jn516xLogDeinit(void);
extern void jn516xAlarmInit(void);
extern void jn516xAlarmDeinit(void);
extern void jn516xAlarmProcess(struct otInstance* aInstance);
extern void jn516xRandomInit(void);
extern void jn516xRandomDeinit(void);
extern void jn516xSpiSlaveInit(void);
extern void jn516xSpiSlaveDeinit(void);
extern void jn516xSpiSlaveProcess(void);
extern void jn516xMiscInit(void);
extern void jn516xMiscDeinit(void);
extern void jn516xRadioInit(void);
extern void jn516xRadioDeinit(void);
extern void jn516xRadioProcess(struct otInstance* aInstance);
extern void jn516xTempInit(void);
extern void jn516xTempDeinit(void);
extern void jn516xTempProcess(void);

#endif //PLATFORM_JN516X_H_
