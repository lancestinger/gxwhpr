#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "platform/deca/deca_sleep.h"
#include "platform/port/port.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////

#define FLASH_WAITETIME  50000U          //FLASH等待超时时间

extern void flash_write(u8 *buf, u16 len);
extern void flash_read(u8 *buf, u16 len);


#endif






