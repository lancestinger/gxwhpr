#ifndef __DELAY_H
#define __DELAY_H 			   
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "platform/deca/deca_sleep.h"
#include "platform/port/port.h"


void delay_init(u8 SYSCLK);
//void delay_ms(u32 nms);
void delay_us(u32 nus);
void delay_10us(u32 nms);
extern u32 get_cur_time(void);
u32 get_count_time(u32 start_count, u32 end_count);
void test_time(void);



#endif





























