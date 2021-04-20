/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : monitor_apl.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年5月30日
  最近修改   :
  功能描述   : monitor头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

#ifndef _MONITOR_APL_H_
#define _MONITOR_APL_H_

#include "pubdef.h"


extern F32 cpu_usage_g;
extern U32 sysup_seconds_g;
extern F32 cpu_temper_g;
extern U16 TIMER_SHIFT;
extern u16 TAG_ID_SUM;


typedef struct
{
	U8 dev_id;
	U8 dev_sn[8];					/* 设备序列号 */
	U8 dev_mac[6];					/* 设备MAC地址 */
	F32 cpu_useage;				    /*  */
	F32 cpu_temper;
	F32 env_temper;
	U32 state_word1;				/* 状态字1 */
}monitor_status_t;
extern void monitor_apl_init(void);

#endif
/*eof*/
