/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : sshell_fml.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年5月30日
  最近修改   :
  功能描述   : SSHELL驱动头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


#ifndef _SSHELL_FML_H_
#define _SSHELL_FML_H_

#include "pubdef.h"
#include "license/license.h"

#define CPU_TYPE	"STM32H743IIT6"
#define CPU_FLASH_SIZE	"2Mbyte"
#define CPU_SRAM_SIZE	"1024Kbyte"
#define EXT_FLASH_SIZE	"32Mbyte"
#define EXT_SRAM_SIZE	"512Kbyte"

#define BUILD_DATE		        __DATE__
#define BUILD_TIME		        __TIME__

//beta版本
#define SW_VERSION_H		1
#define SW_VERSION_M		9
#define SW_VERSION_L		28



#define HW_VERSION		    "V1.00"



#define SYSINFO_PRINT()	GLOBAL_PRINT(("PROJECT NAME\t\t: %s\r\n", PROJECT_NAME));\
							GLOBAL_PRINT(("CPU TYPE\t\t: %s\r\n", CPU_TYPE));\
							GLOBAL_PRINT(("CPU SN\t\t\t: %02X%02X%02X%02X%02X%02X%02X%02X\r\n", \
											main_handle_g.p_monitor->dev_sn[0], main_handle_g.p_monitor->dev_sn[1], \
											main_handle_g.p_monitor->dev_sn[2], main_handle_g.p_monitor->dev_sn[3], \
											main_handle_g.p_monitor->dev_sn[4], main_handle_g.p_monitor->dev_sn[5], \
											main_handle_g.p_monitor->dev_sn[6], main_handle_g.p_monitor->dev_sn[7]));\
							GLOBAL_PRINT(("INT FLASH\t\t: %s\r\n", CPU_FLASH_SIZE));\
							GLOBAL_PRINT(("INT SRAM\t\t: %s\r\n", CPU_SRAM_SIZE));\
							GLOBAL_PRINT(("EXT FLASH\t\t: %s\r\n", EXT_FLASH_SIZE));\
							GLOBAL_PRINT(("EXT SRAM\t\t: %s\r\n", EXT_SRAM_SIZE));\
							GLOBAL_PRINT(("BUILD DATE\t\t: %s\t%s\r\n", BUILD_DATE, BUILD_TIME));\
							GLOBAL_PRINT(("HARDWARE VERSION\t: %s\r\n", HW_VERSION));\
							GLOBAL_PRINT(("SOFTWARE VERSION\t: v%u.%u.%u\r\n", SW_VERSION_H, SW_VERSION_M, SW_VERSION_L));\
							GLOBAL_PRINT(("SYSRUN TIME\t\t: %02dd %02d:%02d:%02d\r\n", sysup_seconds_g/86400, sysup_seconds_g%86400/3600, sysup_seconds_g%3600/60, sysup_seconds_g%60));\
							GLOBAL_PRINT(("CPU USAGE\t\t: %.1f%%\r\n", cpu_usage_g));
							//GLOBAL_PRINT(("CHIP TEMPER\t\t: %.2f ℃ \r\n", cpu_temper_g));


extern void sshell_fml_init(void);
extern  BOOL sshell_excute_cmd(U8* ptr);
#endif
/*eof*/
