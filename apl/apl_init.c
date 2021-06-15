/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : apl_init.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年5月30日
  最近修改   :
  功能描述   : APL层全局初始化
  函数列表   :
              apl_init
  修改历史   :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


/*------------------------------头文件------------------------------*/

#include "monitor/monitor_apl.h"
#include "update/update_apl.h"
#include "iotclient/iotclient.h"
#include "uwb/uwb.h"
#include "Ntrip/Ntrip.h"
#include "server/server_apl.h"


/*------------------------------头文件------------------------------*/

/*------------------------------文件宏------------------------------*/

/*------------------------------文件宏------------------------------*/

/*------------------------------文件变量------------------------------*/

/*------------------------------文件变量------------------------------*/

/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/

/*****************************************************************************
 函 数 名  : Led_light_int
 功能描述  : LED灯初始化开关灯函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月
    作    者   : zxf
    修改内容   : 创建

*****************************************************************************/
static void Led_light_int(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);
	delay_ms(1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
	delay_ms(5);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
	delay_ms(1000);


}

/*****************************************************************************
 函 数 名  : apl_init
 功能描述  : APL层初始化函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年10月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
void apl_init(void)
{
	Led_light_int();
	monitor_apl_init();
	update_apl_init();
	iotclient_init();
	Ntrip_apl_Init();
//	EH_uwb_apl_init();
	Imu_apl_init();
	Server_apl_init();
	//UWB_apl_init();
	
}
/*eof*/
