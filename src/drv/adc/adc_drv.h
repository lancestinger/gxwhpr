/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : adc_drv.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年8月23日
  最近修改   :
  功能描述   : 内部ADC驱动头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年8月23日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


#ifndef _ADC_DRV_H_
#define _ADC_DRV_H_

#include "pubdef.h"

//ADC采样通道
typedef enum
{
	ADC_1 = 0,
	ADC_2,
	ADC_3,
	ADC_NUM,
}adc_chn_enum;

#define ADC_TEMPERATURE ADC_1
#define ADC_ANT0 ADC_2
#define ADC_ANT1 ADC_3

#define ADC_MAX_VALUE	(pow(2,16))

#define ADC2VOLTAGE(x)	(x/ADC_MAX_VALUE * 3.0)

//ADC配置数据结构
typedef struct
{
	ADC_HandleTypeDef* 			adc_handle;			/* 操作对象地址 */
	ADC_TypeDef*				adc_sel;				/* ADC选择 */
	U32							chn;
	U32							clk_precaler;			/* 时钟分频系数 */
	U32							adc_resolution;		/* AD分辨率 */
	U32							gpio_enable;			/* GPIO使能 */
	GPIO_TypeDef*				gpio_port;			/* GPIO端口 */
	U32							gpio_pin;			/* GPIO PIN */
}adc_cfg_t;

extern void adc_drv_init(void);
extern U16 get_adc_value(adc_chn_enum adc_ch);
extern F32 adc_get_temperature(void);

#endif
/*eof*/

