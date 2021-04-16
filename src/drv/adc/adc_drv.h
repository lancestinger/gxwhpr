/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : adc_drv.h
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��8��23��
  ����޸�   :
  ��������   : �ڲ�ADC����ͷ�ļ�
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��8��23��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


#ifndef _ADC_DRV_H_
#define _ADC_DRV_H_

#include "pubdef.h"

//ADC����ͨ��
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

//ADC�������ݽṹ
typedef struct
{
	ADC_HandleTypeDef* 			adc_handle;			/* ���������ַ */
	ADC_TypeDef*				adc_sel;				/* ADCѡ�� */
	U32							chn;
	U32							clk_precaler;			/* ʱ�ӷ�Ƶϵ�� */
	U32							adc_resolution;		/* AD�ֱ��� */
	U32							gpio_enable;			/* GPIOʹ�� */
	GPIO_TypeDef*				gpio_port;			/* GPIO�˿� */
	U32							gpio_pin;			/* GPIO PIN */
}adc_cfg_t;

extern void adc_drv_init(void);
extern U16 get_adc_value(adc_chn_enum adc_ch);
extern F32 adc_get_temperature(void);

#endif
/*eof*/

