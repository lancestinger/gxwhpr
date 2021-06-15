/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : apl_init.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��5��30��
  ����޸�   :
  ��������   : APL��ȫ�ֳ�ʼ��
  �����б�   :
              apl_init
  �޸���ʷ   :
  1.��    ��   : 2019��5��30��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


/*------------------------------ͷ�ļ�------------------------------*/

#include "monitor/monitor_apl.h"
#include "update/update_apl.h"
#include "iotclient/iotclient.h"
#include "uwb/uwb.h"
#include "Ntrip/Ntrip.h"
#include "server/server_apl.h"


/*------------------------------ͷ�ļ�------------------------------*/

/*------------------------------�ļ���------------------------------*/

/*------------------------------�ļ���------------------------------*/

/*------------------------------�ļ�����------------------------------*/

/*------------------------------�ļ�����------------------------------*/

/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/

/*****************************************************************************
 �� �� ��  : Led_light_int
 ��������  : LED�Ƴ�ʼ�����صƺ���
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��
    ��    ��   : zxf
    �޸�����   : ����

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
 �� �� ��  : apl_init
 ��������  : APL���ʼ������
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��10��
    ��    ��   : sunj
    �޸�����   : ����

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
