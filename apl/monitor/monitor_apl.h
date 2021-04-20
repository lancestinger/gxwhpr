/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : monitor_apl.h
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��5��30��
  ����޸�   :
  ��������   : monitorͷ�ļ�
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��5��30��
    ��    ��   : wzq
    �޸�����   : �����ļ�

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
	U8 dev_sn[8];					/* �豸���к� */
	U8 dev_mac[6];					/* �豸MAC��ַ */
	F32 cpu_useage;				    /*  */
	F32 cpu_temper;
	F32 env_temper;
	U32 state_word1;				/* ״̬��1 */
}monitor_status_t;
extern void monitor_apl_init(void);

#endif
/*eof*/
