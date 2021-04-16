/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : gnss_fml.h
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��6��3��
  ����޸�   :
  ��������   : GNSS���ջ�����ͷ�ļ�
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��6��3��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


#ifndef _GNSS_FML_H_
#define _GNSS_FML_H_

#include "pubdef.h"


#define UBLOX_USED	//ʹ��UBLOX���ջ�

/*GNSS ���ݽṹ*/
typedef struct
{
	U32 receiver_criterion;	//���ջ���Ч�ж�����

	U8 ubx_enable;			//UBX���ʹ��
	U8 mxt_enable;          //MXT���RTCMʹ��
	U8 valid;				//���ջ��Ƿ���Ч
	U8 lat_ind;				//γ�ȱ�־
	U8 lon_ind;				//���ȱ�־
	
//	time_s time;	        //ʱ����Ϣ
	F64 lattitude;		    //γ��
	F64 lontitude;		    //����
	F32 altitude;		    //�߶�
	
	F32 vx;				    //x�����ϵ��ٶ�
	F32 vy;				    //y�����ϵ��ٶ�
	F32 vz;				    //x�����ϵ��ٶ�
	
	U8 gps_leap_seconds;	//GPS������
	U8 view_sat_num;	    //����������
	U8 use_sat_num;		    //����������
	U8 db30_sat_num;	    //����30dB������

	U8 db30_sat_num_gps;	//GPS�Ĵ���30dB������
	U8 db30_sat_num_bds;	//BDS�Ĵ���30dB������
	U8 db30_sat_num_glo;	//GLO�Ĵ���30dB������
	U8 reserve;				//Ԥ��
	
	F32 pps_err;		    // 1PPS�в�
	F32 pdop;			    //λ�þ�������
	F32 hdop;			    //ˮƽ��������
	F32 vdop;			    //��ֱ��������
	U32 gps_tow;		    //������	
	U32 gps_weeks;		    //GPS����

	//U8  rtk_mode;           //RTK�߾��ȶ�λģʽ
}gnss_data_t;


typedef enum
{
    GNSS_REF_CRITERION_SATELLITES_NUM = 0x0,       /* ������*/
   	GNSS_REF_CRITERION_POSITION_FIX,               /* ��ʱ��λ��־ */
   	GNSS_REF_CRITERION_RCV_STATUS,                 /* ����״̬��־ */
	GNSS_REF_CRITERION_CNO_STATUS,                 /* ����ȴ���29 ����4*/
	GNSS_REF_CRITERION_ANTEN_STATUS,               /* ����״̬*/
	GNSS_REF_CRITERION_LEAP_FLAG,                  /* ������ */
	GNSS_REF_CRITERION_RCV_DATA,                   /* ���ڱ��Ľ���*/
	GNSS_REF_CRITERION_SEC_CONTINUES,              /* ������*/

   GNSS_REF_CRITERION_END,
}GNSS_CRITERION_ENUM;

#define  GNSS_CRITERION_VALID     ( (1<< GNSS_REF_CRITERION_POSITION_FIX) |\
										 (1<< GNSS_REF_CRITERION_RCV_DATA))
extern U32 gnss_condition_cnt;
extern void uart_fml_send_gnss_cmd(U8* pbuf, U32 len);
extern void gnss_fml_init(gnss_data_t** p_handle);
#endif
/*eof*/
