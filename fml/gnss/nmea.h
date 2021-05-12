/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : nmea.h
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��6��3��
  ����޸�   :
  ��������   : NMEA����ͷ�ļ�
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��6��3��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/

#ifndef _NMEA_H_
#define _NMEA_H_

#include "gnss/gnss_fml.h"
#include "apl/imu/ins_baseparams.h"


#define MSG_BUFLEN      15

extern U8 GGA_OLD_FLAG;
extern U8 GGA_DATA_READY;
extern U8 RMC_DATA_READY;


typedef struct
{
	F64 lat;		    //γ��
	F64 lon;		    //����
	F32 alt;		    //�߶�
	U8 RTK_mode;		 //RTKģʽ
	F32 angle;         //�����
	F64 veloc;         //����
	char *EW_flag;       //������־
	char *NS_flag;       //�ϱ���־
	U8 GGA_slevel;     //GGA���������ຣƽ�����
	char *GGA_unit;     //GGA��λ
	char *RMC_VA;       //RMC��Ч��־
	
}Parse_data;

typedef enum
{
	GGA=0,
	RMC=1,
	
}Nmea_msg;


extern Parse_data NMEA_Data_ptr;
extern Parse_data NMEA_Rebuild_data;
extern char Sys_Date[20];
extern char Sys_UTC[20];
extern U32 UBX_1PPS_time;
extern U8 Origin_flag;
extern U8 NMEA_OPEN_SWITCH;
extern int FIRST_UWB_GGA;
extern int FIRST_UWB_RMC;

U8 gnss_nmea_data_process(IN U8 *data_ptr, IN U32 len);
extern void NMEA_Rebuild(Nmea_msg mode,U32 len);
void gnss_nmea_init(gnss_data_t* p_handle);
#endif
/*eof*/
