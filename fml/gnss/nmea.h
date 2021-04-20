/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : nmea.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年6月3日
  最近修改   :
  功能描述   : NMEA解析头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年6月3日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

#ifndef _NMEA_H_
#define _NMEA_H_

#include "gnss/gnss_fml.h"
#include "apl/imu/ins_baseparams.h"


extern U8 GGA_OLD_FLAG;
extern U8 GGA_DATA_READY;
extern U8 RMC_DATA_READY;


typedef struct
{
	F64 lat;		    //纬度
	F64 lon;		    //经度
	F32 alt;		    //高度
	U8 RTK_mode;		 //RTK模式
	F32 angle;         //航向角
	F64 veloc;         //航速
	
}Parse_data;


extern Parse_data NMEA_Data_ptr;
extern char Sys_Date[20];
extern char Sys_UTC[20];
extern U32 UBX_1PPS_time;
extern U8 Origin_flag;
extern U8 NMEA_OPEN_SWITCH;

U8 gnss_nmea_data_process(IN U8 *data_ptr, IN U32 len);
void gnss_nmea_init(gnss_data_t* p_handle);
#endif
/*eof*/
