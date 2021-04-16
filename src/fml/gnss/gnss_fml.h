/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : gnss_fml.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年6月3日
  最近修改   :
  功能描述   : GNSS接收机处理头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年6月3日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


#ifndef _GNSS_FML_H_
#define _GNSS_FML_H_

#include "pubdef.h"


#define UBLOX_USED	//使用UBLOX接收机

/*GNSS 数据结构*/
typedef struct
{
	U32 receiver_criterion;	//接收机有效判断条件

	U8 ubx_enable;			//UBX输出使能
	U8 mxt_enable;          //MXT输出RTCM使能
	U8 valid;				//接收机是否有效
	U8 lat_ind;				//纬度标志
	U8 lon_ind;				//经度标志
	
//	time_s time;	        //时间信息
	F64 lattitude;		    //纬度
	F64 lontitude;		    //经度
	F32 altitude;		    //高度
	
	F32 vx;				    //x方向上的速度
	F32 vy;				    //y方向上的速度
	F32 vz;				    //x方向上的速度
	
	U8 gps_leap_seconds;	//GPS闰秒数
	U8 view_sat_num;	    //可视卫星数
	U8 use_sat_num;		    //可用卫星数
	U8 db30_sat_num;	    //大于30dB卫星数

	U8 db30_sat_num_gps;	//GPS的大于30dB卫星数
	U8 db30_sat_num_bds;	//BDS的大于30dB卫星数
	U8 db30_sat_num_glo;	//GLO的大于30dB卫星数
	U8 reserve;				//预留
	
	F32 pps_err;		    // 1PPS残差
	F32 pdop;			    //位置精度因子
	F32 hdop;			    //水平精度因子
	F32 vdop;			    //垂直精度因子
	U32 gps_tow;		    //周内秒	
	U32 gps_weeks;		    //GPS周数

	//U8  rtk_mode;           //RTK高精度定位模式
}gnss_data_t;


typedef enum
{
    GNSS_REF_CRITERION_SATELLITES_NUM = 0x0,       /* 卫星数*/
   	GNSS_REF_CRITERION_POSITION_FIX,               /* 授时定位标志 */
   	GNSS_REF_CRITERION_RCV_STATUS,                 /* 接收状态标志 */
	GNSS_REF_CRITERION_CNO_STATUS,                 /* 信噪比大于29 超过4*/
	GNSS_REF_CRITERION_ANTEN_STATUS,               /* 天线状态*/
	GNSS_REF_CRITERION_LEAP_FLAG,                  /* 闰秒标记 */
	GNSS_REF_CRITERION_RCV_DATA,                   /* 串口报文接收*/
	GNSS_REF_CRITERION_SEC_CONTINUES,              /* 秒连续*/

   GNSS_REF_CRITERION_END,
}GNSS_CRITERION_ENUM;

#define  GNSS_CRITERION_VALID     ( (1<< GNSS_REF_CRITERION_POSITION_FIX) |\
										 (1<< GNSS_REF_CRITERION_RCV_DATA))
extern U32 gnss_condition_cnt;
extern void uart_fml_send_gnss_cmd(U8* pbuf, U32 len);
extern void gnss_fml_init(gnss_data_t** p_handle);
#endif
/*eof*/
