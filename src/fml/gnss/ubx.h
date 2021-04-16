/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : ubx.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年6月3日
  最近修改   :
  功能描述   : UBX解析头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年6月3日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

#ifndef _UBX_H_
#define _UBX_H_

#include "gnss/gnss_fml.h"

//UBLOX 消息ID
typedef enum
{
	UBX_NAV_VELECEF = 0x0111,
	UBX_AID_ALM = 0x0B30,
	UBX_AID_EPH = 0x0B31,
	UBX_NAV_DOP = 0x0104,
	UBX_NAV_PVT = 0x0107,
	UBX_NAV_SAT = 0x0135,
	UBX_NAV_TIMEGPS = 0x0120,
	UBX_RXM_SFRBX = 0x0213,
	UBX_TIM_TP = 0x0D01,
	
	NMEA_DTM = 0xF00A,
	NMEA_GBQ = 0xF044,
	NMEA_GBS = 0xF009,
	NMEA_GGA = 0xF000,
	NMEA_GLL = 0xF001,
	NMEA_GLQ = 0xF043,
	NMEA_GNQ = 0xF042,
	NMEA_GNS = 0xF00D,
	NMEA_GPQ = 0xF040,
	NMEA_GRS = 0xF006,
	NMEA_GSA = 0xF002,
	NMEA_GST = 0xF007,
	NMEA_GSV = 0xF003,
	NMEA_RMC = 0xF004,
	NMEA_TXT = 0xF041,
	NMEA_VLW = 0xF00F,
	NMEA_VTG = 0xF005,
	NMEA_ZDA = 0xF008,

	NMEA_PUBX_POS = 0xF100,
	NMEA_PUBX_SVS = 0xF103,
	NMEA_PUBX_TIME = 0xF104,
}UBLOX_MSG_CLS_ID_Enum;

//GNSS系统切换类型
typedef enum
{
    GNSS_SWITCH_GPS_BDS,
    GNSS_SWITCH_GPS_GLO
}GNSS_SWITCH_TYPE_Enum;

/* 参数属性结构体 */
typedef struct
{
	UBLOX_MSG_CLS_ID_Enum	ubx_msg_id;	
	U8 						(*parse)(U8*, U32);	
}UBX_ParaAttr_t;

extern GNSS_SWITCH_TYPE_Enum g_gnss_switch_type; //added by sunj 2019-10-10 14:51

extern void gnss_ublox_enable_fixmode(void);
extern void gnss_ublox_disable_fixmode(void);
extern void gnss_ublox_cfg(void);
extern void gnss_ublox_enable_gnss(GNSS_SWITCH_TYPE_Enum type);
extern void gnss_ublox_disable_gnss(void);
extern void gnss_ublox_poll_aid_eth(void);
extern void gnss_ublox_save_cfg(void);
extern void gnss_ublox_hot_start(void);
extern void gnss_ublox_warm_start(void);
extern void gnss_ublox_cold_start(void);
extern void gnss_ublox_poll_aid_eth(void);
extern void gnss_ublox_poll_aid_alm(void);
extern void gnss_ubx_init(gnss_data_t* p_handle);
extern void gnss_mxt_init(void);
extern U8 gnss_ublox_ubx_data_process(IN U8 *data_ptr, IN U32 len);
extern F32 gnss_ublox_get_1pps_err(void);
extern void ublox_calc_ubx_checksum(U8* buf, U32 len, U8* ck_a, U8* ck_b);
#endif
/*eof*/
