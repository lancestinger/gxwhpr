/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : ubx.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年6月3日
  最近修改   :
  功能描述   : UBX解析程序
  函数列表   :
  修改历史   :
  1.日    期   : 2019年6月3日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


/*------------------------------头文件------------------------------*/
#include "gnss/ubx.h"
#include "project_def.h"
#include "uart/uart_fml.h"
/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/
#define UBX_PARSE_NUM	15
/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
//禁止所有ubx协议输出
static const U8 ublox_disable_ubx_cmd[] = 
{
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x30, 0x00, 0x45, 0xC0, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x50, 0x00, 0x65, 0x00, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x32, 0x00, 0x47, 0xC4, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x33, 0x00, 0x48, 0xC6, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x31, 0x00, 0x46, 0xC2, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x02, 0x00, 0x17, 0x64, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x01, 0x00, 0x16, 0x62, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x00, 0x00, 0x15, 0x60, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x10, 0x15, 0x00, 0x2F, 0x99, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x10, 0x02, 0x00, 0x1C, 0x73, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x10, 0x02, 0x00, 0x1C, 0x73, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x10, 0x02, 0x00, 0x1C, 0x73, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x10, 0x03, 0x00, 0x1D, 0x75, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x10, 0x10, 0x00, 0x2A, 0x8F, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x28, 0x00, 0x00, 0x32, 0xB7, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x21, 0x0E, 0x00, 0x39, 0xBE, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x21, 0x08, 0x00, 0x33, 0xB2, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x21, 0x0B, 0x00, 0x36, 0xB8, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x21, 0x0F, 0x00, 0x3A, 0xC0, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x21, 0x0D, 0x00, 0x38, 0xBC, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x13, 0x60, 0x00, 0x7D, 0x38, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x13, 0x80, 0x00, 0x9D, 0x78, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x13, 0x80, 0x00, 0x9D, 0x78, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x13, 0x80, 0x00, 0x9D, 0x78, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x13, 0x80, 0x00, 0x9D, 0x78, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x13, 0x80, 0x00, 0x9D, 0x78, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x13, 0x80, 0x00, 0x9D, 0x78, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x13, 0x21, 0x00, 0x3E, 0xBA, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x05, 0x00, 0x19, 0x67, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x09, 0x00, 0x1D, 0x6F, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x0B, 0x00, 0x1F, 0x73, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x37, 0x00, 0x4B, 0xCB, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x02, 0x00, 0x16, 0x61, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x06, 0x00, 0x1A, 0x69, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x38, 0x00, 0x4C, 0xCD, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x07, 0x00, 0x1B, 0x6B, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x21, 0x00, 0x35, 0x9F, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x2E, 0x00, 0x42, 0xB9, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0A, 0x08, 0x00, 0x1C, 0x6D, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x60, 0x00, 0x6B, 0x02, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x05, 0x00, 0x10, 0x4C, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x22, 0x00, 0x2D, 0x86, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x31, 0x00, 0x3C, 0xA4, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x04, 0x00, 0x0F, 0x4A, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x40, 0x00, 0x4B, 0xC2, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x61, 0x00, 0x6C, 0x04, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x39, 0x00, 0x44, 0xB4, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x13, 0x00, 0x1E, 0x68, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x14, 0x00, 0x1F, 0x6A, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x09, 0x00, 0x14, 0x54, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x34, 0x00, 0x3F, 0xAA, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x01, 0x00, 0x0C, 0x44, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x00, 0x0D, 0x46, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x00, 0x12, 0x50, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x3C, 0x00, 0x47, 0xBA, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x35, 0x00, 0x40, 0xAC, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x32, 0x00, 0x3D, 0xA6, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x43, 0x00, 0x4E, 0xC8, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x42, 0x00, 0x4D, 0xC6, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x00, 0x11, 0x4E, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x00, 0x0E, 0x48, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x3B, 0x00, 0x46, 0xB8, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x00, 0x3B, 0xA2, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x24, 0x00, 0x2F, 0x8A, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x25, 0x00, 0x30, 0x8C, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x23, 0x00, 0x2E, 0x88, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x20, 0x00, 0x2B, 0x82, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x26, 0x00, 0x31, 0x8E, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x21, 0x00, 0x2C, 0x84, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x11, 0x00, 0x1C, 0x64, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x00, 0x1D, 0x66, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x30, 0x00, 0x3C, 0xA5, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x31, 0x00, 0x3D, 0xA7, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x61, 0x00, 0x6D, 0x07, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x14, 0x00, 0x20, 0x6D, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x10, 0x00, 0x1C, 0x65, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x15, 0x00, 0x21, 0x6F, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x59, 0x00, 0x65, 0xF7, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x32, 0x00, 0x3E, 0xA9, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x11, 0x00, 0x1D, 0x67, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x13, 0x00, 0x1F, 0x6B, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x20, 0x00, 0x2C, 0x85, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x27, 0x01, 0x00, 0x32, 0xB6, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x27, 0x03, 0x00, 0x34, 0xBA, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x11, 0x00, 0x28, 0x88, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x16, 0x00, 0x2D, 0x92, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x13, 0x00, 0x2A, 0x8C, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x04, 0x00, 0x1B, 0x6E, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x03, 0x00, 0x1A, 0x6C, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x12, 0x00, 0x29, 0x8A, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x01, 0x00, 0x18, 0x68, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x15, 0x00, 0x2C, 0x90, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x06, 0x00, 0x1D, 0x72, 
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x09, 0x14, 0x00, 0x27, 0x82
};


//打开ubx语句输出
static const U8 ublox_enable_ubx_cmd[] = 
{
	//0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0B, 0x31, 0x01, 0x47, 0xC3,	//开启AID-EPH,即GPS星历
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x20, 0x01, 0x2C, 0x83,	//开启GPS时间信息 ，包含闰秒有效信息和时间信息
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x35, 0x01, 0x41, 0xAD,	//开启NAV-SAT信息，包含卫星颗数和载噪比CN0,可用卫星数
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51,	//开启NAV-PVT信息，包含定位状态，坐标信息,PDOP
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x04, 0x01, 0x10, 0x4B,	//开启NAV-DOP信息，包含PDOP,HDOP,VDOP,GDOP,TDOP
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x0D, 0x01, 0x01, 0x19, 0x69,	//开启TIM-TP信息，可以得到1PPS残差进行补偿
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x11, 0x01, 0x1D, 0x65,	//开启NAV_VELECEF,可以得到速度信息
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x02, 0x13, 0x01, 0x20, 0x6C,	//开启导航电文接收
	//0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x21, 0x01, 0x2D, 0x85	//开启UTC时间信息 ，包含闰秒有效信息和时间信息
};

//POLL AID-EPH命令数据，即GPS星历
static const U8 ublox_ubx_poll_aid_eth[] = {0xB5, 0x62, 0x0B, 0x31, 0x00, 0x00, 0x3C, 0xBF}; //POLL AID-EPH

//POLL AID-ALM命令数据，即GPS星历
static const U8 ublox_ubx_poll_aid_alm[] = {0xB5, 0x62, 0x0B, 0x30, 0x00, 0x00, 0x3B, 0xBC}; //POLL AID-ALM

//配置UBLOX串口输出115200波特率，且输出为UBX协议格式
static const U8 ublox_cfg_port_115200_ubx[] = 
{
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x72, 0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22
};

//配置UBLOX串口输出115200波特率，且输出为NMEA协议格式
static const U8 ublox_cfg_port_115200_nmea[] = 
{
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0x78, 0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22
};

static const U8 ublox_save_cfg_cmd[] = 
{
	0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB
};

static const U8 ublox_cfg_fix_mode[] = 
{
	0xB5, 0x62, 0x06, 0x3D, 0x1C, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
 	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
 	0x05, 0x00, 0x00, 0x00, 0xB8, 0x0B, 0x00, 0x00, 0x28, 0xB0, 0xB5, 0x62, 0x06, 
 	0x3D, 0x00, 0x00, 0x43, 0xCF
 	/* // 15分钟 精度小于5米进入固定位置模式
 	0xB5, 0x62, 0x06, 0x3D, 0x1C, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x03, 0x00, 0x00, 0x88, 0x13, 0x00, 0x00, 0x82, 0x15, 0xB5, 0x62, 0x06, 0x3D, 0x00, 0x00, 0x43, 0xCF
	*/
};

static const U8 ublox_cfg_disable_fix_mode[] = 
{
	0xB5, 0x62, 0x06, 0x3D, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x6B, 0xB5, 0x62, 0x06, 
	0x3D, 0x00, 0x00, 0x43, 0xCF
};

//使能GPS+GLONASS
const U8 ublox_cfg_gnss_enable_gps_glo[] = 
{
	//GPS
	/*0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 
	0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 
	0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 
	0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 
	0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 
	0x01, 0x2C, 0x4D, 0xB5, 0x62, 0x06, 0x3E, 0x00, 0x00, 0x44, 0xD2*/

	//GPS+GLONASS
	0xB5,0x62,0x06,0x3E,
    0x34,0x00,
    0x00,0x00,0x1E,0x06,
    0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01,
    0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,
    0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,
    0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01,
    0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01,
    0x06,0x08,0x0E,0x00,0x01,0x00,0x01,0x01,
    0x12,0x65,
    0xB5,0x62,0x06,0x3E,0x00,0x00,0x44,0xD2
};

/*
static const U8 ublox_cfg_gnss_enable_bds_glo[] = 
{
	0xB5, 0x62, 0x06, 0x3E, 0x34, 0x00, 0x00, 0x00, 0x1E, 0x06, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 
	0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 
	0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 
	0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x12, 0x55, 0xB5, 0x62, 0x06, 0x3E, 0x00, 0x00, 0x44, 0xD2
};
*/
//使能GPS+BDS
const U8 ublox_cfg_gnss_enable_gps_bds[] = 
{
	0xB5,0x62,0x06,0x3E, 
    0x34,0x00, 
    0x00,0x00,0x1E,0x06, 
    0x00,0x08,0x10,0x00,0x01,0x00,0x01,0x01, 
    0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,
    0x03,0x08,0x10,0x00,0x01,0x00,0x01,0x01, 
    0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01, 
    0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01, 
    0x06,0x08,0x0E,0x00,0x00,0x00,0x01,0x01, 
    0x12,0x7D, 
    0xB5,0x62,0x06,0x3E,0x00,0x00,0x44,0xD2
};

//使能BDS
const U8 ublox_cfg_gnss_enable_bds[] = 
{
    0xB5,0x62,0x06,0x3E, 
    0x34,0x00, 
    0x00,0x00,0x1E,0x06, 
    0x00,0x08,0x10,0x00,0x00,0x00,0x01,0x01, 
    0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,
    0x03,0x08,0x10,0x00,0x01,0x00,0x01,0x01, 
    0x04,0x00,0x08,0x00,0x00,0x00,0x01,0x01, 
    0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01, 
    0x06,0x08,0x0E,0x00,0x00,0x00,0x01,0x01, 
    0x11,0x51, 
    0xB5,0x62,0x06,0x3E,0x00,0x00,0x44,0xD2
};

//关闭GNSS接收
static const U8 ubloc_cfg_gnss_disable_gnss[] = 
{
	0xB5, 0x62, 0x06, 0x3E, 
    0x3C, 0x00, 
    0x00, 0x00, 0x20, 0x07, 
    0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 
    0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 
    0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 
    0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 
    0x04, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 
    0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 
    0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 
    0x25, 0x93, 
    0xB5, 0x62, 0x06, 0x3E, 0x00, 0x00, 0x44, 0xD2
};

//使能GPS PPS作为时间脉冲
static const U8 ublox_enable_timepulse_gps[] = 
{
	0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 
	0x00, 0x40, 0x42, 0x0F, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7, 0x00, 0x00, 0x00, 0xCA, 
	0xB6, 0xB5, 0x62, 0x06, 0x31, 0x01, 0x00, 0x00, 0x38, 0xE5
};

//热启动
static const U8 ublox_hot_start_cmd[] = 
{
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68 
};

//暖启动
static const U8 ublox_warm_start_cmd[] = 
{
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x11, 0x6C 
};

//冷启动
static const U8 ublox_cold_start_cmd[] = 
{
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x02, 0x00, 0x0E, 0x61 
};

static const UBLOX_MSG_CLS_ID_Enum disable_nmea_msg[] = {
				NMEA_DTM, NMEA_GBQ, NMEA_GBS, NMEA_GGA, NMEA_GLL, NMEA_GLQ, NMEA_GNQ, NMEA_GNS, NMEA_GPQ, 
				NMEA_GRS, NMEA_GSA, NMEA_GST, NMEA_GSV, NMEA_RMC, NMEA_TXT, NMEA_VLW, NMEA_VTG, NMEA_ZDA,
				NMEA_PUBX_POS, NMEA_PUBX_SVS, NMEA_PUBX_TIME};
static const UBLOX_MSG_CLS_ID_Enum enable_nmea_msg[] = {NMEA_GGA, NMEA_GSA, NMEA_GSV, NMEA_RMC, NMEA_ZDA,NMEA_PUBX_TIME};				
static gnss_data_t* p_gnss_handle;
static UBX_ParaAttr_t _m_ubx_parse_array[UBX_PARSE_NUM];

GNSS_SWITCH_TYPE_Enum g_gnss_switch_type = GNSS_SWITCH_GPS_BDS; //added by sunj 2019-10-10 14:51
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/


/*****************************************************************************
 函 数 名  : ublox_calc_ubx_checksum
 功能描述  : ubx校验和计算
 输入参数  : U8* buf   
             U32 len   
             U8* ck_a  
             U8* ck_b  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月5日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void ublox_calc_ubx_checksum(U8* buf, U32 len, U8* ck_a, U8* ck_b)
{
	U32 i = 0, tmp_a = 0, tmp_b = 0;

	for(i = 0; i < len; i++)
	{
		tmp_a += buf[i];
		tmp_b += tmp_a;
	}
	*ck_a = tmp_a & 0xFF;
	*ck_b = tmp_b & 0xFF;
}

/*****************************************************************************
 函 数 名  : ublox_set_open_pubx4
 功能描述  : 使能ublox pubx协议输出
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月5日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void _ublox_set_open_pubx4(void)
{
	U8 sbuf[50];
	U32 index  = 0;

	sbuf[index++] = 0xB5;
	sbuf[index++] = 0x62;
	sbuf[index++] = 0x06;
	sbuf[index++] = 0x01;
	sbuf[index++] = 0x03;
	sbuf[index++] = 0x00;
	sbuf[index++] = 0xF1;
	sbuf[index++] = 0x04;
	sbuf[index++] = 0x01;
	sbuf[index++] = 0x00;
	sbuf[index++] = 0x1B;
	uart_fml_send_gnss_cmd(sbuf, index);
}

/*****************************************************************************
 函 数 名  : _ublox_set_disable_all_nmea
 功能描述  : 禁止NMEA语句输出
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月5日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void _ublox_set_disable_all_nmea(void)
{
	U32 i = 0;
	U8 index = 0;
	U8 sbuf[20];
	U8 ck_a = 0, ck_b = 0;
	
	for(i = 0; i < (sizeof(disable_nmea_msg)/sizeof(UBLOX_MSG_CLS_ID_Enum)); i++)
	{
		index = 0;
		sbuf[index++] = 0xB5;
		sbuf[index++] = 0x62;
		sbuf[index++] = 0x06;
		sbuf[index++] = 0x01;
		sbuf[index++] = 0x03;
		sbuf[index++] = 0x00;
		sbuf[index++] = ((disable_nmea_msg[i] >> 8) & 0xFF);
		sbuf[index++] = (disable_nmea_msg[i] & 0xFF);
		sbuf[index++] = 0x00;
		ublox_calc_ubx_checksum(sbuf+2, index-2, &ck_a, &ck_b);
		sbuf[index++] = ck_a;
		sbuf[index++] = ck_b;
		uart_fml_send_gnss_cmd(sbuf, index);
		delay_ms(10);
	}
}

//使能NMEA协议
void _ublox_set_enable_nmea(void)
{
	
	U32 i = 0;
	U8 index = 0;
	U8 sbuf[20];
	U8 ck_a = 0, ck_b = 0;
	
	for(i = 0; i < (sizeof(enable_nmea_msg)/sizeof(UBLOX_MSG_CLS_ID_Enum)); i++)
	{
		index = 0;
		sbuf[index++] = 0xB5;
		sbuf[index++] = 0x62;
		sbuf[index++] = 0x06;
		sbuf[index++] = 0x01;
		sbuf[index++] = 0x03;
		sbuf[index++] = 0x00;
		sbuf[index++] = ((enable_nmea_msg[i] >> 8) & 0xFF);
		sbuf[index++] = (enable_nmea_msg[i] & 0xFF);
		sbuf[index++] = 0x01;
		ublox_calc_ubx_checksum(sbuf+2, index-2, &ck_a, &ck_b);
		sbuf[index++] = ck_a;
		sbuf[index++] = ck_b;
		uart_fml_send_gnss_cmd(sbuf, index);
		delay_ms(10);
	}
}

//解析UBX NAV-TIMEGPS消息
static U8 _ublox_parse_ubx_timegps(IN U8 *payload_ptr, U32 payload_len)
{

	#define UBX_TIME_GPS_PAY_LEN	16	//payload 长度
	U8 ret = FALSE;
#if 0

	U8 temp8 = 0;
	U32 temp32 = 0;
	U16 tmp_week = 0;
	S32 gps_secs = 0;

	if(UBX_TIME_GPS_PAY_LEN == payload_len)		//长度相等
	{
		/* 取GPS闰秒标记 */
		GLOBAL_MEMCPY(&temp8, payload_ptr+11, sizeof(temp8));
		if(temp8 & 0x04)
		{
			APP_SET_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_LEAP_FLAG);
			GLOBAL_MEMCPY(&temp8, payload_ptr+10, sizeof(temp8));
			p_gnss_handle->gps_leap_seconds = temp8;
			main_handle_g.p_time_handle->gps_leap_sec = temp8;

            if(sys_debug_get_type(SYS_DEBUG_UBX))
            {
                GLOBAL_TIME_PRINT(("闰秒有效时,leapSecond:%us\r\n",payload_ptr[10]));
            }
		}
		else
		{
            if(sys_debug_get_type(SYS_DEBUG_UBX))
            {
                GLOBAL_TIME_PRINT(("闰秒无效时,leapSecond:%us\r\n",payload_ptr[10]));
            }
            
			APP_CLEAR_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_LEAP_FLAG);
		}

		GLOBAL_MEMCPY(&temp32, payload_ptr, sizeof(temp32));	//取TOW时间
		temp32 = temp32*1e-3+0.5;
		GLOBAL_MEMCPY(&tmp_week, payload_ptr+8, sizeof(tmp_week));	//取周数
		gps_secs = temp32+tmp_week*TOTAL_SECONDS_WEEK;
		//gpstime_to_utctime(gps_secs, p_gnss_handle->gps_leap_seconds, &(p_gnss_handle->time));
		ref_cpy_time(REF_GNSS, &(p_gnss_handle->time));
		ret = TRUE;
	}
#endif
	return ret;
}

//解析UBX NAV-PVT消息,包含坐标信息和定位状态
static U8 _ublox_parse_ubx_pvt(IN U8 *payload_ptr, U32 payload_len)
{
	#define UBX_PVT_PAY_LEN	92	//payload 长度
	U8 ret = FALSE;
#if 0

	S8 temp8 = 0;
	//S16 temp16 = 0;
	S32 temp32 = 0;
	static U8 pre_seconds = 0;
	U8 cur_seconds = 0;

	if(UBX_PVT_PAY_LEN == payload_len)		//长度相等
	{
		/* 取定位标记 */
		GLOBAL_MEMCPY(&temp8, payload_ptr+21, sizeof(temp8));
		if(temp8 & 0x01)
		{
			APP_SET_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_POSITION_FIX);
		}
		else
		{
			APP_CLEAR_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_POSITION_FIX);
		}
		GLOBAL_MEMCPY(&temp8, payload_ptr+11, sizeof(temp8));	//取时间有效标记
		if(((U8)temp8 & 0x3) == 0x3)	//时间有效， 日期有效
		{
			/*GLOBAL_MEMCPY(&temp16, payload_ptr+4, sizeof(temp16));
			mult_handle_g.ref_handle.ref_time[REF_GNSS].year = temp16;
			GLOBAL_MEMCPY(&temp8, payload_ptr+6, sizeof(temp8));
			mult_handle_g.ref_handle.ref_time[REF_GNSS].month = temp8;
			GLOBAL_MEMCPY(&temp8, payload_ptr+7, sizeof(temp8));
			mult_handle_g.ref_handle.ref_time[REF_GNSS].mday = temp8;
			GLOBAL_MEMCPY(&temp8, payload_ptr+8, sizeof(temp8));
			mult_handle_g.ref_handle.ref_time[REF_GNSS].hour = temp8;
			GLOBAL_MEMCPY(&temp8, payload_ptr+9, sizeof(temp8));
			mult_handle_g.ref_handle.ref_time[REF_GNSS].min = temp8;
			GLOBAL_MEMCPY(&temp8, payload_ptr+10, sizeof(temp8));
			mult_handle_g.ref_handle.ref_time[REF_GNSS].sec = temp8;
			*/
			GLOBAL_MEMCPY(&cur_seconds, payload_ptr+10, sizeof(cur_seconds));
		}
		GLOBAL_MEMCPY(&temp8, payload_ptr+23, sizeof(temp8));	//用于定位的卫星数
		p_gnss_handle->use_sat_num = temp8;

		/*位置信息*/
		GLOBAL_MEMCPY(&temp32, payload_ptr+24, sizeof(temp32));	
		p_gnss_handle->lontitude = temp32*1e-7;
		GLOBAL_MEMCPY(&temp32, payload_ptr+28, sizeof(temp32));	
		p_gnss_handle->lattitude = temp32*1e-7;
		GLOBAL_MEMCPY(&temp32, payload_ptr+36, sizeof(temp32));	
		p_gnss_handle->altitude = temp32*1e-3;
		if(p_gnss_handle->lontitude > 0)
		{
			p_gnss_handle->lon_ind = 'E';
		}
		else
		{
			p_gnss_handle->lon_ind = 'W';
		}
		if(p_gnss_handle->lattitude > 0)
		{
			p_gnss_handle->lat_ind = 'N';
		}
		else
		{
			p_gnss_handle->lat_ind = 'S';
		}

		if(pre_seconds != cur_seconds)
		{
			APP_SET_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_SEC_CONTINUES);
		}
		else
		{
			APP_CLEAR_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_SEC_CONTINUES);
		}
		pre_seconds = cur_seconds;

		U8 ref_valid_flag = 0;

	    if((p_gnss_handle->receiver_criterion & GNSS_CRITERION_VALID) == GNSS_CRITERION_VALID)
	    {
	    	ref_valid_flag = TRUE;
	    }
	    else
	    {
	    	ref_valid_flag = FALSE;
	    }

		/* Check machine status */
		main_handle_g.p_ref_handle->ref_status[REF_GNSS] = ref_check_valid(REF_GNSS, &gnss_condition_cnt, ref_valid_flag);
		//GLOBAL_INTVAL(ref_valid_flag);
		/*mult_handle_g.ref_handle.ref_status[REF_GNSS] = sys_ref_check_valid(REF_GNSS,
										&ublox_condition_cnt, ref_valid_flag);
		*/
		//mult_handle_g.ref_handle.ref_status[REF_GNSS] = (ref_valid_flag == TRUE) ? EXT_REF_STATUS_VALID : EXT_REF_STATUS_INVALID;
		ret = TRUE;
	}
#endif
	return ret;
}

//解析UBX NAV-DOP消息
static U8 _ublox_parse_ubx_dop(IN U8 *payload_ptr, U32 payload_len)
{
	#define UBX_DOP_PAY_LEN	18	//payload 长度
	U8 ret = FALSE;
	U16 temp16 = 0;

	if(UBX_DOP_PAY_LEN == payload_len)		//长度相等
	{
		/* 取PDOP */
		GLOBAL_MEMCPY(&temp16, payload_ptr+6, sizeof(temp16));
		p_gnss_handle->pdop = temp16 * 0.01;
		GLOBAL_MEMCPY(&temp16, payload_ptr+10, sizeof(temp16));
		p_gnss_handle->vdop = temp16 * 0.01;
		GLOBAL_MEMCPY(&temp16, payload_ptr+12, sizeof(temp16));
		p_gnss_handle->hdop = temp16 * 0.01;
		ret = TRUE;
	}
	return ret;
}

//解析UBX NAV-VELECEF消息
static U8 _ublox_parse_ubx_velecef(IN U8 *payload_ptr, U32 payload_len)
{
	#define UBX_VELECEF_PAY_LEN	20	//payload 长度
	U8 ret = FALSE;
	S32 temp32 = 0;

	if(UBX_VELECEF_PAY_LEN == payload_len)		//长度相等
	{
		/* 取ecef速度 */
		GLOBAL_MEMCPY(&temp32, payload_ptr+4, sizeof(temp32));
		p_gnss_handle->vx = temp32 * 0.01;
		GLOBAL_MEMCPY(&temp32, payload_ptr+8, sizeof(temp32));
		p_gnss_handle->vy = temp32 * 0.01;
		GLOBAL_MEMCPY(&temp32, payload_ptr+12, sizeof(temp32));
		p_gnss_handle->vz = temp32 * 0.01;
		ret = TRUE;
	}
	return ret;
}

//解析UBX NAV-SAT消息
static U8 _ublox_parse_ubx_sat(IN U8 *payload_ptr, U32 payload_len)
{
	//#define UBX_SAT_PAY_LEN	18	//payload 长度
	U8 ret = TRUE;
#if 0
	U8 temp8 = 0;
	U8 svid = 0;
	U32 temp32 = 0;
	U8 sat_num = 0;
	U8 i = 0;
	U8 db30_num = 0, db30_num_gps = 0, db30_num_bds = 0, db30_num_glo = 0;
	U8 used_num = 0;
	U8 gnss_type = 0;
	S16 residual = 0;
	U8	cn0 = 0;
	S8 El = 0;


	for(i = 0; i < GPS_MAX_NUM; i++)
	{
		gps_eph_org_g[i].use = 0;
        ublox_bds_eph_org_g[i].use = 0;
		ublox_glo_eph_org_g[i].use = 0;
	}
	GLOBAL_MEMCPY(&sat_num, payload_ptr+5, sizeof(sat_num));
	if(sat_num > 0)		
	{
		for(i = 0; i < sat_num; i++)
		{
			gnss_type = payload_ptr[8+12*i];
			GLOBAL_MEMCPY(&svid, payload_ptr+9+12*i, sizeof(svid));
			GLOBAL_MEMCPY(&temp8, payload_ptr+10+12*i, sizeof(temp8));
			if(temp8 >= 30)
			{
				db30_num++;

				if(gnss_type == 0)
				{
					db30_num_gps++;
				}
				else if(gnss_type == 3)
				{
					db30_num_bds++;
				}
				else if(gnss_type == 6)
				{
					db30_num_glo++;
				} 
			}
			GLOBAL_MEMCPY(&temp32, payload_ptr+16+12*i, sizeof(temp32));
			if(temp32 & 0x8)	//被用于定位
			{
				if(gnss_type == 0)
				{
					gps_eph_org_g[svid].use = 1;
				}
				else if(gnss_type == 6)
				{
					ublox_glo_eph_org_g[svid].use = 1;
				}
				used_num++;
			}
			if(gnss_type == 0)
			{
				gps_eph_org_g[svid].cn0 = temp8;
			}
			GLOBAL_MEMCPY(&temp8, payload_ptr+11+12*i, sizeof(temp8));
			gps_eph_org_g[svid].elev = (S8)temp8;

			GLOBAL_MEMCPY(&residual, payload_ptr+14+12*i, sizeof(residual));
			GLOBAL_MEMCPY(&cn0, payload_ptr+10+12*i, sizeof(cn0));
			GLOBAL_MEMCPY(&El, payload_ptr+11+12*i, sizeof(El));
			if(gnss_type == 0)	//GPS
			{
				DBG_UBX_SAT_Print("SV:G%u\tCN0:%u\tResidual:%f\tNav:%u\tEl:%d\tEPH:%u\r\n", svid,cn0,residual*0.1,
					(temp32&0x8)?1:0,El,(temp32 & 0x800)?1:0);
			}
			else if(gnss_type == 3)	//BDS
			{
				DBG_UBX_SAT_Print("SV:B%d\tCN0:%u\tResidual:%f\tNav:%u\tEl:%d\tEPH:%u\r\n",svid,cn0,residual*0.1,
					(temp32 & 0x8)?1:0,El,(temp32 & 0x800)?1:0);
			}	
			else if(gnss_type == 6)	//GLO
			{
				DBG_UBX_SAT_Print("SV:R%d\tCN0:%u\tResidual:%f\tNav:%u\tEl:%d\tEPH:%u\r\n",svid,cn0,residual*0.1,
					(temp32 & 0x8)?1:0,El,(temp32 & 0x800)?1:0);
			}	
		}
	}
	
	for(i = 0; i < GPS_MAX_NUM; i++)
	{
//		gps_eph_org_g[i].eph.sat_num = (used_num > PL_SINGLE_CHN_MAX_NUM) ? PL_SINGLE_CHN_MAX_NUM : used_num;
	}
	
	if(db30_num>3)
	{
        APP_SET_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_CNO_STATUS);
    }
    else
    {
        APP_CLEAR_BIT(p_gnss_handle->receiver_criterion, GNSS_REF_CRITERION_CNO_STATUS);
    }
	p_gnss_handle->db30_sat_num = db30_num;
	p_gnss_handle->db30_sat_num_gps = db30_num_gps;
	p_gnss_handle->db30_sat_num_bds = db30_num_bds;
	p_gnss_handle->db30_sat_num_glo = db30_num_glo;
#endif
	return ret;
}

//解析TIM-TP报文
static U8 _ublox_parse_ubx_tim_tp(IN U8 *payload_ptr, U32 payload_len)
{
	#define UBX_TP_PAY_LEN	16	//payload 长度
	U8 ret = FALSE;
	S32 temp32 = 0;

	p_gnss_handle->pps_err = 0;
	if(UBX_TP_PAY_LEN == payload_len)		//长度相等
	{
		/* 取1PPS残差 */
		GLOBAL_MEMCPY(&temp32, payload_ptr+8, sizeof(temp32));
		p_gnss_handle->pps_err = temp32 * 0.001;
		ret = TRUE;
	}
	return ret;
}

//解析AID-EPH报文
static U8 _ublox_parse_ubx_aid_eph(IN U8 *payload_ptr, U32 payload_len)
{
	#define UBX_AID_EPH_PAY_LEN	104	//payload 长度
	U8 ret = FALSE;
#if 0
	U8 svid = 0, index = 0;;
	U32 temp32 = 0;

	//p_gnss_handle->pps_err = 0;
	if(UBX_AID_EPH_PAY_LEN == payload_len)		//长度相等
	{
		
		/* 解析第一子帧数据 */
		index = 8;
		GLOBAL_MEMCPY(&temp32, payload_ptr+0, sizeof(temp32));	//SVID
		svid = temp32 % GPS_MAX_NUM;
		gps_eph_org_g[svid].use = 1;
		gps_eph_org_g[svid].eph.sat_type = 0;
		gps_eph_org_g[svid].eph.prn = (U16)svid;
		gps_eph_org_g[svid].eph.week = (U16)((payload_ptr[index+2]<<2) + (payload_ptr[index+1] >> 6));
		gps_eph_org_g[svid].eph.iodc = (U16)((payload_ptr[index+0]&0x3)<<8) | (payload_ptr[index+22]);
		gps_eph_org_g[svid].eph.tgd = (S8)(payload_ptr[index+16]);
		gps_eph_org_g[svid].eph.toc = (U32)(payload_ptr[index+20] + (payload_ptr[index+21] << 8));
		gps_eph_org_g[svid].eph.af2 = (S16)(payload_ptr[index+26]);
		gps_eph_org_g[svid].eph.af1 = (S16)(payload_ptr[index+24] + (payload_ptr[index+25] << 8));
		gps_eph_org_g[svid].eph.af0 = ((S32)(((payload_ptr[index+28]>>2) << 10) + (payload_ptr[index+29] << 16) + ((payload_ptr[index+30]) << 24))) >> 10;

		/* 解析第二子帧数据 */
		index += 32;
		gps_eph_org_g[svid].eph.iode = (U16)payload_ptr[index+2];
		gps_eph_org_g[svid].eph.crs = (S16)(payload_ptr[index] + (payload_ptr[index+1]<<8));
		gps_eph_org_g[svid].eph.delta_n = (S16)(payload_ptr[index+5] + (payload_ptr[index+6]<<8));
		gps_eph_org_g[svid].eph.m0 = (S32)((payload_ptr[index+4] << 24) + payload_ptr[index+8] + 
										(payload_ptr[index+9] << 8) + (payload_ptr[index+10] << 16));
		gps_eph_org_g[svid].eph.cuc = (S16)(payload_ptr[index+13] + (payload_ptr[index+14]<<8));
		gps_eph_org_g[svid].eph.e = (U32)((payload_ptr[index+12] << 24) + payload_ptr[index+16] + 
										(payload_ptr[index+17] << 8) + (payload_ptr[index+18] << 16));
		gps_eph_org_g[svid].eph.cus = (S16)(payload_ptr[index+21] + (payload_ptr[index+22]<<8));
		gps_eph_org_g[svid].eph.sqrta = (U32)((payload_ptr[index+20] << 24) + payload_ptr[index+24] + 
										(payload_ptr[index+25] << 8) + (payload_ptr[index+26] << 16));
		gps_eph_org_g[svid].eph.toe = (U32)(payload_ptr[index+29] + (payload_ptr[index+30]<<8));
		gps_eph_org_g[svid].eph.aodo = (S16)((payload_ptr[index+28]>>2)&0x1F);

		/* 解析第三子帧数据 */
		index += 32;
		gps_eph_org_g[svid].eph.cic = (S16)(payload_ptr[index+1] + (payload_ptr[index+2]<<8));
		gps_eph_org_g[svid].eph.omega0 = (S32)((payload_ptr[index+0] << 24) + payload_ptr[index+4] + 
										(payload_ptr[index+5] << 8) + (payload_ptr[index+6] << 16));
		gps_eph_org_g[svid].eph.cis = (S16)(payload_ptr[index+9] + (payload_ptr[index+10]<<8));
		gps_eph_org_g[svid].eph.i0 = (S32)((payload_ptr[index+8] << 24) + payload_ptr[index+12] + 
										(payload_ptr[index+13] << 8) + (payload_ptr[index+14] << 16));
		gps_eph_org_g[svid].eph.crc = (S16)(payload_ptr[index+17] + (payload_ptr[index+18]<<8));
		gps_eph_org_g[svid].eph.w = (S32)((payload_ptr[index+16] << 24) + payload_ptr[index+20] + 
										(payload_ptr[index+21] << 8) + (payload_ptr[index+22] << 16));
		gps_eph_org_g[svid].eph.omegad = ((S32)((payload_ptr[index+24] + (payload_ptr[index+25] << 8) + (payload_ptr[index+26] << 16)) << 8)) >> 8;
		gps_eph_org_g[svid].eph.iode = (U16)payload_ptr[30];
		gps_eph_org_g[svid].eph.idot = ((S16)(((payload_ptr[index+28]>>2) << 2) + (payload_ptr[index+29]<<8)) >> 2);

		ublox_gps_eph_org_g[svid].gnss_num = gps_eph_org_g[svid].eph.sat_num;
		ublox_gps_eph_org_g[svid].gnss_type= 0;
		ublox_gps_eph_org_g[svid].prn = gps_eph_org_g[svid].eph.prn;
		ublox_gps_eph_org_g[svid].week = gps_eph_org_g[svid].eph.week;
		ublox_gps_eph_org_g[svid].iodc = gps_eph_org_g[svid].eph.iodc;
		ublox_gps_eph_org_g[svid].toc = gps_eph_org_g[svid].eph.toc*16;
		ublox_gps_eph_org_g[svid].af2 = gps_eph_org_g[svid].eph.af2*pow(2.0, -55);
		ublox_gps_eph_org_g[svid].af1 = gps_eph_org_g[svid].eph.af1*pow(2.0, -43);
		ublox_gps_eph_org_g[svid].af0 = gps_eph_org_g[svid].eph.af0*pow(2.0, -31);
		ublox_gps_eph_org_g[svid].deltn = gps_eph_org_g[svid].eph.delta_n*pow(2.0, -43)*PI;
		ublox_gps_eph_org_g[svid].m0 = gps_eph_org_g[svid].eph.m0*pow(2.0, -31)*PI;
		ublox_gps_eph_org_g[svid].crs = gps_eph_org_g[svid].eph.crs*pow(2.0, -5);
		ublox_gps_eph_org_g[svid].cuc = gps_eph_org_g[svid].eph.cuc*pow(2.0, -29);
		ublox_gps_eph_org_g[svid].cus = gps_eph_org_g[svid].eph.cus*pow(2.0, -29);
		ublox_gps_eph_org_g[svid].cic = gps_eph_org_g[svid].eph.cic*(pow(2.0, -29));
		ublox_gps_eph_org_g[svid].cis = gps_eph_org_g[svid].eph.cis*pow(2.0, -29);
		ublox_gps_eph_org_g[svid].crc = gps_eph_org_g[svid].eph.crc*pow(2.0, -5);
		ublox_gps_eph_org_g[svid].e = gps_eph_org_g[svid].eph.e*pow(2.0, -33);
		ublox_gps_eph_org_g[svid].sqrta = gps_eph_org_g[svid].eph.sqrta*pow(2.0, -19);
		ublox_gps_eph_org_g[svid].toe = gps_eph_org_g[svid].eph.toe*16;
		ublox_gps_eph_org_g[svid].omega0 = gps_eph_org_g[svid].eph.omega0*pow(2.0, -31)*PI;
		ublox_gps_eph_org_g[svid].i0 = gps_eph_org_g[svid].eph.i0*pow(2.0, -31)*PI;
		ublox_gps_eph_org_g[svid].w = gps_eph_org_g[svid].eph.w*pow(2.0, -31)*PI;
		ublox_gps_eph_org_g[svid].omegad = gps_eph_org_g[svid].eph.omegad*pow(2.0, -43)*PI;
		ublox_gps_eph_org_g[svid].tgd = gps_eph_org_g[svid].eph.tgd*pow(2.0, -31);
		ublox_gps_eph_org_g[svid].iode = gps_eph_org_g[svid].eph.iode;
		ublox_gps_eph_org_g[svid].aodo = gps_eph_org_g[svid].eph.aodo;
		ublox_gps_eph_org_g[svid].idot = gps_eph_org_g[svid].eph.idot*pow(2.0, -43)*PI;

        DBG_EPH_GPS_Print("ublox_gps_eph_org_g svid=%u, toe=%u\r\n",svid, ublox_gps_eph_org_g[svid].toe);
		ret = TRUE;
	}
#endif	
	return ret;
}

//解析AID-ALM报文
static U8 _ublox_parse_ubx_aid_alm(IN U8 *payload_ptr, U32 payload_len)
{
	#define UBX_AID_ALM_PAY_LEN	40	//payload 长度
	U8 ret = FALSE;
#if 0

	U8 svid = 0, index = 0;;
	S32 temp32 = 0;
	S16 temp16 = 0;

	//p_gnss_handle->pps_err = 0;
	if(UBX_AID_ALM_PAY_LEN == payload_len)		//长度相等
	{
		/* 解析第一子帧数据 */
		index = 8;
		GLOBAL_MEMCPY(&temp32, payload_ptr+0, sizeof(temp32));	//SVID
		svid = temp32 % GPS_MAX_NUM;
		gps_alm_data_g[svid].alm.prn= svid;
		gps_alm_data_g[svid].cur_valid = 1;
		GLOBAL_MEMCPY(&temp32, payload_ptr+4, sizeof(temp32));	//week
		gps_alm_data_g[svid].alm.week = temp32%1024;
		gps_alm_data_g[svid].alm.e = ((U16)(payload_ptr[index] + (payload_ptr[index+1]<<8)))*pow(2.0,-21);
		index += 4;
		gps_alm_data_g[svid].alm.toa = ((U32)payload_ptr[index+2])*4096;
		gps_alm_data_g[svid].alm.deltI = ((S16)(payload_ptr[index] + (payload_ptr[index+1]<<8)))*pow(2.0,-19)*PI + 0.3*PI;
		index += 4;
		gps_alm_data_g[svid].alm.omegad = ((S16)(payload_ptr[index+1] + (payload_ptr[index+2]<<8)))*pow(2.0,-38)*PI;
		index += 4;
		gps_alm_data_g[svid].alm.sqrta = (*((U32*)(payload_ptr+index)) & 0xFFFFFF)*pow(2.0,-11);
		index += 4;
		GLOBAL_MEMCPY(&temp32, payload_ptr+index, sizeof(temp32));
		temp32 = temp32 & 0xFFFFFF;
		if(temp32 & 0x800000)
		{
			temp32 |= 0xFF000000;
		}
		gps_alm_data_g[svid].alm.omega0 = temp32*pow(2.0,-23)*PI;
		index += 4;
		GLOBAL_MEMCPY(&temp32, payload_ptr+index, sizeof(temp32));
		temp32 = temp32 & 0xFFFFFF;
		if(temp32 & 0x800000)
		{
			temp32 |= 0xFF000000;
		}
		gps_alm_data_g[svid].alm.w = temp32*pow(2.0,-23)*PI;
		index += 4;
		GLOBAL_MEMCPY(&temp32, payload_ptr+index, sizeof(temp32));
		temp32 = temp32 & 0xFFFFFF;
		if(temp32 & 0x800000)
		{
			temp32 |= 0xFF000000;
		}
		gps_alm_data_g[svid].alm.m0 = temp32 *pow(2.0,-23)*PI;
		index += 4;
		GLOBAL_MEMCPY(&temp32, payload_ptr+index, sizeof(temp32));
		temp32 = temp32 & 0xFFFFFF;
		temp16 = (((temp32 & 0x00FF0000) >> 16) << 3) | ((temp32 >> 2) & 0x7);
		if(temp16 & 0x400)
		{
			temp16 |= (0xF800);
		}
		gps_alm_data_g[svid].alm.af0 = temp16 * pow(2.0, -20);
		temp16 = ((temp32>>5) & 0x7FF);
		if(temp16 & 0x400)
		{
			temp16 |= (0xF800);
		}
		gps_alm_data_g[svid].alm.af1 = temp16 * pow(2.0, -38);
		/*GLOBAL_UINTVAL(svid);
		GLOBAL_UINTVAL(gps_alm_data_g[svid].alm.week);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.e);
		GLOBAL_UINTVAL(gps_alm_data_g[svid].alm.toa);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.deltI);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.omegad);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.sqrta);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.omega0);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.w);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.m0);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.af0);
		GLOBAL_DBFLTVAL(gps_alm_data_g[svid].alm.af1);
		*/
		#if 0
		gps_eph_org_g[svid].eph.tgd = (S8)(payload_ptr[index+16]);
		gps_eph_org_g[svid].eph.toc = (U32)(payload_ptr[index+20] + (payload_ptr[index+21] << 8));
		gps_eph_org_g[svid].eph.af2 = (S16)(payload_ptr[index+26]);
		gps_eph_org_g[svid].eph.af1 = (S16)(payload_ptr[index+24] + (payload_ptr[index+25] << 8));
		gps_eph_org_g[svid].eph.af0 = ((S32)(((payload_ptr[index+28]>>2) << 10) + (payload_ptr[index+29] << 16) + ((payload_ptr[index+30]) << 24))) >> 10;

		/* 解析第二子帧数据 */
		index += 32;
		gps_eph_org_g[svid].eph.iode = (U16)payload_ptr[index+2];
		gps_eph_org_g[svid].eph.crs = (S16)(payload_ptr[index] + (payload_ptr[index+1]<<8));
		gps_eph_org_g[svid].eph.delta_n = (S16)(payload_ptr[index+5] + (payload_ptr[index+6]<<8));
		gps_eph_org_g[svid].eph.m0 = (S32)((payload_ptr[index+4] << 24) + payload_ptr[index+8] + 
										(payload_ptr[index+9] << 8) + (payload_ptr[index+10] << 16));
		gps_eph_org_g[svid].eph.cuc = (S16)(payload_ptr[index+13] + (payload_ptr[index+14]<<8));
		gps_eph_org_g[svid].eph.e = (U32)((payload_ptr[index+12] << 24) + payload_ptr[index+16] + 
										(payload_ptr[index+17] << 8) + (payload_ptr[index+18] << 16));
		gps_eph_org_g[svid].eph.cus = (S16)(payload_ptr[index+21] + (payload_ptr[index+22]<<8));
		gps_eph_org_g[svid].eph.sqrta = (U32)((payload_ptr[index+20] << 24) + payload_ptr[index+24] + 
										(payload_ptr[index+25] << 8) + (payload_ptr[index+26] << 16));
		gps_eph_org_g[svid].eph.toe = (U32)(payload_ptr[index+29] + (payload_ptr[index+30]<<8));
		gps_eph_org_g[svid].eph.aodo = (S16)((payload_ptr[index+28]>>2)&0x1F);

		/* 解析第三子帧数据 */
		index += 32;
		gps_eph_org_g[svid].eph.cic = (S16)(payload_ptr[index+1] + (payload_ptr[index+2]<<8));
		gps_eph_org_g[svid].eph.omega0 = (S32)((payload_ptr[index+0] << 24) + payload_ptr[index+4] + 
										(payload_ptr[index+5] << 8) + (payload_ptr[index+6] << 16));
		gps_eph_org_g[svid].eph.cis = (S16)(payload_ptr[index+9] + (payload_ptr[index+10]<<8));
		gps_eph_org_g[svid].eph.i0 = (S32)((payload_ptr[index+8] << 24) + payload_ptr[index+12] + 
										(payload_ptr[index+13] << 8) + (payload_ptr[index+14] << 16));
		gps_eph_org_g[svid].eph.crc = (S16)(payload_ptr[index+17] + (payload_ptr[index+18]<<8));
		gps_eph_org_g[svid].eph.w = (S32)((payload_ptr[index+16] << 24) + payload_ptr[index+20] + 
										(payload_ptr[index+21] << 8) + (payload_ptr[index+22] << 16));
		gps_eph_org_g[svid].eph.omegad = ((S32)((payload_ptr[index+24] + (payload_ptr[index+25] << 8) + (payload_ptr[index+26] << 16)) << 8)) >> 8;
		gps_eph_org_g[svid].eph.iode = (U16)payload_ptr[30];
		gps_eph_org_g[svid].eph.idot = ((S16)(((payload_ptr[index+28]>>2) << 2) + (payload_ptr[index+29]<<8)) >> 2);
		#endif
		ret = TRUE;
	}
#endif
	return ret;
}

//UBLOX接收机GLONASS导航电文分解各个bit
static void _ubx_glo_nav_bit_fill(IN U32* int_data, OUT U8* bit_data, IN U8 num)
{
	U32 i = 0, j = 0;
	U32 temp32 = 0x80000000;
	U8 valid_bit = 32;

	if(num == 10)
	{
		valid_bit = 22;
		temp32 = (1 << 29);
	}

	for(i = 0; i < num; i++)
	{
		for(j = 0; j < valid_bit; j++)
		{
			bit_data[i*valid_bit+j] = (int_data[i] & (temp32 >> j)) ? 1 : 0;
		}
	}
}

//获取指定bit对应的数据
static U32 _ubx_glo_get_nbit(U8* bit_array, U8 bit_start, U8 bit_num)
{
	U32 ret = 0;
	U8 i = 0;

	if(bit_num < 1)
	{
		ERR_PRINT(("bit数错误!!!!!\r\n"));
		return 0;
	}

	for(i = 0; i < bit_num; i++)
	{
		if(bit_array[bit_start+i])
		{
			ret |= (0x01 << (bit_num-1-i));
		}
		
	}
	return ret;
}

//获取GPS指定bit对应的数据
//sign_flag 0--正数， 1---负数
static S32 _ubx_gps_get_nbit(U8* bit_array, U8 bit_start, U8 bit_num, U8 sign_flag)
{
	S32 ret = 0;
	U8 i = 0;

	if(bit_num < 1)
	{
		ERR_PRINT(("bit数错误!!!!!\r\n"));
		return 0;
	}

	for(i = 0; i < bit_num; i++)
	{
		if(bit_array[bit_start+i])
		{
			ret |= (0x01 << (bit_num-1-i));
		}
		
	}
	if(sign_flag && (bit_num < 32))	//负数处理
	{
		if(ret & (0x1 << (bit_num-1)))	//符号为负数
		{
			for(i = 0; i < (32-bit_num); i++)
			{
				ret |= (0x01 << (31-i));
			}
		}
	}
	return ret;
}



//解析导航电文
static U8 _ublox_parse_ubx_rxm_sfrbx(IN U8 *payload_ptr, U32 payload_len)
{
	//#define UBX_AID_EPH_PAY_LEN	104	//payload 长度
	#define UBX_SFRBX_GLONASS_LEN	24
	#define UBX_SFRBX_BDS_LEN	48
	U8 ret = FALSE;
#if 0
	U8 svid = 0, sign_bit = 0;;
	U32 temp_buf[10] = {0};
	U8 bit_tmp[320];
	U32 temp32 = 0;
    //	U16 temp16 = 0;
	U8 str_index = 0, frame_index = 0;
	
	//p_gnss_handle->pps_err = 0;
	// GLOBAL_PRINT(("rxm_sfrbx rcved!!!!\r\n"));
    //GLOBAL_TIME_PRINT(("rxm_sfrbx rcved!!!!\r\n"));
	if((payload_ptr[0] == 6) && (payload_len == UBX_SFRBX_GLONASS_LEN))	//GLONASS电文解析
	{
		svid = payload_ptr[1];
        
        if(sys_debug_get_type(SYS_DEBUG_UBX))
        {
           GLOBAL_TIME_PRINT(("rxm_sfrbx rcved!!!!,GLONASS SVID:%u\r\n",svid)); 
        }
        
		if(svid > 24) return ret;
		ublox_glo_eph_org_g[svid].glo_eph.prn = svid;
		ublox_glo_eph_org_g[svid].cur_valid = ((ublox_glo_eph_org_g[svid].nav_rcv_flag & 0xF) == 0xF) ? 1 : 0;
		GLOBAL_MEMCPY(temp_buf, payload_ptr+8, 12);
		GLOBAL_MEMSET(bit_tmp, 0x0, sizeof(bit_tmp));
		_ubx_glo_nav_bit_fill(temp_buf, bit_tmp, 3);
		str_index = _ubx_glo_get_nbit(bit_tmp, 1, 4);	//取串号
		/*GLOBAL_PRINT(("导航电文(0x%08X, 0x%08X, 0x%08X), 串号:%u[%u %u %u %u]\r\n", 
									temp_buf[0],temp_buf[1],temp_buf[2],str_index,
									bit_tmp[1],bit_tmp[2],bit_tmp[3],bit_tmp[4]));*/
		//GLOBAL_PRINT(("glonass prn:%u\r\n", svid));
		switch(str_index)
		{
			case 1:
			{
				ublox_glo_eph_org_g[svid].nav_rcv_flag |= (0x1 << (str_index-1));
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 21, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 22, 23);	//取x'
				ublox_glo_eph_org_g[svid].glo_eph.xd = temp32*pow(2.0,-20)*(sign_bit ? (-1):1)*1e3;
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 45, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 46, 4);	//取x''
				ublox_glo_eph_org_g[svid].glo_eph.xdd = temp32*pow(2.0,-30)*(sign_bit ? (-1):1)*1e3;
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 50, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 51, 26);	//取x
				ublox_glo_eph_org_g[svid].glo_eph.x = temp32*pow(2.0,-11)*(sign_bit ? (-1):1)*1e3;
				break;
			}

			case 2:
			{
				ublox_glo_eph_org_g[svid].nav_rcv_flag |= (0x1 << (str_index-1));
				temp32 = _ubx_glo_get_nbit(bit_tmp, 9, 7);	//取tb
				ublox_glo_eph_org_g[svid].glo_eph.tb = temp32*900;
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 21, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 22, 23);	//取y'
				ublox_glo_eph_org_g[svid].glo_eph.yd = temp32*pow(2.0,-20)*(sign_bit ? (-1):1)*1e3;
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 45, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 46, 4);	//取y''
				ublox_glo_eph_org_g[svid].glo_eph.ydd = temp32*pow(2.0,-30)*(sign_bit ? (-1):1)*1e3;
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 50, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 51, 26);	//取y
				ublox_glo_eph_org_g[svid].glo_eph.y = temp32*pow(2.0,-11)*(sign_bit ? (-1):1)*1e3;

				if(ublox_glo_eph_org_g[svid].glo_eph.tb != ublox_glo_eph_org_g[svid].pre_tb)	//星历更新需要清零状态
				{
					ublox_glo_eph_org_g[svid].cur_valid = 0;
					ublox_glo_eph_org_g[svid].refresh_flag = 1;
					ublox_glo_eph_org_g[svid].pre_tb = ublox_glo_eph_org_g[svid].glo_eph.tb;
					ublox_glo_eph_org_g[svid].nav_rcv_flag = 0;
					GLOBAL_PRINT(("GLONASS存在星历更新[%u]!!!!!\r\n", svid));
				}
				
				break;
			}

			case 3:
			{
				ublox_glo_eph_org_g[svid].nav_rcv_flag |= (0x1 << (str_index-1));
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 6, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 7, 10);	//取facc
				ublox_glo_eph_org_g[svid].glo_eph.facc = temp32*pow(2.0,-40)*(sign_bit ? (-1):1);
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 21, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 22, 23);	//取z'
				ublox_glo_eph_org_g[svid].glo_eph.zd = temp32*pow(2.0,-20)*(sign_bit ? (-1):1)*1e3;
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 45, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 46, 4);	//取z''
				ublox_glo_eph_org_g[svid].glo_eph.zdd = temp32*pow(2.0,-30)*(sign_bit ? (-1):1)*1e3;
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 50, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 51, 26);	//取z
				ublox_glo_eph_org_g[svid].glo_eph.z = temp32*pow(2.0,-11)*(sign_bit ? (-1):1)*1e3;
				break;
			}

			case 4:
			{
				ublox_glo_eph_org_g[svid].nav_rcv_flag |= (0x1 << (str_index-1));
				sign_bit = _ubx_glo_get_nbit(bit_tmp, 5, 1);	//取符号位
				temp32 = _ubx_glo_get_nbit(bit_tmp, 6, 21);	//取tn
				ublox_glo_eph_org_g[svid].glo_eph.tn = temp32*pow(2.0,-30)*(sign_bit ? (-1):1);
				ublox_glo_eph_org_g[svid].glo_eph.nt = _ubx_glo_get_nbit(bit_tmp, 59, 11);	//取nt
				break;
			}

			case 5:
			{
				ublox_glo_eph_org_g[svid].nav_rcv_flag |= (0x1 << (str_index-1));
				ublox_glo_eph_org_g[svid].glo_eph.n4 = _ubx_glo_get_nbit(bit_tmp, 49, 5);	//取n4
				break;
			}

			default:
				break;
		}
	}
	else if((payload_ptr[0] == 3) && (payload_len == UBX_SFRBX_BDS_LEN))	//解析北斗导航电文
	{
		static U32 tmp_toe[33] = {0};
		U8 cur_iode = 0;
		svid = payload_ptr[1];

        if(sys_debug_get_type(SYS_DEBUG_UBX))
        {
            GLOBAL_TIME_PRINT(("rxm_sfrbx rcved!!!!,BDS SVID:%u\r\n",svid));
        }
        
		if(svid > 37) return ret;
		ublox_bds_eph_org_g[svid].eph.prn = svid;
		GLOBAL_MEMCPY(temp_buf, payload_ptr+8, 40);
		GLOBAL_MEMSET(bit_tmp, 0x0, sizeof(bit_tmp));
		_ubx_glo_nav_bit_fill(temp_buf, bit_tmp, 10);	//取220bits
		if(svid > 5)	//非GEO卫星
		{
			frame_index = _ubx_gps_get_nbit(bit_tmp, 15, 3, 0);	//取子帧号
			switch(frame_index)
			{
				case 1:		//子帧1
				{
					ublox_bds_eph_org_g[svid].nav_rcv_flag |= (0x1 << 0);
					ublox_bds_eph_org_g[svid].eph.iodc = _ubx_gps_get_nbit(bit_tmp, 35, 5, 0);		//AODC
					ublox_bds_eph_org_g[svid].eph.week = _ubx_gps_get_nbit(bit_tmp, 44, 13, 0);	//week
					ublox_bds_eph_org_g[svid].eph.toc = _ubx_gps_get_nbit(bit_tmp, 57, 17, 0)*8;		//toc
					ublox_bds_eph_org_g[svid].eph.tgd = _ubx_gps_get_nbit(bit_tmp, 74, 10, 1)/1e10;	//tgd
					ublox_bds_eph_org_g[svid].eph.af2 = _ubx_gps_get_nbit(bit_tmp, 158, 11, 1)*pow(2.0, -66);	//af2
					ublox_bds_eph_org_g[svid].eph.af0  = _ubx_gps_get_nbit(bit_tmp, 169, 24, 1)*pow(2.0, -33);	//af0
					ublox_bds_eph_org_g[svid].eph.af1  = _ubx_gps_get_nbit(bit_tmp, 193, 22, 1)*pow(2.0, -50);	//af1
					cur_iode = _ubx_gps_get_nbit(bit_tmp, 215, 5, 0);	//iode
					if(cur_iode != ublox_bds_eph_org_g[svid].eph.iode)
					{
						ublox_bds_eph_org_g[svid].cur_valid = 0;
						ublox_bds_eph_org_g[svid].nav_rcv_flag = 0;
						ublox_bds_eph_org_g[svid].refresh_flag = 1;
						GLOBAL_PRINT(("BDS(%u)存在星历更新!!!!!!\r\n", svid));
					}
					ublox_bds_eph_org_g[svid].eph.iode = cur_iode;
					//ublox_bds_eph_org_g[svid].eph.iodc = cur_iode;
					if(ublox_bds_eph_org_g[svid].nav_rcv_flag == 0x7)
					{
						ublox_bds_eph_org_g[svid].cur_valid = 1;
						//ublox_bds_eph_org_g[svid].refresh_flag = 1;
					}
					if(ublox_bds_eph_org_g[svid].eph.toc != ublox_bds_eph_org_g[svid].eph.toe)
					{
						ublox_bds_eph_org_g[svid].cur_valid = 0;
						ublox_bds_eph_org_g[svid].nav_rcv_flag = 0;
						ublox_bds_eph_org_g[svid].refresh_flag = 1;
					}
					break;
				}

				case 2:
				{
					ublox_bds_eph_org_g[svid].nav_rcv_flag |= (0x1 << 1);
					ublox_bds_eph_org_g[svid].eph.deltn = _ubx_gps_get_nbit(bit_tmp, 34, 16, 1)*pow(2.0, -43)*PI;		//deltan
					ublox_bds_eph_org_g[svid].eph.cuc = _ubx_gps_get_nbit(bit_tmp, 50, 18, 1)*pow(2.0, -31);	//cuc
					ublox_bds_eph_org_g[svid].eph.m0 = _ubx_gps_get_nbit(bit_tmp, 68, 32, 1)*pow(2.0, -31)*PI;	//m0
					ublox_bds_eph_org_g[svid].eph.e = _ubx_gps_get_nbit(bit_tmp, 100, 32, 1)*pow(2.0, -33);	//e
					ublox_bds_eph_org_g[svid].eph.cus = _ubx_gps_get_nbit(bit_tmp, 132, 18, 1)*pow(2.0, -31);	//cus
					ublox_bds_eph_org_g[svid].eph.crc = _ubx_gps_get_nbit(bit_tmp, 150, 18, 1)*pow(2.0, -6);	//crc
					ublox_bds_eph_org_g[svid].eph.crs = _ubx_gps_get_nbit(bit_tmp, 168, 18, 1)*pow(2.0, -6);	//crs
					ublox_bds_eph_org_g[svid].eph.sqrta = ((U32)_ubx_gps_get_nbit(bit_tmp, 186, 32, 0))*pow(2.0, -19);	//sqrta
					//ublox_bds_eph_org_g[svid].eph.toe = (_ubx_gps_get_nbit(bit_tmp, 218, 2, 0) << 15);	//toe
					tmp_toe[svid] = (_ubx_gps_get_nbit(bit_tmp, 218, 2, 0) << 15);	//toe高2位
					break;
				}
				case 3:
				{
					ublox_bds_eph_org_g[svid].nav_rcv_flag |= (0x1 << 2);
					//ublox_bds_eph_org_g[svid].eph.toe |= _ubx_gps_get_nbit(bit_tmp, 34, 15, 0);	//toe
					tmp_toe[svid] |= _ubx_gps_get_nbit(bit_tmp, 34, 15, 0);	//toe
					ublox_bds_eph_org_g[svid].eph.toe = tmp_toe[svid]*8;
					ublox_bds_eph_org_g[svid].eph.i0 = _ubx_gps_get_nbit(bit_tmp, 49, 32, 1)*pow(2.0, -31)*PI;	//i0
					ublox_bds_eph_org_g[svid].eph.cic = _ubx_gps_get_nbit(bit_tmp, 81, 18, 1)*pow(2.0, -31);	//cic
					ublox_bds_eph_org_g[svid].eph.omegad = _ubx_gps_get_nbit(bit_tmp, 99, 24, 1)*pow(2.0, -43)*PI;	//omegad
					ublox_bds_eph_org_g[svid].eph.cis = _ubx_gps_get_nbit(bit_tmp, 123, 18, 1)*pow(2.0, -31);	//cis
					ublox_bds_eph_org_g[svid].eph.idot = _ubx_gps_get_nbit(bit_tmp, 141, 14, 1)*pow(2.0, -43)*PI;	//idot
					ublox_bds_eph_org_g[svid].eph.omega0 = _ubx_gps_get_nbit(bit_tmp, 155, 32, 1)*pow(2.0, -31)*PI;	//omega0
					ublox_bds_eph_org_g[svid].eph.w = _ubx_gps_get_nbit(bit_tmp, 187, 32, 1)*pow(2.0, -31)*PI;	//w
					if(ublox_bds_eph_org_g[svid].eph.toc != ublox_bds_eph_org_g[svid].eph.toe)
					{
						//ublox_bds_eph_org_g[svid].eph.toe = ublox_bds_eph_org_g[svid].eph.toc;
						ublox_bds_eph_org_g[svid].cur_valid = 0;
						ublox_bds_eph_org_g[svid].nav_rcv_flag = 0;
						ublox_bds_eph_org_g[svid].refresh_flag = 1;
					}
					break;
				}
				default:
					break;
			}
		}
		else
		{
			U8 page_num = 0;
			frame_index = _ubx_gps_get_nbit(bit_tmp, 15, 3, 0);	//取子帧号
			static S32 tmp_af1[6] = {0}, tmp_cuc[6] = {0}, tmp_e[6] = {0}, tmp_cic[6] = {0}, tmp_i0[6] = {0}, tmp_omegad[6] = {0}, tmp_w[6] = {0};
			
			if(frame_index == 1)	//子帧1
			{
				page_num = _ubx_gps_get_nbit(bit_tmp, 34, 4, 0);	//取页号
				switch(page_num)
				{
					case 1:
					{
						ublox_bds_eph_org_g[svid].eph.iodc = _ubx_gps_get_nbit(bit_tmp, 39, 5, 0);		//AODC
						ublox_bds_eph_org_g[svid].eph.week = _ubx_gps_get_nbit(bit_tmp, 48, 13, 0);		//week
						ublox_bds_eph_org_g[svid].eph.toc = _ubx_gps_get_nbit(bit_tmp, 61, 17, 0)*8;		//toc
						ublox_bds_eph_org_g[svid].eph.tgd = _ubx_gps_get_nbit(bit_tmp, 78, 10, 1)/1e10;		//tgd
						if(ublox_bds_eph_org_g[svid].eph.toc != ublox_bds_eph_org_g[svid].eph.toe)
						{
							ublox_bds_eph_org_g[svid].cur_valid = 0;
							ublox_bds_eph_org_g[svid].nav_rcv_flag = 0;
							ublox_bds_eph_org_g[svid].refresh_flag = 1;
						}
						break;
					}
					case 3:
					{
						ublox_bds_eph_org_g[svid].eph.af0 = _ubx_gps_get_nbit(bit_tmp, 76, 24, 1)*pow(2.0, -33);	//af0
						tmp_af1[svid] = _ubx_gps_get_nbit(bit_tmp, 100, 4, 1)<<18;	//af1高4bit
						//GLOBAL_HEX(tmp_af1);
						break;
					}

					case 4:
					{
						tmp_af1[svid] |= _ubx_gps_get_nbit(bit_tmp, 38, 18, 0);
						//GLOBAL_HEX(tmp_af1);
						ublox_bds_eph_org_g[svid].eph.af1 = tmp_af1[svid]*pow(2.0,-50);	//af1
						ublox_bds_eph_org_g[svid].eph.af2 = _ubx_gps_get_nbit(bit_tmp, 56, 11, 1)*pow(2.0, -66);	//af2
						cur_iode = _ubx_gps_get_nbit(bit_tmp, 67, 5, 0);		//AODE
						ublox_bds_eph_org_g[svid].eph.deltn = _ubx_gps_get_nbit(bit_tmp, 72, 16, 1)*pow(2.0, -43)*PI; //deltan
						tmp_cuc[svid] = _ubx_gps_get_nbit(bit_tmp, 88, 14, 1)<<4;	//cuc高14bit
						if(cur_iode != ublox_bds_eph_org_g[svid].eph.iode)
						{
							ublox_bds_eph_org_g[svid].cur_valid = 0;
							ublox_bds_eph_org_g[svid].nav_rcv_flag = 0;
							ublox_bds_eph_org_g[svid].refresh_flag = 1;
							GLOBAL_PRINT(("BDS(%u)存在星历更新!!!!!!\r\n", svid));
						}
						ublox_bds_eph_org_g[svid].eph.iode = cur_iode;
						//ublox_bds_eph_org_g[svid].eph.iodc = cur_iode;
						if(ublox_bds_eph_org_g[svid].nav_rcv_flag == 0x3FF)
						{
							ublox_bds_eph_org_g[svid].cur_valid = 1;
							//ublox_bds_eph_org_g[svid].refresh_flag = 1;
						}
						break;
					}

					case 5:
					{
						tmp_cuc[svid] |= _ubx_gps_get_nbit(bit_tmp, 38, 4, 0);
						ublox_bds_eph_org_g[svid].eph.cuc = tmp_cuc[svid]*pow(2.0, -31);	//cuc
						ublox_bds_eph_org_g[svid].eph.m0 = _ubx_gps_get_nbit(bit_tmp, 42, 32, 1)*pow(2.0, -31)*PI;	//m0
						ublox_bds_eph_org_g[svid].eph.cus = _ubx_gps_get_nbit(bit_tmp, 74, 18, 1)*pow(2.0, -31);	//cus
						tmp_e[svid] = _ubx_gps_get_nbit(bit_tmp, 92, 10, 1)<<22;	//cuc高10bit
						break;
					}

					case 6:
					{
						tmp_e[svid] |= _ubx_gps_get_nbit(bit_tmp, 38, 22, 0);
						ublox_bds_eph_org_g[svid].eph.e = tmp_e[svid]*pow(2.0, -33);	//e
						ublox_bds_eph_org_g[svid].eph.sqrta = ((U32)_ubx_gps_get_nbit(bit_tmp, 60, 32, 0))*pow(2.0, -19);	//sqrta
						tmp_cic[svid] = _ubx_gps_get_nbit(bit_tmp, 92, 10, 1)<<8;	//cic高10bit
						break;
					}

					case 7:
					{
						tmp_cic[svid] |= _ubx_gps_get_nbit(bit_tmp, 38, 8, 0);
						ublox_bds_eph_org_g[svid].eph.cic = tmp_cic[svid]*pow(2.0, -31);	//cic
						ublox_bds_eph_org_g[svid].eph.cis = _ubx_gps_get_nbit(bit_tmp, 46, 18, 1)*pow(2.0, -31);	//cis
						ublox_bds_eph_org_g[svid].eph.toe = _ubx_gps_get_nbit(bit_tmp, 64, 17, 0)*8;		//toe
						tmp_i0[svid] = _ubx_gps_get_nbit(bit_tmp, 81, 21, 1)<<11;	//i0高21bit;
						if(ublox_bds_eph_org_g[svid].eph.toc != ublox_bds_eph_org_g[svid].eph.toe)
						{
							ublox_bds_eph_org_g[svid].cur_valid = 0;
							ublox_bds_eph_org_g[svid].nav_rcv_flag = 0;
							ublox_bds_eph_org_g[svid].refresh_flag = 1;
						}
						break;
					}

					case 8:
					{
						tmp_i0[svid] |= _ubx_gps_get_nbit(bit_tmp, 38, 11, 0);
						ublox_bds_eph_org_g[svid].eph.i0 = tmp_i0[svid]*pow(2.0, -31)*PI;	//i0
						ublox_bds_eph_org_g[svid].eph.crc = _ubx_gps_get_nbit(bit_tmp, 49, 18, 1)*pow(2.0, -6);	//crc
						ublox_bds_eph_org_g[svid].eph.crs = _ubx_gps_get_nbit(bit_tmp, 67, 18, 1)*pow(2.0, -6);	//crs
						tmp_omegad[svid] = _ubx_gps_get_nbit(bit_tmp, 85, 19, 1)<<5;	//omegad高19bit;
						break;
					}

					case 9:
					{
						tmp_omegad[svid] |= _ubx_gps_get_nbit(bit_tmp, 38, 5, 0);
						ublox_bds_eph_org_g[svid].eph.omegad = tmp_omegad[svid]*pow(2.0, -43)*PI;	//omegad
						ublox_bds_eph_org_g[svid].eph.omega0 = _ubx_gps_get_nbit(bit_tmp, 43, 32, 1)*pow(2.0, -31)*PI;	//omega0
						tmp_w[svid] = _ubx_gps_get_nbit(bit_tmp, 75, 27, 1)<<5;	//w高19bit;
						break;
					}

					case 10:
					{
						tmp_w[svid] |= _ubx_gps_get_nbit(bit_tmp, 38, 5, 0);
						ublox_bds_eph_org_g[svid].eph.w = tmp_w[svid]*pow(2.0, -31)*PI;	//w
						ublox_bds_eph_org_g[svid].eph.idot = _ubx_gps_get_nbit(bit_tmp, 43, 14, 1)*pow(2.0, -43)*PI;	//idot
						break;
					}
					default:
						break;
				}
				ublox_bds_eph_org_g[svid].nav_rcv_flag |= (0x1 << (page_num-1));
			}
		}
	}
    else
    {
        if(sys_debug_get_type(SYS_DEBUG_UBX))
        {
           GLOBAL_TIME_PRINT(("rxm_sfrbx rcved!!!!,GNSS ID:%u, SVID:%u\r\n",payload_ptr[0],payload_ptr[1])); 
        } 
    }
    
#endif	
	return ret;
}


//解析UBX数据
U8 gnss_ublox_ubx_data_process(IN U8 *data_ptr, IN U32 len)
{
	U8 ck_a, ck_b, i = 0;

	if(len >= 8)
	{
		ublox_calc_ubx_checksum(data_ptr+2, len-4, &ck_a, &ck_b);
		if((ck_a == data_ptr[len-2]) && (ck_b == data_ptr[len-1]))
		{
			UBLOX_MSG_CLS_ID_Enum msg_id = (UBLOX_MSG_CLS_ID_Enum)((data_ptr[2] << 8) + data_ptr[3]);
			U32 payload_len = (data_ptr[5] << 8) + data_ptr[4];
			for(i = 0; i < UBX_PARSE_NUM; i++)
			{
				if((msg_id) && (_m_ubx_parse_array[i].ubx_msg_id == msg_id))
				{
					if(_m_ubx_parse_array[i].parse)
					{
						_m_ubx_parse_array[i].parse(data_ptr+6, payload_len);
					}
				}
			}
		}
	}
	return TRUE;
}

//解析UBX参数注册
static void _gnss_ublox_ubx_parse_register(UBLOX_MSG_CLS_ID_Enum msg_id, U8 (*parse)(U8*, U32))
{
	U32 i = 0;

	for(i = 0; i < UBX_PARSE_NUM; i++)
	{
		if(_m_ubx_parse_array[i].ubx_msg_id == 0)
		{
			_m_ubx_parse_array[i].ubx_msg_id = msg_id;
			_m_ubx_parse_array[i].parse = parse;
			break;
		}
	}
}

//固定位置模式设置
void gnss_ublox_enable_fixmode(void)
{
	uart_fml_send_gnss_cmd((U8*)ublox_cfg_fix_mode, sizeof(ublox_cfg_fix_mode));	//FIXMODE 配置
	delay_ms(10);
}

void gnss_ublox_disable_fixmode(void)
{
	uart_fml_send_gnss_cmd((U8*)ublox_cfg_disable_fix_mode, sizeof(ublox_cfg_disable_fix_mode));	//FIXMODE 配置
	delay_ms(10);
}

static void _gnss_ublox_cfg(void)
{
	if(p_gnss_handle->ubx_enable)
	{
		U32 i = 0;
		for(i = 0; i < sizeof(ublox_disable_ubx_cmd)/11; i++)
		{
			uart_fml_send_gnss_cmd((U8*)(ublox_disable_ubx_cmd+i*11), 11);	//禁止所有UBX协议数据
			delay_ms(10);
		}

        if(g_gnss_work_model == DEV_GPS_BDS_GLO)
        {
		    gnss_ublox_enable_gnss(GNSS_SWITCH_GPS_BDS);
        }
        else
        {
            gnss_ublox_enable_gnss(GNSS_SWITCH_GPS_GLO);            
        }
        
		//uart_fml_send_gnss_cmd((U8*)ublox_enable_gnss_cmd,sizeof(ublox_enable_gnss_cmd));
		delay_ms(10);
		for(i = 0; i < sizeof(ublox_enable_ubx_cmd)/11; i++)
		{
			uart_fml_send_gnss_cmd((U8*)(ublox_enable_ubx_cmd+i*11), 11);	//开启UBX协议数据
			delay_ms(10);
		}
		//uart_fml_send_gnss_cmd((U8*)(ublox_enable_ubx_cmd), sizeof(ublox_enable_ubx_cmd));
		uart_fml_send_gnss_cmd((U8*)ublox_enable_timepulse_gps, sizeof(ublox_enable_timepulse_gps));
		delay_ms(10);
		uart_fml_send_gnss_cmd((U8*)ublox_cfg_port_115200_ubx, sizeof(ublox_cfg_port_115200_ubx));
		delay_ms(10);
		//ext_ref_ublox_set_usart_baurd(UBLOX_OUT_UBX, 115200);	//将UBLOX串口波特率改为115200
	}
	else
	{
		_ublox_set_disable_all_nmea();
		_ublox_set_enable_nmea();
		GLOBAL_PRINT(("NMEA SET OPEN!!\R\N"));
		uart_fml_send_gnss_cmd((U8*)ublox_cfg_port_115200_nmea, sizeof(ublox_cfg_port_115200_nmea));
		uart_fml_send_gnss_cmd((U8*)ublox_save_cfg_cmd, sizeof(ublox_save_cfg_cmd));
	    //ext_ref_ublox_set_usart_baurd(UBLOX_OUT_NMEA, 115200);	//将UBLOX串口波特率改为115200
	}
}

/*****************************************************************************
 函 数 名  : gnss_ublox_cfg
 功能描述  : UBLOX配置
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年12月7日 星期五
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_cfg(void)
{
	WARN_PRINT(("gnss reciver cfg start!!!!\r\n"));
	uart_set_baudrate(GNSS_COM, 9600);
	_gnss_ublox_cfg();
	delay_ms(100);
	uart_set_baudrate(GNSS_COM, 115200);
	_gnss_ublox_cfg();
	delay_ms(100);
	uart_set_baudrate(GNSS_COM, 115200);
	WARN_PRINT(("gnss reciver cfg done!!!!\r\n"));
}

/*****************************************************************************
 函 数 名  : gnss_ublox_save_cfg
 功能描述  : UBLOX保存配置
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年12月7日 星期五
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_save_cfg(void)
{
	uart_fml_send_gnss_cmd((U8*)ublox_save_cfg_cmd, sizeof(ublox_save_cfg_cmd));	//保存UBLOX配置
	delay_ms(100);
	WARN_PRINT(("gnss reciver cfg save!!!!\r\n"));
}

/*****************************************************************************
 函 数 名  : gnss_ublox_hot_start
 功能描述  : UBLOX热启动
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019-10-14
    作    者   : sunj
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_hot_start(void)
{
	uart_fml_send_gnss_cmd((U8*)ublox_hot_start_cmd, sizeof(ublox_hot_start_cmd));	
	delay_ms(100);
	WARN_PRINT(("gnss reciver hot start!!!!\r\n"));
}

/*****************************************************************************
 函 数 名  : gnss_ublox_cold_start
 功能描述  : UBLOX冷启动
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019-10-14
    作    者   : sunj
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_cold_start(void)
{
	uart_fml_send_gnss_cmd((U8*)ublox_cold_start_cmd, sizeof(ublox_cold_start_cmd));	
	delay_ms(100);
	WARN_PRINT(("gnss reciver cold start!!!!\r\n"));
}

/*****************************************************************************
 函 数 名  : gnss_ublox_warm_start
 功能描述  : UBLOX暖启动
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019-10-14
    作    者   : sunj
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_warm_start(void)
{
	uart_fml_send_gnss_cmd((U8*)ublox_warm_start_cmd, sizeof(ublox_warm_start_cmd));	
	delay_ms(100);
	WARN_PRINT(("gnss reciver warm start!!!!\r\n"));
}


/*****************************************************************************
 函 数 名  : gnss_ublox_poll_aid_eth
 功能描述  : 获取GPS星历数据
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年12月10日 星期一
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_poll_aid_eth(void)
{
	uart_fml_send_gnss_cmd((U8*)ublox_ubx_poll_aid_eth,sizeof(ublox_ubx_poll_aid_eth));
}

/*****************************************************************************
 函 数 名  : gnss_ublox_poll_aid_alm
 功能描述  : 获取GPS历书数据
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年2月27日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_poll_aid_alm(void)
{
	uart_fml_send_gnss_cmd((U8*)ublox_ubx_poll_aid_alm,sizeof(ublox_ubx_poll_aid_alm));
}

/*****************************************************************************
 函 数 名  : gnss_ublox_enable_gnss
 功能描述  : 使能GPS
 输入参数  :   
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年2月27日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_enable_gnss(GNSS_SWITCH_TYPE_Enum type)
{
    if(type==GNSS_SWITCH_GPS_BDS)
    {
        g_gnss_switch_type = GNSS_SWITCH_GPS_BDS;
        uart_fml_send_gnss_cmd((U8*)ublox_cfg_gnss_enable_gps_bds,sizeof(ublox_cfg_gnss_enable_gps_bds));
    }
    else
    {
        g_gnss_switch_type = GNSS_SWITCH_GPS_GLO;
        uart_fml_send_gnss_cmd((U8*)ublox_cfg_gnss_enable_gps_glo,sizeof(ublox_cfg_gnss_enable_gps_glo));
    }
    
	delay_ms(10);

}

/*****************************************************************************
 函 数 名  : gnss_ublox_disable_gnss
 功能描述  : 禁止GPS
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年2月27日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_disable_gnss(void)
{
	uart_fml_send_gnss_cmd((U8*)ubloc_cfg_gnss_disable_gnss,sizeof(ubloc_cfg_gnss_disable_gnss));
	delay_ms(10);
}

/*****************************************************************************
 函 数 名  : gnss_ublox_set_ubx_enable
 功能描述  : ubx使能设置
 输入参数  : U8 flag  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月5日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ublox_set_ubx_enable(U8 flag)
{
	p_gnss_handle->ubx_enable = flag;
	gnss_ublox_cfg();
}

//获取pps残差
F32 gnss_ublox_get_1pps_err(void)
{
	return p_gnss_handle->pps_err;
}

/*****************************************************************************
 函 数 名  : gnss_ubx_init
 功能描述  : ubx初始化
 输入参数  : gnss_data_t* p_handle  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月5日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_ubx_init(gnss_data_t* p_handle)
{
	if(!p_handle)
	{
		ERR_PRINT(("指针非法!!!!\r\n"));
		return;
	}
	p_gnss_handle = p_handle;
	/*
	_gnss_ublox_ubx_parse_register(UBX_NAV_VELECEF, _ublox_parse_ubx_velecef);
	_gnss_ublox_ubx_parse_register(UBX_NAV_DOP, _ublox_parse_ubx_dop);
	_gnss_ublox_ubx_parse_register(UBX_NAV_PVT, _ublox_parse_ubx_pvt);
	_gnss_ublox_ubx_parse_register(UBX_NAV_SAT, _ublox_parse_ubx_sat);
	_gnss_ublox_ubx_parse_register(UBX_NAV_TIMEGPS, _ublox_parse_ubx_timegps);
	_gnss_ublox_ubx_parse_register(UBX_TIM_TP, _ublox_parse_ubx_tim_tp);
	_gnss_ublox_ubx_parse_register(UBX_AID_EPH, _ublox_parse_ubx_aid_eph);
	_gnss_ublox_ubx_parse_register(UBX_AID_ALM, _ublox_parse_ubx_aid_alm);
	_gnss_ublox_ubx_parse_register(UBX_RXM_SFRBX, _ublox_parse_ubx_rxm_sfrbx);
	*/
	//gnss_ublox_enable_gnss();
	//gnss_ublox_cfg();
	if(!p_gnss_handle->ubx_enable)
	{
		//_ublox_set_open_pubx4();
	}
}


/*****************************************************************************
 函 数 名  : gnss_mxt_init
 功能描述  : mxt初始化
 输入参数  : 无  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月5日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void gnss_mxt_init(void)
{
	static char* buf_0 = "$reset,1,2\r\n";
	static char* buf_1 = "$cfgprt,,,4,1;cfgsave,\r\n";
	static char* buf_2 = "$cfgmsg,2,1075,0;cfgmsg,2,1085,0;cfgmsg,2,1125,0;cfgmsg,2,1019,0;cfgmsg,2,1020,0;cfgmsg,2,1042,0;cfgsave,\r\n";
	static char* buf_3 = "$cfgmsg,2,1005,1;cfgmsg,2,1074,1;cfgmsg,2,1084,1;cfgmsg,2,1124,1;cfgtpm,2,900,,,,;cfgsave,\r\n";

	//NOTE_PRINT(("buf_1 = %s\r\n",buf_1));
	uart_send_data(MXT_COM,(U8*)buf_1,strlen(buf_1));	
	NOTE_PRINT(("Sending MXT CFG 1 !!\r\n"));
	
	delay_ms(500);
	/*
	uart_send_data(MXT_COM,buf_2,strlen(buf_1));
	NOTE_PRINT(("Sending MXT CFG 2 !!\r\n"));
	delay_ms(500);
	uart_send_data(MXT_COM,buf_3,strlen(buf_2));
	NOTE_PRINT(("Sending MXT CFG 3 !!\r\n"));
	delay_ms(500);
*/
}

/*eof*/
