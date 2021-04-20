
#ifndef _PROJECT_DEF_H_
#define _PROJECT_DEF_H_
#include "pubdef.h"
#include "gnss/gnss_fml.h"
#include "monitor/monitor_apl.h"


#define GPS_MAX_NUM		50





//全局事件标志,bit位有效
typedef enum
{
	OS_EVENT_PPS = (0x1<<0),				//PPS事件
	OS_EVENT_DOPLER_UPDATE = (0x1<<1),	// 多普勒更新事件
	OS_EVENT_PPS_SSHELL = (0x1<<2),		//PPS指向SSHELL事件
}os_event_flag_enum;

//设备类型
typedef enum
{
	DEV_GPS_GLO = 0,
	DEV_GPS_BDS_GLO,
	// DEV_GPS_BDS,
	// DEV_BDS_GLO
}device_enum;

//调试串口工作模式
typedef enum
{
    DBG_OUT_USART_NORMAL_MODEL   = 0,
    DBG_OUT_USART_RECEIVER_MODEL = 1
}DBG_OUT_USART_WORK_MODEL;

extern device_enum g_gnss_work_model;          //工作模式  0表示GPS&GLONASS  1表示GPS&BDS&GLONASS
extern DBG_OUT_USART_WORK_MODEL g_dbg_out_usart_model;     //调试串口工作模式

//协议星历结构体数据
#pragma pack(4)

typedef struct
{
	U16 sat_num;
	U16 sat_type;
	U16 prn;
	U16 week;
	U16 iodc;
	U32 toc;
	S16 af2;
	S16 af1;
	S32 af0;
	U16 iode;
	S16 crs;
	S16 delta_n;
	S32 m0;
	S16 cuc;
	U64 e;
	S16 cus;
	U64 sqrta;
	U32 toe;
	S16 cic;
	S32 omega0;
	S16 cis;
	S32 i0;
	S16 crc;
	S32 w;
	S32 omegad;
	S16 tgd;
	S16 aodo;
	S16 idot;
}EPH_t;

typedef struct
{
	U8 valid;
	U8 use;
	U8 cn0;
	S8 elev;
	EPH_t eph;
}PROTOCOL_EPH_t;

typedef struct
{
	U8 cur_valid;
	U8 use;
	U16 nav_rcv_flag;
	U8 refresh_flag;
	S8 elev;
	//eph_type1_t eph;
}PROTOCOL_GPS_ORG_EPH_t;

typedef struct
{
	U8 gnss_num;
	U8 gnss_type;
	U16 prn;
	U32 toa;
	F32 deltI;
	F64 e;
	F64 sqrta;
	F64 omega0;
	F64 w;
	F64 m0;
	F64 omegad;
	F32 af1;
	F32 af0;
	U32 week;
}GPS_ALM_ORG_t;

typedef struct
{
	//U8 prn;			//卫星编号
	U8 valid_num;	//总的有效卫星数目
	U8 cur_valid;	//当前卫星是否有效
	U16 resvd;
	GPS_ALM_ORG_t alm;
}PROTOCOL_ALM_t;

typedef struct
{
	U8 valid_num;	//总的有效卫星数目
	U8 cur_valid;	//当前卫星是否有效
	U8 refresh_flag;	//刷新 标志
	U8 use;
	U8 nav_rcv_flag;	//导航电文接收标志
	U32 pre_tb;			//保存之前的星历参考时刻
//	eph_type2_t glo_eph;
}PROTOCOL_GLO_EPH_t;

typedef struct
{
    U8 predicNum;   //推算次数,当为0时表示为直接使用星历数据,未进行自行推算
//    eph_type1_t eph;
}GPS_CUR_EPH_TypeDef;

typedef struct
{
    U8 predicNum;   //推算次数,当为0时表示为直接使用星历数据,未进行自行推算
//    eph_type2_t eph;
}GLO_CUR_EPH_TypeDef;


//网络数据结构体
typedef struct
{
	U8 dhcp;		//DHCP功能 0---禁止， 1---使能
	U8 net_ip[4];
	U8 net_mask[4];
	U8 net_gateway[4];
	U8 net_mac[6];
	U8 server_ip[4];
	U16 server_port;
}net_para_t;

typedef struct
{
	U8 ubx_enable;
	S8 gps_leap_sec;				//GPS闰秒数
	// S8 time_zone;					//时区
	// U8 pl_start_enable;			    //伪卫星上电自启
//	ref_type_enum ref_sel;			//参考源选择
//	ref_switch_enum ref_sw_mode;	//切换模式
//	S32 ocxo_voltage[OCXO_NUM];	    //压控电压
//	lla_t usr_lla;					//用户位置
	// F32 el;							//截止角
	// F32 max_speed;					//最大速度
//	F32 atten[4];			//衰减值
//	F32 delay[4];
//	circle_para_t circle_para;		//圆周运动参数
//	head_speed_t speed_para;		//直线运动参数
//	head_acc_t accspeed_para;		//加速度参数
	net_para_t net;					//网络参数
    // U32 work_model;                 //工作模式 0表示GPS&GLONASS 1表示GPS&BDS&GLONASS
    // U32 pl_auto_time_enable;        //伪卫星自动授时使能
    // U32 circle_curPos_enable;       //使用当前定位位置作为圆心进行圆周运动使能
    // U8 sendIP1[4];                  //发送IP1
    // U8 sendIP2[4];                  //发送IP2
    // U32 devInfoSendCycle;           //发送状态巡检信息周期
    // F32 atten_lim[4];        //通道功率衰减最小设置
    // F32 atten_offset[4][12];
    // U8 transCtrlMode;               //发射控制模式(安则)
    // U8 antsup;                      //天线馈电状态
    // U8 reserved2;                   //保留
    // U8 reserved3;                   //保留
	U32 crc;
}cfg_para_t;

typedef struct
{
	U32 used_seconds;	//已用秒数
	//S32 remain_seconds;	//剩余秒数
	U32 crc16;
}extra_para_t;

//主操作对象
typedef struct
{
	cfg_para_t cfg;
	gnss_data_t* p_gnss_handle;
//	ocxo_handle_t* p_ocxo1_handle;
//	ref_handle_t* p_ref_handle;
//	time_handle_t* p_time_handle;
//	pl_handle_t* p_pl_handle;
	monitor_status_t* p_monitor;
}main_handle_t;

//license数据结构
typedef struct
{
    U32 license;
    U32 crc;
}license_handle_t;

//hn配置文件使用数据结构
typedef struct
{
    U8 type;//1表示启动正常程序,2表示启动测试程序,其他值亦表示启动正常程序
    U8 reserve1;
    U8 reserve2;
    U8 reserve3;
    U32 crc;
}hn_handle_t;



#pragma pack()


extern extra_para_t ex_para_g;
extern main_handle_t main_handle_g;
extern osEventFlagsId_t os_event_id_g;
extern license_handle_t license_handle_g;  //license操作对象
extern hn_handle_t hn_handle_g;    //hn配置操作对象

extern void sys_para_init(void);
extern S8 sys_para_get_gps_leapsec(void);
extern void sys_set_ipaddr(U8* ip_buf);
extern void sys_set_maskaddr(U8* ip_buf);
extern void sys_set_gatewayaddr(U8* ip_buf);
extern void sys_set_macaddr(U8* ip_buf);
extern U8 sys_para_set_authcode(U32 code);
extern void sys_reset_para_save(void);
extern void sys_para_save(void);
extern void sys_net_cfg_start(void);
extern void sys_para_reset(void);

extern void license_reset(void);
extern void license_save(void);
extern void hn_cfg_save(void);
#endif
/*eof*/
