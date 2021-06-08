
#ifndef _SYS_DEBUG_H_
#define _SYS_DEBUG_H_

//#include "PubDef.h"


typedef enum
{
	SYS_DEBUG_TEST = 0,			//测试
	SYS_DEBUG_WARN,				//告警打印
	SYS_DEBUG_ERROR,			//错误打印
	SYS_DEBUG_NOTE,				//提示打印
	SYS_DEBUG_CTRL,				//控制中心协议测试
	SYS_DEBUG_GNSS,				//GNSS协议调试
	SYS_DEBUG_OCXO,				//OCXO
	SYS_DEBUG_EPH_GPS,			//星历换星
    SYS_DEBUG_EPH_BDS,			//星历换星
    SYS_DEBUG_EPH_GLO,			//星历换星
	SYS_DEBUG_ANT,				//天线检测
	SYS_DEBUG_NAV,				//导航电文
	SYS_DEBUG_SOCKET,			//SOCKET调试
	SYS_DEBUG_AD936X,			//AD936X调试
	SYS_DEBUG_FPGA,			    //FPGA调试
	//SYS_DEBUG_RB,				//銣钟调试
	SYS_DEBUG_TDC,				//TDC测试
	SYS_DEBUG_PL,				//PL测试
	SYS_DEBUG_PROTOCOL_NET,		//网络协议调试
	SYS_DEBUG_TIMER,
    SYS_DEBUG_UBX,              //UBX协议打印
	SYS_DEBUG_UBX_SAT,			//UBX协议卫星状态打印
    SYS_DEBUG_TEMP,             //温度监测打印
    SYS_DEBUG_BCODE,            //B码打印
    SYS_DEBUG_JSON,             //JSON打印
    SYS_DEBUG_USARTCTRL,        //串口控制设备打印
    SYS_DEBUG_SAT,
    SYS_DEBUG_EFS_GENERAL,					//UWB一般信息打印    
    SYS_DEBUG_EFS_POST,							//UWB定位信息打印
    SYS_DEBUG_EFS_POST_NOTE,	//UWB测距接受信号质量信息打印
    SYS_DEBUG_EFS_RANGING,					//测距信息打印
    SYS_DEBUG_EFS_RSSI,				   		//UWB测距接受信号强度打印
    SYS_DEBUG_EFS_ERR,				  	 	//UWB错误打印    
    SYS_DEBUG_SERVER,           		//服务器交互信息打印
	SYS_DEBUG_MQTT,             //MQTT测试
	SYS_DEBUG_ENHANCE,          //增强基站消息
	SYS_DEBUG_RTK,              //RTK定位信息打印
	SYS_DEBUG_GGA,             //NMEA GGA消息打印
	SYS_DEBUG_RMC,             //NMEA RMC消息打印
	SYS_DEBUG_NTRIP,            //Ntrip调试信息打印	
    SYS_DEBUG_IMU,              //IMU位置输入与融合输出信息打印
    SYS_DEBUG_IMU_RAW,          //IMU原始数据信息打印
    
	SYS_DEBUG_NUM,				//调试数量  
}SYS_DEBUG_TYPE_Enum;

#define SYS_DBG_Print(module, format, ...) \
{\
if(sys_debug_get_type(module))\
{print_def(format, ##__VA_ARGS__);}\
}
//{print("[%s][%d] ", __FUNCTION__, __LINE__);print(format, ##__VA_ARGS__);}
/*
#define DBG_Tuner_Print(x, ...)				SYS_DBG_Print(SYS_DEBUG_TUNER, x, ##__VA_ARGS__)
#define DBG_Ocxo_Print(x, ...)				SYS_DBG_Print(SYS_DEBUG_OCXO, x, ##__VA_ARGS__)
#define DBG_SipDelay_Print(x, ...)			SYS_DBG_Print(SYS_DEBUG_SIP_DELAY, x, ##__VA_ARGS__)
#define DBG_ExtProtocol_Print(x, ...)		SYS_DBG_Print(SYS_DEBUG_EXTPROTOCOL, x, ##__VA_ARGS__)
//#define DBG_IntProtocol_Print(x, ...)		SYS_DBG_Print(SYS_DEBUG_INTPROTOCOL, x, ##__VA_ARGS__)
#define DBG_SFN_Print(x, ...)				SYS_DBG_Print(SYS_DEBUG_SFN, x, ##__VA_ARGS__)
#define DBG_GNS_Print(x, ...)				SYS_DBG_Print(SYS_DEBUG_GNS, x, ##__VA_ARGS__)
#define DBG_CodeRate_Print(x, ...)			SYS_DBG_Print(SYS_DEBUG_CODE_RATE, x, ##__VA_ARGS__)
*/
#define DBG_PROTOCOL_SERIAL_Print(x, ...)			SYS_DBG_Print(SYS_DEBUG_PROTOCOL_SERIAL, x, ##__VA_ARGS__)
#define DBG_PROTOCOL_NET_Print(x, ...)			    SYS_DBG_Print(SYS_DEBUG_PROTOCOL_NET, x, ##__VA_ARGS__)
#define DBG_TEST_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_TEST, x, ##__VA_ARGS__)
#define DBG_TRAN_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_TRAN, x, ##__VA_ARGS__)
#define DBG_GNSS_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_GNSS, x, ##__VA_ARGS__)
#define DBG_Ocxo_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_OCXO, x, ##__VA_ARGS__)
#define DBG_TDC_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_TDC, x, ##__VA_ARGS__)
#define DBG_TIMER_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_TIMER, x, ##__VA_ARGS__)
#define DBG_EPH_GPS_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_EPH_GPS, x, ##__VA_ARGS__)
#define DBG_EPH_BDS_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_EPH_BDS, x, ##__VA_ARGS__)
#define DBG_EPH_GLO_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_EPH_GLO, x, ##__VA_ARGS__)
#define DBG_PL_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_PL, x, ##__VA_ARGS__)
#define DBG_ANT_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_ANT, x, ##__VA_ARGS__)
#define DBG_NAV_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_NAV, x, ##__VA_ARGS__)
#define DBG_Socket_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_SOCKET, x, ##__VA_ARGS__)
#define DBG_AD936x_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_AD936X, x, ##__VA_ARGS__)
#define DBG_FPGA_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_FPGA, x, ##__VA_ARGS__)
#define DBG_UBX_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_UBX, x, ##__VA_ARGS__)
#define DBG_UBX_SAT_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_UBX_SAT, x, ##__VA_ARGS__)
#define DBG_TEMP_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_TEMP, x, ##__VA_ARGS__)
#define DBG_BCODE_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_BCODE, x, ##__VA_ARGS__)
#define DBG_JSON_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_JSON, x, ##__VA_ARGS__)
#define DBG_SAT_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_SAT, x, ##__VA_ARGS__)
#define DBG_EFS_GENERAL_Print(x, ...)				SYS_DBG_Print(SYS_DEBUG_EFS_GENERAL, x, ##__VA_ARGS__)
#define DBG_EFS_POST_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_EFS_POST, x, ##__VA_ARGS__)
#define DBG_EFS_RANGING_Print(x, ...)				SYS_DBG_Print(SYS_DEBUG_EFS_RANGING, x, ##__VA_ARGS__)
#define DBG_EFS_POST_NOTE_Print(x, ...)				SYS_DBG_Print(SYS_DEBUG_EFS_POST_NOTE, x, ##__VA_ARGS__)
#define DBG_EFS_RSSI_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_EFS_RSSI, x, ##__VA_ARGS__)
#define DBG_EFS_ERR_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_EFS_ERR, x, ##__VA_ARGS__)
#define DBG_SERVER_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_SERVER, x, ##__VA_ARGS__)
#define DBG_MQTT_Print(x, ...)						SYS_DBG_Print(SYS_DEBUG_MQTT, x, ##__VA_ARGS__)
#define DBG_ENHANCE_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_ENHANCE, x, ##__VA_ARGS__)
#define DBG_RTK_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_RTK, x, ##__VA_ARGS__)
#define DBG_GGA_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_GGA, x, ##__VA_ARGS__)
#define DBG_RMC_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_RMC, x, ##__VA_ARGS__)
#define DBG_NTRIP_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_NTRIP, x, ##__VA_ARGS__)
#define DBG_IMU_Print(x, ...)					    SYS_DBG_Print(SYS_DEBUG_IMU, x, ##__VA_ARGS__)
#define DBG_IMU_RAW_Print(x, ...)					SYS_DBG_Print(SYS_DEBUG_IMU_RAW, x, ##__VA_ARGS__)



extern void sys_debug_init(void);
extern void sys_debug_set_type(SYS_DEBUG_TYPE_Enum type);
extern void sys_debug_clear_type(SYS_DEBUG_TYPE_Enum type);
//#define sys_debug_get_type(x) 1
extern unsigned char sys_debug_get_type(SYS_DEBUG_TYPE_Enum type);
extern void sys_debug_clear_all(void);



#endif

/*EOF*/

