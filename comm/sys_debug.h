
#ifndef _SYS_DEBUG_H_
#define _SYS_DEBUG_H_

//#include "PubDef.h"


typedef enum
{
	SYS_DEBUG_TEST = 0,			//����
	SYS_DEBUG_WARN,				//�澯��ӡ
	SYS_DEBUG_ERROR,			//�����ӡ
	SYS_DEBUG_NOTE,				//��ʾ��ӡ
	SYS_DEBUG_CTRL,				//��������Э�����
	SYS_DEBUG_GNSS,				//GNSSЭ�����
	SYS_DEBUG_OCXO,				//OCXO
	SYS_DEBUG_EPH_GPS,			//��������
    SYS_DEBUG_EPH_BDS,			//��������
    SYS_DEBUG_EPH_GLO,			//��������
	SYS_DEBUG_ANT,				//���߼��
	SYS_DEBUG_NAV,				//��������
	SYS_DEBUG_SOCKET,			//SOCKET����
	SYS_DEBUG_AD936X,			//AD936X����
	SYS_DEBUG_FPGA,			    //FPGA����
	//SYS_DEBUG_RB,				//��ӵ���
	SYS_DEBUG_TDC,				//TDC����
	SYS_DEBUG_PL,				//PL����
	SYS_DEBUG_PROTOCOL_NET,		//����Э�����
	SYS_DEBUG_TIMER,
    SYS_DEBUG_UBX,              //UBXЭ���ӡ
	SYS_DEBUG_UBX_SAT,			//UBXЭ������״̬��ӡ
    SYS_DEBUG_TEMP,             //�¶ȼ���ӡ
    SYS_DEBUG_BCODE,            //B���ӡ
    SYS_DEBUG_JSON,             //JSON��ӡ
    SYS_DEBUG_USARTCTRL,        //���ڿ����豸��ӡ
    SYS_DEBUG_SAT,
    SYS_DEBUG_EFS_GENERAL,					//UWBһ����Ϣ��ӡ    
    SYS_DEBUG_EFS_POST,							//UWB��λ��Ϣ��ӡ
    SYS_DEBUG_EFS_POST_NOTE,	//UWB�������ź�������Ϣ��ӡ
    SYS_DEBUG_EFS_RANGING,					//�����Ϣ��ӡ
    SYS_DEBUG_EFS_RSSI,				   		//UWB�������ź�ǿ�ȴ�ӡ
    SYS_DEBUG_EFS_ERR,				  	 	//UWB�����ӡ    
    SYS_DEBUG_SERVER,           		//������������Ϣ��ӡ
	SYS_DEBUG_MQTT,             //MQTT����
	SYS_DEBUG_ENHANCE,          //��ǿ��վ��Ϣ
	SYS_DEBUG_RTK,              //RTK��λ��Ϣ��ӡ
	SYS_DEBUG_GGA,             //NMEA GGA��Ϣ��ӡ
	SYS_DEBUG_RMC,             //NMEA RMC��Ϣ��ӡ
	SYS_DEBUG_NTRIP,            //Ntrip������Ϣ��ӡ	
    SYS_DEBUG_IMU,              //IMUλ���������ں������Ϣ��ӡ
    SYS_DEBUG_IMU_RAW,          //IMUԭʼ������Ϣ��ӡ
    
	SYS_DEBUG_NUM,				//��������  
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

