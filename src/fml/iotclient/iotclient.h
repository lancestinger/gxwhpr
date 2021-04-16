#ifndef _IOTCLIENT_H__
#define _IOTCLIENT_H__

#include "project_def.h"

typedef enum
{
    IOT_STA_INIT         = 0,            /* 初始化 */
    IOT_STA_MQTTCONNECT  = 1,            /* 连接服务器 */
    IOT_STA_MQTTSUB      = 2,                /* 订阅消息 */
    IOT_STA_IDEL         = 3,            /* 空闲 */
}IOT_WorkState_t;


typedef struct
{
    UN32INT un32Ip;         /* 服务器IP地址 */
    U16 Port;               /* 服务器端口 */
}IOT_Net;


typedef struct
{
    IOT_WorkState_t sta;		/* IOT工作状态 */
    S32 mqttSock;               //MQTT SOCKET文件描述符
    U8 bInit;
    U8 bMqttConnect;
    U8 bMqttSub;
    char clienId[50];           //客户端ID
    char subTopicName[50];      //订阅消息主题
    char pubTopicName[50];      //推送消息主题
    IOT_Net net;

}IOT_Device_t;

typedef enum
{
	SUSBEND_RCV = 0,            //挂起MQTT接收线程
	RESUME_RCV  = 1,            //恢复MQTT接收线程
}Susb_resum_rcv;

typedef enum
{
	MQTT_CMD = 0,              //MQTT订阅cmd
	MQTT_RTCM  = 1,            //MQTT订阅RTCM数据
	
}MQTT_Sub_type;

typedef enum
{
	NTRIP_GET_RTCM = 0,            //Ntrip接收RTCM
	MQTT_GET_RTCM  = 1,            //MQTT订阅RTCM数据
	
}Get_rtcm_source;


extern U8 RTCM_SWITCH;
extern IOT_Device_t iotDev;
extern IOT_Device_t iotCmdDev;
extern IOT_Device_t iotRTCMDev;

extern int MQTT_OFF_LINE;
extern osThreadId_t thread_iotclient_rcv_id;

/*初始化*/
extern void iotclient_init(void);
extern void mqttDisconnect(IOT_Device_t* iotDev);
extern int deviceSubscribe(IOT_Device_t* iotDev);
extern int rebuildMQTTConnection(void);

/*推送消息*/
extern int iotclient_publish(IOT_Device_t* iotDev, const char* payload, int payloadlen);

#endif
