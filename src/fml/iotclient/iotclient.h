#ifndef _IOTCLIENT_H__
#define _IOTCLIENT_H__

#include "project_def.h"

typedef enum
{
    IOT_STA_INIT         = 0,            /* ��ʼ�� */
    IOT_STA_MQTTCONNECT  = 1,            /* ���ӷ����� */
    IOT_STA_MQTTSUB      = 2,                /* ������Ϣ */
    IOT_STA_IDEL         = 3,            /* ���� */
}IOT_WorkState_t;


typedef struct
{
    UN32INT un32Ip;         /* ������IP��ַ */
    U16 Port;               /* �������˿� */
}IOT_Net;


typedef struct
{
    IOT_WorkState_t sta;		/* IOT����״̬ */
    S32 mqttSock;               //MQTT SOCKET�ļ�������
    U8 bInit;
    U8 bMqttConnect;
    U8 bMqttSub;
    char clienId[50];           //�ͻ���ID
    char subTopicName[50];      //������Ϣ����
    char pubTopicName[50];      //������Ϣ����
    IOT_Net net;

}IOT_Device_t;

typedef enum
{
	SUSBEND_RCV = 0,            //����MQTT�����߳�
	RESUME_RCV  = 1,            //�ָ�MQTT�����߳�
}Susb_resum_rcv;

typedef enum
{
	MQTT_CMD = 0,              //MQTT����cmd
	MQTT_RTCM  = 1,            //MQTT����RTCM����
	
}MQTT_Sub_type;

typedef enum
{
	NTRIP_GET_RTCM = 0,            //Ntrip����RTCM
	MQTT_GET_RTCM  = 1,            //MQTT����RTCM����
	
}Get_rtcm_source;


extern U8 RTCM_SWITCH;
extern IOT_Device_t iotDev;
extern IOT_Device_t iotCmdDev;
extern IOT_Device_t iotRTCMDev;

extern int MQTT_OFF_LINE;
extern osThreadId_t thread_iotclient_rcv_id;

/*��ʼ��*/
extern void iotclient_init(void);
extern void mqttDisconnect(IOT_Device_t* iotDev);
extern int deviceSubscribe(IOT_Device_t* iotDev);
extern int rebuildMQTTConnection(void);

/*������Ϣ*/
extern int iotclient_publish(IOT_Device_t* iotDev, const char* payload, int payloadlen);

#endif
