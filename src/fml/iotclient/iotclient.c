#include "iotclient.h"
#include "drv/MQTTPacket/MQTTPacket.h"
#include "drv/MQTTPacket/MQTTConnect.h"
#include "transport/transport.h"
#include "project_def.h"
#include "server/server_apl.h"
#include "apl/Ntrip/Ntrip.h"
#include "drv/gpio/gpio_drv.h"
#include "fml/gnss/nmea.h"


static U64 thread_iotclient_monitor_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_iotclient_monitor_attr = {
  .stack_mem  = &thread_iotclient_monitor_stk[0],
  .stack_size = sizeof(thread_iotclient_monitor_stk),
  .priority = osPriorityBelowNormal6,
};

static U64 thread_iotclient_rcv_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_iotclient_rcv_attr = {
  .stack_mem  = &thread_iotclient_rcv_stk[0],
  .stack_size = sizeof(thread_iotclient_rcv_stk),
  .priority = osPriorityNormal,
};

IOT_Device_t iotDev;
IOT_Device_t iotRTCMDev;
IOT_Device_t iotCmdDev;

int MQTT_OFF_LINE=FALSE;
osThreadId_t thread_iotclient_rcv_id = 0;
U8 RTCM_SWITCH = NTRIP_GET_RTCM;

static unsigned char mqttbuf[2048];
static unsigned char rcvbuf[2048];
static char dedata[1200];
static U8 FIRST_DEV=TRUE;

/*****************************************************************************
 �� �� ��  : mqttDisconnect
 ��������  : �Ͽ�MQTT���Ӻ���
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
/*�Ͽ�MQTT����*/
void mqttDisconnect(IOT_Device_t* iotDev)
{
    unsigned char buf[10];
    int buflen = sizeof(buf);
    int len = 0;

    len = MQTTSerialize_disconnect(buf, buflen);
    transport_sendPacketBuffer(buf, len);

    iotDev->bMqttConnect = FALSE;
    if(iotDev->mqttSock>0)
    {
        DBG_MQTT_Print("�ر�SOCKET[%d]\r\n",iotDev->mqttSock);
        transport_close(iotDev->mqttSock);
    }    
}


/*****************************************************************************
 �� �� ��  : iotDev_reset
 ��������  : MQTT����reset����
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
static void iotDev_reset(IOT_Device_t* iotDev)
{
    DBG_MQTT_Print("reset mqtt client begin!!!\r\n");
    mqttDisconnect(iotDev);
    iotDev->bMqttSub = FALSE;
    iotDev->mqttSock = -1;
    DBG_MQTT_Print("reset mqtt client finish!!!\r\n");
}


/*****************************************************************************
 �� �� ��  : connectMqttServer
 ��������  : ����MQTT����������
 �������  : ��
 �������  : ��
 �� �� ֵ  : ret
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����
  2.��    ��   : 2021��3��16��
    ��    ��   : zxf
    �޸�����   : ���MQTT��������

*****************************************************************************/
static int connectMqttServer(IOT_Device_t* iotdev)
{
    int ret = -1;

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    int rc = 0;
	unsigned char buf[200];
	int buflen = sizeof(buf);
	int len = 0;

	SOCKADDR_IN server_addr;

    server_addr.sin_family = PF_INET;
    server_addr.sin_addr.s_b1 = iotdev->net.un32Ip.u8Val[0];
    server_addr.sin_addr.s_b2 = iotdev->net.un32Ip.u8Val[1];
    server_addr.sin_addr.s_b3 = iotdev->net.un32Ip.u8Val[2];
    server_addr.sin_addr.s_b4 = iotdev->net.un32Ip.u8Val[3];
    server_addr.sin_port = htons(iotdev->net.Port);

	if(!iotDev.bMqttConnect)
	{
		iotDev.mqttSock = transport_open(server_addr);
	}
	else
	{
		iotdev->mqttSock = iotDev.mqttSock;
		iotdev->bMqttConnect = TRUE;
		return 0;
	}
                                                                                                                                                                     
    data.clientID.cstring = iotdev->clienId;//�ͻ���ID
	data.keepAliveInterval = 20;//�������������ʱ��
	data.cleansession = 1;//����Ự
	data.username.cstring = "yxb";//�û���
	data.password.cstring = "123456";//����
	/*��������*/
	data.willFlag = 1;
	data.will.topicName.cstring = iotdev->pubTopicName;
	data.will.message.cstring = "{\"cmd\":400,\"msg\":\"offline\"}";
    
	if(iotdev->mqttSock <= 0)
	{
		WARN_PRINT(("mysock error,mysock = %d\r\n",iotdev->mqttSock));
		//return mysock;
	}
    else
    {
        DBG_MQTT_Print("����Server[%d.%d.%d.%d][%d],Socket descriptor:%d\r\n",\
			server_addr.sin_addr.s_b1, server_addr.sin_addr.s_b2,\
            server_addr.sin_addr.s_b3, server_addr.sin_addr.s_b4,\
            htons(server_addr.sin_port),\
            iotdev->mqttSock);
    }
    
	len = MQTTSerialize_connect(buf, buflen, &data);
	
	//���������ݵķ���
	rc = transport_sendPacketBuffer(buf, len);
	
	/* �ȴ�connack *///���ͺ���շ��������ص����ݣ�����ʹ����һ��������ָ�룬Ҫ�������ָ��
    if (MQTTPacket_read(buf, buflen, transport_getdata) == CONNACK)//CONNACK �C ȷ����������20s��ʱ
    {
        unsigned char sessionPresent, connack_rc;

        if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen) != 1 || connack_rc != 0)
        {
            DBG_MQTT_Print("Unable to connect, return code %d\n", connack_rc);
            //goto exit;
        }
        else
        {
            ret = 0;
            iotdev->bMqttConnect = TRUE;
        }
	}
	else
	{
		DBG_MQTT_Print("δ���յ�MQTT��������Ӧ\r\n");
	}	

    return ret;
}


/*****************************************************************************
 �� �� ��  : deviceSubscribe
 ��������  : MQTT������Ϣ����
 �������  : ��
 �������  : ��
 �� �� ֵ  : ret
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
int deviceSubscribe(IOT_Device_t* iotDev)
{
    /*��Ϣ����*/
    unsigned char buf[200];
    int buflen = sizeof(buf);
	MQTTString topicString = MQTTString_initializer;
	int len = 0;
	int msgid = 1;
	int req_qos = 0;//QOS
	int rc = 0;
    int ret = -1;

    topicString.cstring = iotDev->subTopicName;
	
    len = MQTTSerialize_subscribe(buf, buflen, 0, msgid, 1, &topicString, &req_qos);

    rc = transport_sendPacketBuffer(buf, len);
    if (MQTTPacket_read(buf, buflen, transport_getdata) == SUBACK) 	/* wait for suback */
    {
        unsigned short submsgid;
        int subcount;
        int granted_qos;

        rc = MQTTDeserialize_suback(&submsgid, 1, &subcount, &granted_qos, buf, buflen);
        if (granted_qos != 0)
        {
            DBG_MQTT_Print("granted qos != 0, %d\r\n", granted_qos);
        }

        iotDev->bMqttSub = TRUE;
        ret = 0;
    }
    else
    {
        DBG_MQTT_Print("not wait for suback!!!\r\n");
    }

    return ret;
}


/*****************************************************************************
 �� �� ��  : iot_set_server_addr
 ��������  : MQTT���÷�����IP�Ͷ˿ں���
 �������  : U8 *ip, U16 port
 �������  : ��
 �� �� ֵ  : TRUE
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
static U8 iot_set_server_addr(IOT_Device_t* iotDev, U8 *ip, U16 port)
{
    memcpy(iotDev->net.un32Ip.u8Val, ip, 4);
    iotDev->net.Port = port;
    NOTE_PRINT(("��������ַ���Ϊ:%d.%d.%d.%d[%d]\r\n",
                                iotDev->net.un32Ip.u8Val[0],
                                iotDev->net.un32Ip.u8Val[1],
                                iotDev->net.un32Ip.u8Val[2],
                                iotDev->net.un32Ip.u8Val[3],
                                iotDev->net.Port));
    return TRUE;
}

/*****************************************************************************
 �� �� ��  : iotclient_publish
 ��������  : MQTT����/������Ϣ����
 �������  : const char* topicname, const char* payload, int payloadlen
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
int iotclient_publish(IOT_Device_t* iotDev, const char* payload, int payloadlen)
{
	int rc = 0;
	int len = 0;
    MQTTString topicString = MQTTString_initializer;
    topicString.cstring = (char*)iotDev->pubTopicName;
	
	if(iotDev->mqttSock>0)
	{
		len = MQTTSerialize_publish(mqttbuf, sizeof(mqttbuf), 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
        if(len>0)
        {
            rc = transport_sendPacketBuffer(mqttbuf, len);
            if(rc>0)
            {
            /*
                DBG_MQTT_Print("send %d bytes,data:", len);
                for(int i=0;i<len;i++)
                {
                    DBG_MQTT_Print("%02X ",mqttbuf[i]);
                }
                DBG_MQTT_Print("\r\n");
                */
                DBG_MQTT_Print("send Packet Buffer success\r\n");
                return 0;
            }
            else
            {
                DBG_MQTT_Print("send Packet error!!!,err=%d\r\n",rc);
                iotDev->bMqttConnect = FALSE;
                iotDev->sta = IOT_STA_INIT;
                return -1;
            } 
        }
        else
        {
            DBG_MQTT_Print(("MQTTSerialize publish is error!!!,err=%d\r\n",len));
            return -2;
        }
	}
	else
	{
		return -3;
	}
}

/*****************************************************************************
 �� �� ��  : iotclient_susb_resum
 ��������  : MQTT�����̹߳�����ָ�����
 �������  : U8 flag
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��23��
    ��    ��   : zxf
    �޸�����   : ����

*****************************************************************************/
static void iotclient_susb_resum(U8 flag)
{
	osStatus_t status=0;

	if(flag == FALSE)
	{
		status = osThreadSuspend(thread_iotclient_rcv_id);
		
		if(status == osOK)
		{
			DBG_MQTT_Print(("Stop MQTT rcv OK!\r\n"));
		}
		else
		{
			WARN_PRINT(("Stop MQTT rcv fail, osThreadSuspend return %d\r\n", status));
		}
	}
	else if(flag == TRUE)
	{
		status = osThreadResume(thread_iotclient_rcv_id);
		if(status == osOK)
		{
			DBG_MQTT_Print(("Resume MQTT rcv OK!\r\n"));
		}
		else
		{
			WARN_PRINT(("Resume MQTT rcv fail, osThreadResume return %d\r\n", status));
		}
	}
	delay_ms(1000);

}

/*****************************************************************************
 �� �� ��  : _iotclient_rcv_thread
 ��������  : MQTT��������������߳�
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
static void _iotclient_rcv_thread(void* arg)
{
    int msgtype = 0;
    
    int buflen = sizeof(rcvbuf);
    
    unsigned char dup;
    int qos;
    unsigned char retained;
    unsigned short msgid;
    int payloadlen_in;
    unsigned char* payload_in;
	static char Rcv_topic[50] = {0};
    int rc;
	int str_len=0;
    MQTTString receivedTopic;
	
	static U8 flag=0;

    while(1)
    {
        if(iotDev.sta==IOT_STA_IDEL || iotRTCMDev.sta==IOT_STA_IDEL || iotCmdDev.sta==IOT_STA_IDEL)
        {
            GLOBAL_MEMSET(rcvbuf,0,sizeof(rcvbuf));
            msgtype = MQTTPacket_read(rcvbuf, buflen, transport_getdata);

            if(msgtype!=-1)
            {
                switch (msgtype)
                {
                    case PUBLISH:
                        rc = MQTTDeserialize_publish(&dup, &qos, &retained, &msgid, &receivedTopic,\
                                                &payload_in, &payloadlen_in, rcvbuf, buflen);
						GLOBAL_MEMSET(Rcv_topic,0x0,50);
						GLOBAL_MEMSET(dedata,0x0,sizeof(dedata));
						GLOBAL_MEMCPY(Rcv_topic,receivedTopic.lenstring.data,receivedTopic.lenstring.len);
						if((strcmp(Rcv_topic,iotRTCMDev.subTopicName) == 0) && RTCM_SWITCH == MQTT_GET_RTCM)
						{
							str_len = base64_decode(payload_in,dedata);
							DBG_MQTT_Print("iotRTCMDev���ճɹ�!!! ���ݳ��� = %d\r\n",str_len);
							
							//MXT906Bģ��RTCM���뱣��
							if(str_len >20 && GGA_OLD_FLAG == FALSE)
								Ntrip_RTCM_to_UBX(dedata, str_len);						
						}
						else if(strcmp(Rcv_topic,iotDev.subTopicName) == 0)
						{
							DBG_MQTT_Print("iotDev��Ϣ���ճɹ�!!! %.*s\r\n", payloadlen_in, payload_in);
	                        parse_server_data(payload_in,payloadlen_in);
						}
						else if(strcmp(Rcv_topic,iotCmdDev.subTopicName) == 0)
						{
							DBG_MQTT_Print("iotCmdDev��Ϣ���ճɹ�!!! %.*s\r\n", payloadlen_in, payload_in);
							Cmd_parse_data(payload_in, payloadlen_in);
						}

                    break;
					
                    case PINGRESP:
                    case DISCONNECT:
                    default:
                        //GLOBAL_PRINT(("MQTTRcv_alive:\"%s\"\r\n",MQTTPacket_getName(msgtype)));
                        break;
                }  
            }
            else
            {
                delay_ms(100);
            }
			delay_ms(200);
			if(!flag){
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
				flag = TRUE;
			}else{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
				flag = FALSE;
			}
        }
        else
        {
            // WARN_PRINT(("rcv thread idle!!!\r\n"));
            delay_ms(1000);
        }
        
    }
}


/*****************************************************************************
 �� �� ��  : _iotclient_monitor_thread
 ��������  : MQTTͨ�ż���߳�
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����
  2.��    ��   : 2021��3��22��
    ��    ��   : zxf
    �޸�����   : ���MQTT�����̹߳�����ָ��߼�
*****************************************************************************/
static void _iotclient_monitor_thread(void* arg)
{
    static U8 retry_cnt = 0;
	static U8 flag = 0;

    while(1)
    {
        delay_ms(500);
		retry_cnt = 0;
		while(iotDev.sta != IOT_STA_IDEL)
		{	
			delay_ms(500);
	        switch (iotDev.sta)//��־��Ϣ���ĳ�ʼ��
	        {
	            case IOT_STA_INIT:
		
					MQTT_OFF_LINE = TRUE;
					iotclient_susb_resum(SUSBEND_RCV);
	                iotDev_reset(&iotDev);
	                iotDev.sta = IOT_STA_MQTTCONNECT;
	            
	            	break;

	            case IOT_STA_MQTTCONNECT:
	            
	                if(!iotDev.bMqttConnect)
	                {//δ���ӵ�MQTT������
	                    retry_cnt++;
	                    mqttDisconnect(&iotDev);
	                    connectMqttServer(&iotDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotDev.sta = IOT_STA_INIT;
	                        iotDev_reset(&iotDev);
	                        DBG_MQTT_Print("MQTT������iotDev����ʧ��!!!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotDev.sta = IOT_STA_MQTTSUB;
	                    NOTE_PRINT(("MQTT������iotDev���ӳɹ�!!!\r\n"));
	                }
	            	break;

	            case IOT_STA_MQTTSUB:
	                if(!iotDev.bMqttSub)
	                {//δ������Ϣ
	                    retry_cnt++;
	                    deviceSubscribe(&iotDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotDev.sta = IOT_STA_INIT;
	                        iotDev_reset(&iotDev);
	                        DBG_MQTT_Print("MQTT����iotDev��Ϣʧ��!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotDev.sta = IOT_STA_IDEL;
	                    NOTE_PRINT(("MQTT����iotDev��Ϣ�ɹ�!\r\n"));
	                }
	           		break;

				case IOT_STA_IDEL:
					break;
					
	            default:
	            	break;
	        }
		}
		
		retry_cnt=0;
		while(iotRTCMDev.sta != IOT_STA_IDEL)
		{
			delay_ms(500);
			switch(iotRTCMDev.sta)//RTCM��Ϣ���ĳ�ʼ��
			{
				case IOT_STA_INIT:
					iotRTCMDev.sta = IOT_STA_MQTTSUB;
					break;
				
				case IOT_STA_MQTTSUB:
	            
	                if(!iotRTCMDev.bMqttSub)
	                {//δ������Ϣ
	                    retry_cnt++;
	                    deviceSubscribe(&iotRTCMDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotRTCMDev.sta = IOT_STA_INIT;
	                        DBG_MQTT_Print("MQTT����iotRTCMDev��Ϣʧ��!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotRTCMDev.sta = IOT_STA_IDEL;
	                    NOTE_PRINT(("MQTT����iotRTCMDev��Ϣ�ɹ�!\r\n"));
	                }
	            	break;
				
				case IOT_STA_IDEL:
					break;
					
	            default:
	            	break;
			}
		}

		retry_cnt=0;
		while(iotCmdDev.sta != IOT_STA_IDEL)
		{
			delay_ms(500);
			switch(iotCmdDev.sta)//CMD��Ϣ���ĳ�ʼ��
			{
				case IOT_STA_INIT:
					iotCmdDev.bMqttSub = FALSE;
    				iotCmdDev.mqttSock = -1;
					iotCmdDev.sta = IOT_STA_MQTTCONNECT;
					
				break;

				case IOT_STA_MQTTCONNECT:
	            
	                if(!iotCmdDev.bMqttConnect)
	                {//δ���ӵ�MQTT������
	                    retry_cnt++;
	                    //mqttDisconnect(&iotCmdDev);
	                    connectMqttServer(&iotCmdDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotCmdDev.sta = IOT_STA_INIT;
	                        iotCmdDev.bMqttSub = FALSE;
    						iotCmdDev.mqttSock = -1;
	                        DBG_MQTT_Print("MQTT������iotCmdDev����ʧ��!!!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotCmdDev.sta = IOT_STA_MQTTSUB;
	                    NOTE_PRINT(("MQTT������iotCmdDev���ӳɹ�!!!\r\n"));
	                }
	            	break;
				
				case IOT_STA_MQTTSUB:
	            
	                if(!iotCmdDev.bMqttSub)
	                {//δ������Ϣ
	                    retry_cnt++;
	                    deviceSubscribe(&iotCmdDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotCmdDev.sta = IOT_STA_INIT;
	                        iotCmdDev.bMqttSub = FALSE;
    						iotCmdDev.mqttSock = -1;
	                        DBG_MQTT_Print("MQTT����iotCmdDev��Ϣʧ��!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotCmdDev.sta = IOT_STA_IDEL;
						iotclient_susb_resum(RESUME_RCV);
						MQTT_OFF_LINE = FALSE;
	                    NOTE_PRINT(("MQTT����iotCmdDev��Ϣ�ɹ�!\r\n"));
						NOTE_PRINT(("����MQTT��Ϣ���!!!\r\n"));
	                }
	            	break;
				
				case IOT_STA_IDEL:
					
					break;
					
	            default:
	            	break;
			}
		}
		
		if(!flag){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);
			flag = TRUE;
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
			flag = FALSE;
		}
    }
}

void MQTT_global_init(IOT_Device_t* iotDev)
{

	//ȫ�ֱ���iot��ʼ��
	iotDev->sta = IOT_STA_INIT;
	iotDev->bMqttConnect = FALSE;
	iotDev->bMqttSub = FALSE;
	iotDev->mqttSock = -1;
	GLOBAL_MEMSET(iotDev->clienId, 0x0, sizeof(iotDev->clienId));
	GLOBAL_MEMSET(iotDev->subTopicName, 0x0, sizeof(iotDev->subTopicName));
	GLOBAL_MEMSET(iotDev->pubTopicName, 0x0, sizeof(iotDev->pubTopicName));
	GLOBAL_MEMSET(&iotDev->net, 0x0, sizeof(iotDev->net));

	//�ͻ���ID����
    sprintf(iotDev->clienId,"uwb_hpr_%02X%02X%02X%02X",main_handle_g.p_monitor->dev_sn[4],
                                            main_handle_g.p_monitor->dev_sn[5],
                                            main_handle_g.p_monitor->dev_sn[6],
                                            main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("�ͻ���ID����[%s]\r\n",iotDev->clienId));

}


/*****************************************************************************
 �� �� ��  : iotclient_init
 ��������  : MQTT���̳߳�ʼ������
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��1��22��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
void iotclient_init(void)
{
    osThreadId_t thread_iotclient_monitor_id = 0;
    //osThreadId_t thread_iotclient_rcv_id = 0;

	MQTT_OFF_LINE = FALSE;//MQTT���߱�־
	RTCM_SWITCH = NTRIP_GET_RTCM;//Ĭ�Ͽ�����RTCM���ػ�վ��ȡRTCM���ݣ�(���԰汾)
	
	MQTT_global_init(&iotDev);
	MQTT_global_init(&iotRTCMDev);
	MQTT_global_init(&iotCmdDev);

//--------------------------------------����iotDev��Ϣ��������---------------------------------------------//

    sprintf(iotDev.subTopicName,"uwb_service/server_info/hpr/%02X%02X%02X%02X",\
    	main_handle_g.p_monitor->dev_sn[4],main_handle_g.p_monitor->dev_sn[5],\
    	main_handle_g.p_monitor->dev_sn[6],main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("������Ϣ��������[%s]\r\n",iotDev.subTopicName));


    //����iotDev��Ϣ��������
    sprintf(iotDev.pubTopicName,"uwb_service/client_info/hpr/%02X%02X%02X%02X",\
    	main_handle_g.p_monitor->dev_sn[4],main_handle_g.p_monitor->dev_sn[5],\
    	main_handle_g.p_monitor->dev_sn[6],main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("������Ϣ��������[%s]\r\n",iotDev.pubTopicName));

    iot_set_server_addr(&iotDev, main_handle_g.cfg.net.server_ip, main_handle_g.cfg.net.server_port);

//----------------------------------//--------------------- ------------//---------------------------------//	

//--------------------------------------����iotRTCMDev��Ϣ��������-----------------------------------------//

    sprintf(iotRTCMDev.subTopicName,"RTK_service/RTCM_3/hpr");
    GLOBAL_TRACE(("������Ϣ��������[%s]\r\n",iotRTCMDev.subTopicName));

	iot_set_server_addr(&iotRTCMDev, main_handle_g.cfg.net.server_ip, main_handle_g.cfg.net.server_port);

//----------------------------------//--------------------- ------------//---------------------------------//


//--------------------------------------����iotCmdDev��Ϣ��������------------------------------------------//

    sprintf(iotCmdDev.subTopicName,"uwb_service/server_info/hpr/cmd/%02X%02X%02X%02X",\
    	main_handle_g.p_monitor->dev_sn[4],main_handle_g.p_monitor->dev_sn[5],\
    	main_handle_g.p_monitor->dev_sn[6],main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("������Ϣ��������[%s]\r\n",iotCmdDev.subTopicName));


    //����iotCmdDev��Ϣ��������
    sprintf(iotCmdDev.pubTopicName,"uwb_service/client_info/hpr/cmd/%02X%02X%02X%02X",\
    	main_handle_g.p_monitor->dev_sn[4],main_handle_g.p_monitor->dev_sn[5],\
    	main_handle_g.p_monitor->dev_sn[6],main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("������Ϣ��������[%s]\r\n",iotCmdDev.pubTopicName));

    iot_set_server_addr(&iotCmdDev, main_handle_g.cfg.net.server_ip, main_handle_g.cfg.net.server_port);
	
//----------------------------------//--------------------- ------------//---------------------------------//


    thread_iotclient_monitor_id = osThreadNew(_iotclient_monitor_thread, NULL, &thread_iotclient_monitor_attr);
    GLOBAL_HEX(thread_iotclient_monitor_id);
#if 0
    thread_iotclient_period_id = osThreadNew(_iotclient_period_thread, NULL, &thread_iotclient_period_attr);
    GLOBAL_HEX(thread_iotclient_period_id);
#endif
    thread_iotclient_rcv_id = osThreadNew(_iotclient_rcv_thread, NULL, &thread_iotclient_rcv_attr);
    GLOBAL_HEX(thread_iotclient_rcv_id);
}
