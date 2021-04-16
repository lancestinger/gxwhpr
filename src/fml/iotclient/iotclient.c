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
 函 数 名  : mqttDisconnect
 功能描述  : 断开MQTT连接函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
/*断开MQTT连接*/
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
        DBG_MQTT_Print("关闭SOCKET[%d]\r\n",iotDev->mqttSock);
        transport_close(iotDev->mqttSock);
    }    
}


/*****************************************************************************
 函 数 名  : iotDev_reset
 功能描述  : MQTT连接reset函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建

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
 函 数 名  : connectMqttServer
 功能描述  : 连接MQTT服务器函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : ret
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建
  2.日    期   : 2021年3月16日
    作    者   : zxf
    修改内容   : 添加MQTT遗言设置

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
                                                                                                                                                                     
    data.clientID.cstring = iotdev->clienId;//客户端ID
	data.keepAliveInterval = 20;//设置心跳包间隔时间
	data.cleansession = 1;//清除会话
	data.username.cstring = "yxb";//用户名
	data.password.cstring = "123456";//密码
	/*遗嘱设置*/
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
        DBG_MQTT_Print("连接Server[%d.%d.%d.%d][%d],Socket descriptor:%d\r\n",\
			server_addr.sin_addr.s_b1, server_addr.sin_addr.s_b2,\
            server_addr.sin_addr.s_b3, server_addr.sin_addr.s_b4,\
            htons(server_addr.sin_port),\
            iotdev->mqttSock);
    }
    
	len = MQTTSerialize_connect(buf, buflen, &data);
	
	//现在是数据的发送
	rc = transport_sendPacketBuffer(buf, len);
	
	/* 等待connack *///发送后接收服务器返回的数据，这里使用了一个函数的指针，要定义这个指针
    if (MQTTPacket_read(buf, buflen, transport_getdata) == CONNACK)//CONNACK – 确认连接请求20s超时
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
		DBG_MQTT_Print("未接收到MQTT服务器响应\r\n");
	}	

    return ret;
}


/*****************************************************************************
 函 数 名  : deviceSubscribe
 功能描述  : MQTT订阅消息函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : ret
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
int deviceSubscribe(IOT_Device_t* iotDev)
{
    /*消息订阅*/
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
 函 数 名  : iot_set_server_addr
 功能描述  : MQTT设置服务器IP和端口函数
 输入参数  : U8 *ip, U16 port
 输出参数  : 无
 返 回 值  : TRUE
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
static U8 iot_set_server_addr(IOT_Device_t* iotDev, U8 *ip, U16 port)
{
    memcpy(iotDev->net.un32Ip.u8Val, ip, 4);
    iotDev->net.Port = port;
    NOTE_PRINT(("服务器地址变更为:%d.%d.%d.%d[%d]\r\n",
                                iotDev->net.un32Ip.u8Val[0],
                                iotDev->net.un32Ip.u8Val[1],
                                iotDev->net.un32Ip.u8Val[2],
                                iotDev->net.un32Ip.u8Val[3],
                                iotDev->net.Port));
    return TRUE;
}

/*****************************************************************************
 函 数 名  : iotclient_publish
 功能描述  : MQTT推送/发布消息函数
 输入参数  : const char* topicname, const char* payload, int payloadlen
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建

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
 函 数 名  : iotclient_susb_resum
 功能描述  : MQTT接收线程挂起与恢复函数
 输入参数  : U8 flag
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月23日
    作    者   : zxf
    修改内容   : 创建

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
 函 数 名  : _iotclient_rcv_thread
 功能描述  : MQTT数据与命令接收线程
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建

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
							DBG_MQTT_Print("iotRTCMDev接收成功!!! 数据长度 = %d\r\n",str_len);
							
							//MXT906B模块RTCM输入保护
							if(str_len >20 && GGA_OLD_FLAG == FALSE)
								Ntrip_RTCM_to_UBX(dedata, str_len);						
						}
						else if(strcmp(Rcv_topic,iotDev.subTopicName) == 0)
						{
							DBG_MQTT_Print("iotDev消息接收成功!!! %.*s\r\n", payloadlen_in, payload_in);
	                        parse_server_data(payload_in,payloadlen_in);
						}
						else if(strcmp(Rcv_topic,iotCmdDev.subTopicName) == 0)
						{
							DBG_MQTT_Print("iotCmdDev消息接收成功!!! %.*s\r\n", payloadlen_in, payload_in);
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
 函 数 名  : _iotclient_monitor_thread
 功能描述  : MQTT通信监控线程
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建
  2.日    期   : 2021年3月22日
    作    者   : zxf
    修改内容   : 添加MQTT接收线程挂起与恢复逻辑
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
	        switch (iotDev.sta)//日志消息订阅初始化
	        {
	            case IOT_STA_INIT:
		
					MQTT_OFF_LINE = TRUE;
					iotclient_susb_resum(SUSBEND_RCV);
	                iotDev_reset(&iotDev);
	                iotDev.sta = IOT_STA_MQTTCONNECT;
	            
	            	break;

	            case IOT_STA_MQTTCONNECT:
	            
	                if(!iotDev.bMqttConnect)
	                {//未连接到MQTT服务器
	                    retry_cnt++;
	                    mqttDisconnect(&iotDev);
	                    connectMqttServer(&iotDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotDev.sta = IOT_STA_INIT;
	                        iotDev_reset(&iotDev);
	                        DBG_MQTT_Print("MQTT服务器iotDev连接失败!!!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotDev.sta = IOT_STA_MQTTSUB;
	                    NOTE_PRINT(("MQTT服务器iotDev连接成功!!!\r\n"));
	                }
	            	break;

	            case IOT_STA_MQTTSUB:
	                if(!iotDev.bMqttSub)
	                {//未订阅消息
	                    retry_cnt++;
	                    deviceSubscribe(&iotDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotDev.sta = IOT_STA_INIT;
	                        iotDev_reset(&iotDev);
	                        DBG_MQTT_Print("MQTT订阅iotDev消息失败!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotDev.sta = IOT_STA_IDEL;
	                    NOTE_PRINT(("MQTT订阅iotDev消息成功!\r\n"));
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
			switch(iotRTCMDev.sta)//RTCM消息订阅初始化
			{
				case IOT_STA_INIT:
					iotRTCMDev.sta = IOT_STA_MQTTSUB;
					break;
				
				case IOT_STA_MQTTSUB:
	            
	                if(!iotRTCMDev.bMqttSub)
	                {//未订阅消息
	                    retry_cnt++;
	                    deviceSubscribe(&iotRTCMDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotRTCMDev.sta = IOT_STA_INIT;
	                        DBG_MQTT_Print("MQTT订阅iotRTCMDev消息失败!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotRTCMDev.sta = IOT_STA_IDEL;
	                    NOTE_PRINT(("MQTT订阅iotRTCMDev消息成功!\r\n"));
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
			switch(iotCmdDev.sta)//CMD消息订阅初始化
			{
				case IOT_STA_INIT:
					iotCmdDev.bMqttSub = FALSE;
    				iotCmdDev.mqttSock = -1;
					iotCmdDev.sta = IOT_STA_MQTTCONNECT;
					
				break;

				case IOT_STA_MQTTCONNECT:
	            
	                if(!iotCmdDev.bMqttConnect)
	                {//未连接到MQTT服务器
	                    retry_cnt++;
	                    //mqttDisconnect(&iotCmdDev);
	                    connectMqttServer(&iotCmdDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotCmdDev.sta = IOT_STA_INIT;
	                        iotCmdDev.bMqttSub = FALSE;
    						iotCmdDev.mqttSock = -1;
	                        DBG_MQTT_Print("MQTT服务器iotCmdDev连接失败!!!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotCmdDev.sta = IOT_STA_MQTTSUB;
	                    NOTE_PRINT(("MQTT服务器iotCmdDev连接成功!!!\r\n"));
	                }
	            	break;
				
				case IOT_STA_MQTTSUB:
	            
	                if(!iotCmdDev.bMqttSub)
	                {//未订阅消息
	                    retry_cnt++;
	                    deviceSubscribe(&iotCmdDev);
	                    if(retry_cnt>3)
	                    {
	                        retry_cnt = 0;
	                        iotCmdDev.sta = IOT_STA_INIT;
	                        iotCmdDev.bMqttSub = FALSE;
    						iotCmdDev.mqttSock = -1;
	                        DBG_MQTT_Print("MQTT订阅iotCmdDev消息失败!\r\n");
	                    }
	                }
	                else
	                {
	                    retry_cnt = 0;
	                    iotCmdDev.sta = IOT_STA_IDEL;
						iotclient_susb_resum(RESUME_RCV);
						MQTT_OFF_LINE = FALSE;
	                    NOTE_PRINT(("MQTT订阅iotCmdDev消息成功!\r\n"));
						NOTE_PRINT(("订阅MQTT消息完成!!!\r\n"));
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

	//全局变量iot初始化
	iotDev->sta = IOT_STA_INIT;
	iotDev->bMqttConnect = FALSE;
	iotDev->bMqttSub = FALSE;
	iotDev->mqttSock = -1;
	GLOBAL_MEMSET(iotDev->clienId, 0x0, sizeof(iotDev->clienId));
	GLOBAL_MEMSET(iotDev->subTopicName, 0x0, sizeof(iotDev->subTopicName));
	GLOBAL_MEMSET(iotDev->pubTopicName, 0x0, sizeof(iotDev->pubTopicName));
	GLOBAL_MEMSET(&iotDev->net, 0x0, sizeof(iotDev->net));

	//客户端ID配置
    sprintf(iotDev->clienId,"uwb_hpr_%02X%02X%02X%02X",main_handle_g.p_monitor->dev_sn[4],
                                            main_handle_g.p_monitor->dev_sn[5],
                                            main_handle_g.p_monitor->dev_sn[6],
                                            main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("客户端ID配置[%s]\r\n",iotDev->clienId));

}


/*****************************************************************************
 函 数 名  : iotclient_init
 功能描述  : MQTT各线程初始化函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月22日
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
void iotclient_init(void)
{
    osThreadId_t thread_iotclient_monitor_id = 0;
    //osThreadId_t thread_iotclient_rcv_id = 0;

	MQTT_OFF_LINE = FALSE;//MQTT上线标志
	RTCM_SWITCH = NTRIP_GET_RTCM;//默认开机从RTCM本地基站获取RTCM数据，(测试版本)
	
	MQTT_global_init(&iotDev);
	MQTT_global_init(&iotRTCMDev);
	MQTT_global_init(&iotCmdDev);

//--------------------------------------订阅iotDev消息主题配置---------------------------------------------//

    sprintf(iotDev.subTopicName,"uwb_service/server_info/hpr/%02X%02X%02X%02X",\
    	main_handle_g.p_monitor->dev_sn[4],main_handle_g.p_monitor->dev_sn[5],\
    	main_handle_g.p_monitor->dev_sn[6],main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("订阅消息主题配置[%s]\r\n",iotDev.subTopicName));


    //推送iotDev消息主题配置
    sprintf(iotDev.pubTopicName,"uwb_service/client_info/hpr/%02X%02X%02X%02X",\
    	main_handle_g.p_monitor->dev_sn[4],main_handle_g.p_monitor->dev_sn[5],\
    	main_handle_g.p_monitor->dev_sn[6],main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("推送消息主题配置[%s]\r\n",iotDev.pubTopicName));

    iot_set_server_addr(&iotDev, main_handle_g.cfg.net.server_ip, main_handle_g.cfg.net.server_port);

//----------------------------------//--------------------- ------------//---------------------------------//	

//--------------------------------------订阅iotRTCMDev消息主题配置-----------------------------------------//

    sprintf(iotRTCMDev.subTopicName,"RTK_service/RTCM_3/hpr");
    GLOBAL_TRACE(("订阅消息主题配置[%s]\r\n",iotRTCMDev.subTopicName));

	iot_set_server_addr(&iotRTCMDev, main_handle_g.cfg.net.server_ip, main_handle_g.cfg.net.server_port);

//----------------------------------//--------------------- ------------//---------------------------------//


//--------------------------------------订阅iotCmdDev消息主题配置------------------------------------------//

    sprintf(iotCmdDev.subTopicName,"uwb_service/server_info/hpr/cmd/%02X%02X%02X%02X",\
    	main_handle_g.p_monitor->dev_sn[4],main_handle_g.p_monitor->dev_sn[5],\
    	main_handle_g.p_monitor->dev_sn[6],main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("订阅消息主题配置[%s]\r\n",iotCmdDev.subTopicName));


    //推送iotCmdDev消息主题配置
    sprintf(iotCmdDev.pubTopicName,"uwb_service/client_info/hpr/cmd/%02X%02X%02X%02X",\
    	main_handle_g.p_monitor->dev_sn[4],main_handle_g.p_monitor->dev_sn[5],\
    	main_handle_g.p_monitor->dev_sn[6],main_handle_g.p_monitor->dev_sn[7]);
    GLOBAL_TRACE(("推送消息主题配置[%s]\r\n",iotCmdDev.pubTopicName));

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
