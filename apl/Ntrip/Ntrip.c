#include <stdio.h>
#include "Ntrip.h"
#include "comm/project_def.h"
#include "fml/gnss/nmea.h"
#include "drv/socket/socket_drv.h"
#include "drv/json/cJSON.h"
#include "drv/json/Cjson_run.h"
#include "drv/uart/uart_drv.h"
#include "fml/gnss/NMEA.h"
#include "fml/iotclient/iotclient.h"

static U64 thread_Ntrip_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_Ntrip_attr = {
  .stack_mem  = &thread_Ntrip_stk[0],
  .stack_size = sizeof(thread_Ntrip_stk),
  .priority = osPriorityNormal,
};

static U64 thread_Ntrip_monitor_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_Ntrip_monitor_attr = {
  .stack_mem  = &thread_Ntrip_monitor_stk[0],
  .stack_size = sizeof(thread_Ntrip_monitor_stk),
  .priority = osPriorityBelowNormal6,
};

static U8 TCP_INIT_FAILED = FALSE;
static U8 NTRIP_INIT_FAILED = FALSE;

char Ntrip_Mount[] = MOUNT;	  //Ntrip默认挂载点
char Ntrip_user_code[] = USER_CODE;//Ntrip默认账号密码

osThreadId_t thread_Ntrip_id = 0;

NTRIP_WorkState_t Ntrip_State = NTRIP_INIT;

/*****************************************************************************
 函 数 名  : Ntrip_TCP_Init
 功能描述  : TCP连接千寻Ntrip服务器
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月23日
    作    者   : zxf
    修改内容   : 更新

*****************************************************************************/
void Ntrip_TCP_Init(void)
{
	if(socket_tcp_connect(SOCKET_2) != 0 )
	{
		TCP_INIT_FAILED = TRUE;
		delay_ms(1000);
		DBG_NTRIP_Print("Ntrip TCP connect Overtime!!!\r\n");
	}
	else
	{
		DBG_NTRIP_Print("Ntrip TCP Connect OK!!\r\n");
		TCP_INIT_FAILED = FALSE;
	}
}

/*****************************************************************************
 函 数 名  : Ntrip_QX_Connect
 功能描述  : 通过Ntrip连接获取千寻知寸服务
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月23日
    作    者   : zxf
    修改内容   : 更新

*****************************************************************************/
void Ntrip_QX_Connect(void)
{
	//static char *Ntrip_connect_buff = "GET /RTCM32_GGB HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic cXh1dG11MDAyOmUxMWE3OTc=\r\n\r\n";

	char* Ntrip_connect_buff = (char*)GLOBAL_MALLOC(NTRIP_CODE_LEN);
	GLOBAL_MEMSET(Ntrip_connect_buff,0x0,NTRIP_CODE_LEN);

	sprintf(Ntrip_connect_buff,"GET /%s HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic %s\r\n\r\n",\
		main_handle_g.cfg.net.Ntrip_mount,\
		main_handle_g.cfg.net.Ntrip_usr_pass);
	
	DBG_NTRIP_Print("Ntrip_connect_buff: \r\n%s\r\n",Ntrip_connect_buff);
	
	if(TCP_INIT_FAILED == FALSE)
	{
		if( 0 >= socket_TCP_send_msg(SOCKET_2,(U8*)Ntrip_connect_buff,strlen(Ntrip_connect_buff),0))
		{
			NTRIP_INIT_FAILED = TRUE;
			delay_ms(10);
			DBG_NTRIP_Print("Ntrip Caster connect Overtime!!\r\n");
		}
		else
		{
			NTRIP_INIT_FAILED = FALSE;
			DBG_NTRIP_Print("Ntrip Caster connect OK!!\r\n");
		}
	}
	free(Ntrip_connect_buff);
}

/*****************************************************************************
 函 数 名  : Ntrip_RTCM_to_UBX
 功能描述  : RTCM数据传输到UBX或MXT模块
 输入参数  : uint8_t *buf, int total_len  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月23日
    作    者   : zxf
    修改内容   : 更新

*****************************************************************************/
void Ntrip_RTCM_to_UBX(uint8_t *buf, int total_len )
{
	int i,j,k=0,m=0,D3_pos=0,D3_pos_next=0,D3_TOTAL_NUM=0;
	int pause_pos[10] = {0};
	unsigned short Send_len[10]={0};
		
	i=0;
	j=0;

	while(i<total_len)
	{
		if(buf[i] == 0xd3 && buf[i+1] == 0x00)
		{
			pause_pos[j] = i;
			Send_len[j] = buf[i+2]+6;
			//if(buf[i+2] == 0x13 || buf[i+2] == 0x15)
			//	RTCM_RTK_flag = 1;

			//DBG_PRINT("Send_len = %d\r\n",Send_len[j]);
			/*
			for(k=0;k<Send_len[j];k++)
			{
				if(k%16 ==0 )
				GLOBAL_PRINT("\r\n%04x:",k);
				GLOBAL_PRINT("%02x ",buf[i+k]);
			}
			GLOBAL_PRINT("\r\n");
			*/
			j++;
		}
		i++;
		if((i==total_len)&&(j==0))
		{
			DBG_NTRIP_Print("RTCM dont have Head!!\r\n");
			return 1;
		}	
	}

	D3_TOTAL_NUM = j;
	for(m=0;m<D3_TOTAL_NUM;m++)
	{
		//NOTE_PRINT(("RTCM Sending[%d]!!\r\n",m));
		//DMA_Data_send(UART_COM3,&buf[pause_pos[m]],(S32)Send_len[m]);
		uart_send_data(UART_COM4,&buf[pause_pos[m]],(S32)Send_len[m]);//uart4传输
	}
	DBG_NTRIP_Print("RTCM数据传输完成!!!\r\n");	
}


/*****************************************************************************
 函 数 名  : _Ntrip_thread
 功能描述  : 基于Ntrip+TCP的RTCM数据(RTK)收发线程
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月23日
    作    者   : zxf
    修改内容   : 更新

*****************************************************************************/
static void _Ntrip_thread(void* arg)
{
	int ret = 0;
	static U16 Over_time_send = 0, Over_time_rcv=0;
	static uint8_t First_round;
	static U8 RTCM_DATA_ERROR=0;
	//static U8* Heart_b = "$";
	//static U32 last_tick = 0;
	static U32 tick = 0;
	
//---------------------------------------Wait for NMEA------------------------------------------//	

	while(1)
	{
		while(Ntrip_State != NTRIP_IDEL)
		{
			First_round = TRUE;
			DBG_NTRIP_Print("Ntrip Restarting!!\r\n");
			delay_ms(1000);
		}
		
		if(First_round == TRUE)
		{
			First_round = FALSE;
			while(CacheBuff.RX_flag != TRUE)//waiting for first GGA//CacheBuff.RX_flag != 1
			{
				DBG_NTRIP_Print("Waiting for First NMEA!!\r\n");
				delay_ms(500);
			}
		}
		
//-----------------------------------------NMEA Send--------------------------------------------//

		if(CacheBuff.RX_flag == TRUE || GGA_OLD_FLAG == TRUE)
		{
			tick = sysup_seconds_g;
			Over_time_send = 0;
			Over_time_rcv = 0;
			while(0>= socket_TCP_send_msg(SOCKET_2,CacheBuff.RX_pData,CacheBuff.RX_Size,MSG_DONTWAIT))
			{
				Over_time_send++;
				DBG_NTRIP_Print("Sending NMEA...\r\n");
				delay_ms(1000);
				if(Over_time_send>=3)
				{
					break;
				}
			}
			if(Over_time_send >= 3)
			{
				Over_time_send = 0;
				DBG_NTRIP_Print("Send NMEA error, Socket Restart!!\r\n");
				Ntrip_State = NTRIP_INIT;
				
				continue;
			}
			
//------------------------------------------RTCM Recv--------------------------------------------//		

			while(1)
			{
				ret = socket_rcv_msg(SOCKET_2,NULL,MSG_DONTWAIT,SocketBuff.RX_pData,RX_LEN);//MSG_DONTWAIT
				if(ret>0)
				{
					SocketBuff.RX_Size = ret;
					DBG_NTRIP_Print("RTCM data recv OK!!\r\nRTCM num = %d\r\n",SocketBuff.RX_Size);
					if(ret>20)
					{
						ret = 0;
						break;
					}
					else
					{
						RTCM_DATA_ERROR = 1;
						if(strcmp(SocketBuff.RX_pData,"ICY 200 OK")>=0)
						{
							DBG_NTRIP_Print("Ntrip连接成功!!\r\n");
						}
						else
						{
							DBG_NTRIP_Print("RTCM data too short!\r\n");
							DBG_NTRIP_Print("Rcv = %s\r\n",SocketBuff.RX_pData);
						}
						//Ntrip_State = NTRIP_INIT;
					}
				}
				Over_time_rcv++;
				delay_ms(100);
				if(Over_time_rcv>=25)
				{
					Over_time_rcv = 0;
					RTCM_DATA_ERROR = 1;
					DBG_NTRIP_Print("RTCM rcv OVER TIME!!\r\n");
					
					break;
				}
			}
			
			CacheBuff.RX_flag = FALSE;
			
//--------------------------------------------RTCM send-------------------------------------------------//

#if 1
			//MXT906B模块RTCM输入保护
			if(RTCM_DATA_ERROR == 0 && GGA_OLD_FLAG == FALSE && RTCM_SWITCH == NTRIP_GET_RTCM)
				Ntrip_RTCM_to_UBX(SocketBuff.RX_pData, SocketBuff.RX_Size);
			else
				RTCM_DATA_ERROR = 0;
#endif		
			GLOBAL_MEMSET(SocketBuff.RX_pData,0x0, SocketBuff.RX_Size);
			delay_ms(5);
			
		}	
		else
		{
			delay_ms(10);//NMEA未获取时，线程空转
		}

		if(GGA_OLD_FLAG == TRUE)//1秒定时器，用于NMEA未连续接收时的旧消息发送周期定时
		{		
			while(sysup_seconds_g - tick < 1)
			{
				delay_ms(5);
			}
			tick = 0;
		}	

//------------------------------------------RTCM Source change-----------------------------------------//

		if(RTCM_SWITCH == MQTT_GET_RTCM)
		{
			socket_tcp_disconnect(SOCKET_2);
			while(RTCM_SWITCH == MQTT_GET_RTCM)
			{
				delay_ms(300);
			}
			Ntrip_State = NTRIP_INIT;
		}
		
//-----------------------------------------------------------------------------------------------------//		
	}


}

/*****************************************************************************
 函 数 名  : _Ntrip_monitor
 功能描述  : Ntrip+TCP收发(RTK)监控线程
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月23日
    作    者   : zxf
    修改内容   : 更新

*****************************************************************************/

static void _Ntrip_monitor(void* arg)
{
	static U8 retry_cnt = 0;
	static U8 flag=0;
	static osStatus_t status=0;

	while(1)
	{	
		switch(Ntrip_State)
		{
			case NTRIP_INIT:
				
				Ntrip_TCP_Init();//TCP连接初始化
				
				if(TCP_INIT_FAILED)
					Ntrip_State = NTRIP_INIT;
				else
					Ntrip_State = NTRIP_CONNECT;
				break;

			case NTRIP_CONNECT:
				
				Ntrip_QX_Connect();//千寻RTK连接
				
				if(NTRIP_INIT_FAILED)
				{
					retry_cnt++;
					Ntrip_State = NTRIP_CONNECT;
					if(retry_cnt > 3)
					{
						retry_cnt = 0;
						Ntrip_State = NTRIP_INIT;
					}
				}
				else
				{
					retry_cnt = 0;
					Ntrip_State = NTRIP_IDEL;
				}
				break;

			case NTRIP_IDEL:
				if(RTCM_SWITCH == NTRIP_GET_RTCM)
				{
					//空闲LED闪烁
					if(!flag){
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
						flag = TRUE;
					}else{
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
						flag = FALSE;
					}
				}
				break;

			default:
				break;
		}
		delay_ms(500);
	}
}

/*****************************************************************************
 函 数 名  : Ntrip_apl_Init
 功能描述  : Ntrip+TCP各线程初始化函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月23日
    作    者   : zxf
    修改内容   : 更新

*****************************************************************************/
void Ntrip_apl_Init(void)
{
	
	osThreadId_t thread_Ntrip_monitor_id = 0;

	thread_Ntrip_monitor_id = osThreadNew(_Ntrip_monitor, NULL, &thread_Ntrip_monitor_attr);
	GLOBAL_HEX(thread_Ntrip_monitor_id);

	thread_Ntrip_id = osThreadNew(_Ntrip_thread, NULL, &thread_Ntrip_attr);
	GLOBAL_HEX(thread_Ntrip_id);

#if 0
	//测试版本，默认挂起 thread_Ntrip
	osStatus_t status = osThreadSuspend(thread_Ntrip_id);
	if(status == osOK)
	{
		GLOBAL_PRINT(("Stop Ntrip is OK!\r\n"));
		RTCM_SWITCH = MQTT_GET_RTCM;
		return TRUE;
	}
	else
	{
		GLOBAL_PRINT(("stop Ntrip is fail,osThreadSuspend return %d\r\n", status));
	}
#endif
}


