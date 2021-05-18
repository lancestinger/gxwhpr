/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : socket_drv.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年7月26日
  最近修改   :
  功能描述   : socket驱动程序
  函数列表   :
  修改历史   :
  1.日    期   : 2019年7月26日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


/*------------------------------头文件------------------------------*/
#include "socket/socket_drv.h"
#include "drv/uart/uart_drv.h"
#include "project_def.h"
#include "sys_debug.h"

/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/

/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
static SOCKADDR_IN socket_handle[SOCKET_NUM];
static U8 tcp_addr_port[4]={0};
static U8 MS_ID_cnt =0;
static socket_drv_t socket_drv_cfg[SOCKET_NUM] = 
{
	/* socket1 */
	{
		0,
		TRUE,
		&socket_handle[SOCKET_1],
		7000,
	},

	/* socket2 */
	{
		0,
		TRUE,
		&socket_handle[SOCKET_2],
		8001,
	},

	/* socket3 */
	{
		0,
		FALSE,
		&socket_handle[SOCKET_3],
		9099,
	},

};
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/

/*****************************************************************************
 函 数 名  : socket_send_msg
 功能描述  : socket发送数据
 输入参数  : socket_type_enum type    
             SOCKADDR_IN* dst_socket  
             U8* buf                  
             U32 len                  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月26日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
S32 socket_TCP_send_msg(socket_type_enum type, U8* buf, U32 len, IN S32 flag)
{
	S32 ret = 0, i = 0;
	if(socket_drv_cfg[type].tcp_enable == TRUE)
	{
		ret = send(socket_drv_cfg[type].socket_fd, (CHAR_T*)buf, len,flag);
		if(ret < 0)
		{
			DBG_Socket_Print("TCP(%d)发送数据失败!!!!!!!\r\n", socket_drv_cfg[type].local_port); 
			DBG_Socket_Print("TCP Send ret = %d\r\n",ret);
		}
		else
		{
			DBG_Socket_Print("TCP(%d)发送数据成功!!\r\n", socket_drv_cfg[type].local_port);
		}
		return ret;
	}
}


S32 socket_UDP_send_msg(socket_type_enum type, IN SOCKADDR_IN* dst_socket, U8* buf, U32 len)
{
	S32 ret = 0, i = 0;
	static int over_time = 0;
	
	if(socket_drv_cfg[type].tcp_enable == FALSE)
	{
		ret = sendto(socket_drv_cfg[type].socket_fd, (CHAR_T*)buf, len, MSG_DONTWAIT,(SOCKADDR *)dst_socket, sizeof(SOCKADDR_IN));//MSG_DONTWAIT
		DBG_Socket_Print("\r\nUDP发送数据[len:%u][IP:%u.%u.%u.%u][port:%u]: ", len, dst_socket->sin_addr.s_b1,dst_socket->sin_addr.s_b2,dst_socket->sin_addr.s_b3,dst_socket->sin_addr.s_b4,htons(dst_socket->sin_port));

		if(ret < 0)
		{
			over_time++;
			DBG_Socket_Print("UDP(%d)发送数据[len:%u][IP:%u.%u.%u.%u][port:%u]失败,错误码:%d!!!!!!!\r\n", socket_drv_cfg[type].local_port,len, dst_socket->sin_addr.s_b1,dst_socket->sin_addr.s_b2,dst_socket->sin_addr.s_b3,dst_socket->sin_addr.s_b4,htons(dst_socket->sin_port),ret);
		}
		else
		{
			DBG_Socket_Print("UDP发送成功!!返回码:%d\r\n",ret);
		}
	}
}

void udp_set_dst(SOCKADDR_IN* udp_dst)
{
	udp_dst->sin_family = PF_INET;
	udp_dst->sin_addr.s_b1 = main_handle_g.cfg.net.MS_ip[0];
	udp_dst->sin_addr.s_b2 = main_handle_g.cfg.net.MS_ip[1];
	udp_dst->sin_addr.s_b3 = main_handle_g.cfg.net.MS_ip[2];
	udp_dst->sin_addr.s_b4 = main_handle_g.cfg.net.MS_ip[3];
	udp_dst->sin_port = htons(main_handle_g.cfg.net.MS_port);
}


/*****************************************************************************
 函 数 名  : socket_send_broardcast_msg
 功能描述  : 发送广播消息
 输入参数  : socket_type_enum type  
             U8* buf                
             U32 len                
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月4日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void socket_send_broardcast_msg(socket_type_enum type, U16 port, U8* buf, U32 len)
{
	S32 ret = 0, i = 0;
	SOCKADDR_IN dst_socket;

	dst_socket.sin_port = htons(port);
	dst_socket.sin_family = AF_INET;
	dst_socket.sin_addr.s_addr = 0xFFFFFFFF;
	if(socket_drv_cfg[type].tcp_enable == FALSE)
	{
		ret = sendto(socket_drv_cfg[type].socket_fd, (CHAR_T*)buf, len, MSG_DONTWAIT,(SOCKADDR *)&dst_socket, sizeof(SOCKADDR_IN));
		DBG_Socket_Print("\r\n广播发送数据[len:%u][port:%u]: ", len, port);
		for(i = 0; i < len; i++)
		{
			DBG_Socket_Print("%02X ", buf[i]);
		}
		DBG_Socket_Print("\r\n");
		if(ret < 0)
		{
			DBG_Socket_Print("UDP广播(%d)发送数据失败,错误码:%d!!!!!!!\r\n", socket_drv_cfg[type].local_port,ret);
		}
	}
}


/*****************************************************************************
 函 数 名  : socket_rcv_msg
 功能描述  : socket接收数据
 输入参数  : socket_type_enum type          
             INOUT SOCKADDR_IN* src_socket  
             S32 flags                      
             INOUT U8* rcv_buf              
             U32 buf_len                    
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月26日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
S32 socket_rcv_msg(socket_type_enum type, INOUT SOCKADDR_IN* src_socket, S32 flags, INOUT U8* rcv_buf, U32 buf_len)
{
	S32 ret = 0, i = 0;
	S32 len = sizeof(SOCKADDR_IN);
	
	if(socket_drv_cfg[type].tcp_enable == FALSE)
	{
		ret = recvfrom(socket_drv_cfg[type].socket_fd, (CHAR_T*)rcv_buf, buf_len, flags, (SOCKADDR *)src_socket, &len);
		if(ret > 0)
		{	
			MS_ID_cnt++;
			UDPBuff.RX_Size = ret;
			if(MS_ID_cnt>5)
			{
				UDPBuff.RX_flag = TRUE;
				MS_ID_cnt=0;
			}
			DBG_Socket_Print("\r\nUDP接收数据[sockt:%u][len:%u][port:%u]: ", type, ret, htons(src_socket->sin_port));
			/*
			for(i = 0; i < ret; i++)
			{
				DBG_Socket_Print("%02X ", rcv_buf[i]);
			}
			DBG_Socket_Print("\r\n");
			*/
		}
		else if(ret==-8)
	    {
	        DBG_Socket_Print("Waiting for MS_ID, UDP alive!\r\n");
	    }
	    else if(ret==-14)
	    {//BSD_ECONNRESET
	        DBG_Socket_Print("UDP Connection reset by the peer");
	    }
	    else
	    {
	        DBG_Socket_Print("UDP recv code=%d\r\n", ret);
	    }
	}
	else
	{
		ret = recv(socket_drv_cfg[type].socket_fd, (CHAR_T*)rcv_buf, buf_len, flags);
	}
	return ret;
}


S32 socket_tcp_connect(socket_type_enum type)
{
	int rc;
	int time_out=5000;
	U8 bDontLinger = FALSE;
	
	if(socket_drv_cfg[type].tcp_enable == TRUE)		//TCP Client 配置
	{
		if(socket_drv_cfg[type].socket_fd>0)
		{
			rc = closesocket (socket_drv_cfg[type].socket_fd);
			if(rc<0)
			{
				ERR_PRINT(("close socket failed!!!,sock=%d",socket_drv_cfg[type].socket_fd));
			}
			delay_ms(1100);
		}
		delay_ms(2);
		socket_drv_cfg[type].socket_fd = socket(AF_INET, SOCK_STREAM, 0);
		socket_drv_cfg[type].p_socket_handle->sin_family = PF_INET;
		
		
		if(type==SOCKET_NTRIP)
		{
			GLOBAL_MEMCPY(tcp_addr_port,main_handle_g.cfg.net.ntrip_ip,4);
			socket_drv_cfg[type].p_socket_handle->sin_port = htons(main_handle_g.cfg.net.ntrip_port);
		}
		else
		{   //后续可添加其他socket
			socket_drv_cfg[type].p_socket_handle->sin_port = htons(socket_drv_cfg[type].local_port);
		}
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_b1 = tcp_addr_port[0];
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_b2 = tcp_addr_port[1];
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_b3 = tcp_addr_port[2];
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_b4 = tcp_addr_port[3];
		if (connect(socket_drv_cfg[type].socket_fd, (SOCKADDR *)(socket_drv_cfg[type].p_socket_handle), sizeof(SOCKADDR)) < 0)
		{
			DBG_Socket_Print("socket(tcp-%u)连接[IP:%u.%u.%u.%u][port:%u]失败!\r\n", socket_drv_cfg[type].local_port,tcp_addr_port[0],tcp_addr_port[1],tcp_addr_port[2],tcp_addr_port[3],htons(socket_drv_cfg[type].local_port));
			return 1;
		}
		setsockopt(socket_drv_cfg[type].socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&time_out, sizeof (time_out));
		setsockopt(socket_drv_cfg[type].socket_fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&time_out, sizeof (time_out));

		DBG_Socket_Print("socket(tcp-%u)连接[IP:%u.%u.%u.%u][port:%u]成功!!!!!!\r\n", socket_drv_cfg[type].local_port,tcp_addr_port[0],tcp_addr_port[1],tcp_addr_port[2],tcp_addr_port[3],htons(socket_drv_cfg[type].local_port));
	}

	return 0;
	
}


S32 socket_tcp_disconnect(socket_type_enum type)
{
	int rc;

	if(socket_drv_cfg[type].tcp_enable == TRUE)		//TCP Client 配置
	{
		if(socket_drv_cfg[type].socket_fd>0)
		{
			rc = closesocket (socket_drv_cfg[type].socket_fd);
			if(rc<0)
			{
				DBG_Socket_Print("close socket failed!!!,sock=%d",socket_drv_cfg[type].socket_fd);
			}
			delay_ms(1100);
		}
	}
}



/*****************************************************************************
 函 数 名  : socket_drv_init
 功能描述  : socket创建
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月26日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void socket_udp_drv_init(socket_type_enum type)
{
	U8 i = 0;
	int rc;
	U8 bDontLinger = FALSE;

	
	if(socket_drv_cfg[type].tcp_enable == FALSE)		//UDP配置
	{
		if(socket_drv_cfg[type].socket_fd>0)
		{
			rc = closesocket (socket_drv_cfg[type].socket_fd);
			if(rc<0)
			{
				ERR_PRINT(("close socket failed!!!,sock=%d",socket_drv_cfg[type].socket_fd));
			}
			delay_ms(1100);
		}
		socket_drv_cfg[type].socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
		if(socket_drv_cfg[type].socket_fd < 0)
		{
			ERR_PRINT(("########################socket(udp-%u)创建失败!\r\n", socket_drv_cfg[type].local_port));
			return;
		}
		socket_drv_cfg[type].p_socket_handle->sin_family = AF_INET;
		if(type==SOCKET_MS)
		{
			socket_drv_cfg[type].p_socket_handle->sin_port = htons(main_handle_g.cfg.net.MS_port);
			UDPBuff.RX_flag = FALSE;
		}
		else
		{   //后续可添加其他socket
			socket_drv_cfg[type].p_socket_handle->sin_port = htons(socket_drv_cfg[type].local_port);
		}
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_addr = INADDR_ANY;
/*			
		socket_drv_cfg[i].p_socket_handle->sin_addr.s_b1 = 192;
		socket_drv_cfg[i].p_socket_handle->sin_addr.s_b2 = 168;
		socket_drv_cfg[i].p_socket_handle->sin_addr.s_b3 = 10;
		socket_drv_cfg[i].p_socket_handle->sin_addr.s_b4 = 235;
*/			
		if (bind(socket_drv_cfg[type].socket_fd, (SOCKADDR *)(socket_drv_cfg[type].p_socket_handle), sizeof(SOCKADDR)) < 0)
		{
			ERR_PRINT(("########################socket(udp-%u)绑定失败!\r\n", socket_drv_cfg[type].local_port));
			return;
		}

		
		NOTE_PRINT(("########################socket(udp-%u)创建成功!!!!!!\r\n", socket_drv_cfg[type].local_port));
		delay_ms(100);
	}

}

/*eof*/

