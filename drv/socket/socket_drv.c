/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : socket_drv.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��7��26��
  ����޸�   :
  ��������   : socket��������
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2019��7��26��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


/*------------------------------ͷ�ļ�------------------------------*/
#include "socket/socket_drv.h"
#include "sys_debug.h"

/*------------------------------ͷ�ļ�------------------------------*/


/*------------------------------�ļ���------------------------------*/

/*------------------------------�ļ���------------------------------*/


/*------------------------------�ļ�����------------------------------*/
static SOCKADDR_IN socket_handle[SOCKET_NUM];
static uint8_t tcp_addr_port[4]={203,107,45,154};//ǧѰCM������
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
/*------------------------------�ļ�����------------------------------*/


/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/

/*****************************************************************************
 �� �� ��  : socket_send_msg
 ��������  : socket��������
 �������  : socket_type_enum type    
             SOCKADDR_IN* dst_socket  
             U8* buf                  
             U32 len                  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��26��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
S32 socket_TCP_send_msg(socket_type_enum type, U8* buf, U32 len, IN S32 flag)
{
	S32 ret = 0, i = 0;
	if(socket_drv_cfg[type].tcp_enable == TRUE)
	{
		ret = send(socket_drv_cfg[type].socket_fd, (CHAR_T*)buf, len,flag);
		if(ret < 0)
		{
			DBG_Socket_Print(("TCP(%d)��������ʧ��!!!!!!!\r\n", socket_drv_cfg[type].local_port)); 
			DBG_Socket_Print(("TCP Send ret = %d\r\n",ret));
		}
		else
		{
			DBG_Socket_Print("TCP(%d)�������ݳɹ�!!\r\n", socket_drv_cfg[type].local_port);
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
		DBG_Socket_Print("\r\nUDP��������[len:%u][IP:%u.%u.%u.%u][port:%u]: ", len, dst_socket->sin_addr.s_b1,dst_socket->sin_addr.s_b2,dst_socket->sin_addr.s_b3,dst_socket->sin_addr.s_b4,htons(dst_socket->sin_port));

		if(ret < 0)
		{
			over_time++;
			DBG_Socket_Print(("UDP(%d)��������[len:%u][IP:%u.%u.%u.%u][port:%u]ʧ��,������:%d!!!!!!!\r\n", socket_drv_cfg[type].local_port,len, dst_socket->sin_addr.s_b1,dst_socket->sin_addr.s_b2,dst_socket->sin_addr.s_b3,dst_socket->sin_addr.s_b4,htons(dst_socket->sin_port),ret));
		}
		else
		{
			DBG_Socket_Print("UDP���ͳɹ�!!������:%d\r\n",ret);
		}
	}
}


/*****************************************************************************
 �� �� ��  : socket_send_broardcast_msg
 ��������  : ���͹㲥��Ϣ
 �������  : socket_type_enum type  
             U8* buf                
             U32 len                
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��9��4��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
		DBG_Socket_Print("\r\n�㲥��������[len:%u][port:%u]: ", len, port);
		for(i = 0; i < len; i++)
		{
			DBG_Socket_Print("%02X ", buf[i]);
		}
		DBG_Socket_Print("\r\n");
		if(ret < 0)
		{
			ERR_PRINT(("UDP�㲥(%d)��������ʧ��,������:%d!!!!!!!\r\n", socket_drv_cfg[type].local_port,ret));
		}
	}
}


/*****************************************************************************
 �� �� ��  : socket_rcv_msg
 ��������  : socket��������
 �������  : socket_type_enum type          
             INOUT SOCKADDR_IN* src_socket  
             S32 flags                      
             INOUT U8* rcv_buf              
             U32 buf_len                    
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��26��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
			DBG_Socket_Print("\r\nUDP��������[sockt:%u][len:%u][port:%u]: ", type, ret, htons(src_socket->sin_port));
			for(i = 0; i < ret; i++)
			{
				DBG_Socket_Print("%02X ", rcv_buf[i]);
			}
			DBG_Socket_Print("\r\n");
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
	
	if(socket_drv_cfg[type].tcp_enable == TRUE)		//TCP Client ����
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
		socket_drv_cfg[type].p_socket_handle->sin_port = htons(socket_drv_cfg[type].local_port);
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_b1 = tcp_addr_port[0];
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_b2 = tcp_addr_port[1];
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_b3 = tcp_addr_port[2];
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_b4 = tcp_addr_port[3];
		if (connect(socket_drv_cfg[type].socket_fd, (SOCKADDR *)(socket_drv_cfg[type].p_socket_handle), sizeof(SOCKADDR)) < 0)
		{
			DBG_Socket_Print(("socket(tcp-%u)����[IP:%u.%u.%u.%u][port:%u]ʧ��!\r\n", socket_drv_cfg[type].local_port,tcp_addr_port[0],tcp_addr_port[1],tcp_addr_port[2],tcp_addr_port[3],htons(socket_drv_cfg[type].local_port)));
			return 1;
		}
		setsockopt(socket_drv_cfg[type].socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&time_out, sizeof (time_out));
		setsockopt(socket_drv_cfg[type].socket_fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&time_out, sizeof (time_out));

		DBG_Socket_Print("socket(tcp-%u)����[IP:%u.%u.%u.%u][port:%u]�ɹ�!!!!!!\r\n", socket_drv_cfg[type].local_port,tcp_addr_port[0],tcp_addr_port[1],tcp_addr_port[2],tcp_addr_port[3],htons(socket_drv_cfg[type].local_port));
	}

	return 0;
	
}


S32 socket_tcp_disconnect(socket_type_enum type)
{
	int rc;

	if(socket_drv_cfg[type].tcp_enable == TRUE)		//TCP Client ����
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
	}
}



/*****************************************************************************
 �� �� ��  : socket_drv_init
 ��������  : socket����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��26��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void socket_udp_drv_init(socket_type_enum type)
{
	U8 i = 0;
	int rc;
	U8 bDontLinger = FALSE;

	
	if(socket_drv_cfg[type].tcp_enable == FALSE)		//UDP����
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
			ERR_PRINT(("########################socket(udp-%u)����ʧ��!\r\n", socket_drv_cfg[type].local_port));
			return;
		}
		socket_drv_cfg[type].p_socket_handle->sin_family = AF_INET;
		socket_drv_cfg[type].p_socket_handle->sin_port = htons(socket_drv_cfg[type].local_port);
		socket_drv_cfg[type].p_socket_handle->sin_addr.s_addr = INADDR_ANY;
/*			
		socket_drv_cfg[i].p_socket_handle->sin_addr.s_b1 = 192;
		socket_drv_cfg[i].p_socket_handle->sin_addr.s_b2 = 168;
		socket_drv_cfg[i].p_socket_handle->sin_addr.s_b3 = 10;
		socket_drv_cfg[i].p_socket_handle->sin_addr.s_b4 = 235;
*/			
		if (bind(socket_drv_cfg[type].socket_fd, (SOCKADDR *)(socket_drv_cfg[type].p_socket_handle), sizeof(SOCKADDR)) < 0)
		{
			ERR_PRINT(("########################socket(udp-%u)��ʧ��!\r\n", socket_drv_cfg[type].local_port));
			return;
		}

		
		NOTE_PRINT(("########################socket(udp-%u)�����ɹ�!!!!!!\r\n", socket_drv_cfg[type].local_port));
		delay_ms(100);
	}

}

/*eof*/

