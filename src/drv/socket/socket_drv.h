/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : socket_drv.h
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年7月26日
  最近修改   :
  功能描述   : socket模块头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2019年7月26日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

#ifndef _SOCKET_DRV_H_
#define _SOCKET_DRV_H_

#include "pubdef.h"
#include "rl_net.h"


typedef enum
{
	SOCKET_1 = 0,
	SOCKET_2,
    SOCKET_3,
	SOCKET_NUM,
}socket_type_enum;

#define SOCKET_COM			SOCKET_1
#define SOCKET_BROARDCAST	SOCKET_2
#define SOCKET_JSON         SOCKET_3

typedef struct
{				
	S32 						socket_fd;          /* 文件描述符 */
	const U32					tcp_enable;         /* TCP使能 */
	SOCKADDR_IN*				p_socket_handle;    /* socket操作对象 */
	const U32					local_port;         /* 本地端口号 */
}socket_drv_t;

extern S32 socket_tcp_connect(socket_type_enum type);
extern S32 socket_tcp_disconnect(socket_type_enum type);
extern S32 socket_TCP_send_msg(socket_type_enum type, U8* buf, U32 len, IN S32 flag);
extern S32 socket_UDP_send_msg(socket_type_enum type, IN SOCKADDR_IN* dst_socket, U8* buf, U32 len);
extern S32 socket_rcv_msg(socket_type_enum type, INOUT SOCKADDR_IN* src_socket, S32 flags, INOUT U8* rcv_buf, U32 buf_len);
extern void socket_send_broardcast_msg(socket_type_enum type, U16 port, U8* buf, U32 len);
extern void socket_udp_drv_init(socket_type_enum type);

#endif
/*eof*/

