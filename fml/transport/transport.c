/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *    Sergio R. Caprile - "commonalization" from prior samples and/or documentation extension
 *******************************************************************************/

#include "transport.h"
#include "pubDef.h"
//#include <sys/types.h>
#include "iotclient/iotclient.h"

#if !defined(SOCKET_ERROR)
	/** error in socket operation */
	#define SOCKET_ERROR -1
#endif

#if defined(WIN32)
/* default on Windows is 64 - increase to make Linux and Windows the same */
#define FD_SETSIZE 1024
#include <winsock2.h>
#include <ws2tcpip.h>
#define MAXHOSTNAMELEN 256
#define EAGAIN WSAEWOULDBLOCK
#define EINTR WSAEINTR
#define EINVAL WSAEINVAL
#define EINPROGRESS WSAEINPROGRESS
#define EWOULDBLOCK WSAEWOULDBLOCK
#define ENOTCONN WSAENOTCONN
#define ECONNRESET WSAECONNRESET
#define ioctl ioctlsocket
#define socklen_t int
#else
#define INVALID_SOCKET SOCKET_ERROR
//#include <sys/socket.h>
//#include <sys/param.h>
//#include <sys/time.h>
//#include <netinet/in.h>
//#include <netinet/tcp.h>
//#include <arpa/inet.h>
//#include <netdb.h>
//#include <stdio.h>
//#include <unistd.h>
//#include <errno.h>
//#include <fcntl.h>
//#include <string.h>
//#include <stdlib.h>
#endif

#if defined(WIN32)
#include <Iphlpapi.h>
#else
//#include <sys/ioctl.h>
//#include <net/if.h>
#endif

/**
This simple low-level implementation assumes a single connection for a single thread. Thus, a static
variable is used for that connection.
On other scenarios, the user must solve this by taking into account that the current implementation of
MQTTPacket_read() has a function pointer for a function call to get the data to a buffer, but no provisions
to know the caller or other indicator (the socket id): int (*getfn)(unsigned char*, int)
*/
static int mysock = INVALID_SOCKET;


int transport_sendPacketBuffer(unsigned char* buf, int buflen)
{
	int rc = 0;

	rc = send (mysock, (char *)buf, buflen, MSG_DONTWAIT);

    if(rc>0)
    {
        // GLOBAL_PRINT(("send %d bytes,data:", buflen));
        // for(int i=0;i<buflen;i++)
        // {
        //     GLOBAL_PRINT(("0x%02X ",buf[i]));
        // }
        // GLOBAL_PRINT(("\r\n"));
    }
    else
    {
        //ERR_PRINT(("send error,err_code=%d", rc));
    }
    

	return rc;
}


int transport_getdata(unsigned char* buf, int count)
{
	int rc = recv(mysock, (char*)buf, count, 0);
    if(rc>0)
    {
        // DBG_MQTT_Print("received %d bytes count,data:", (int)count);
        // for(int i=0;i<rc;i++)
        // {
        //     DBG_MQTT_Print("0x%02X ",buf[i]);
        // }
        // DBG_MQTT_Print("\r\n");
    }
    else if(rc==-8)
    {
        DBG_MQTT_Print("Waiting for cmd, MQTT alive!\r\n");
    }
    else if(rc==-14)
    {//BSD_ECONNRESET
        DBG_MQTT_Print("Connection reset by the peer");
		MQTT_OFF_LINE = TRUE;
        iotDev.sta = IOT_STA_INIT;
		iotRTCMDev.sta = IOT_STA_INIT;
		iotCmdDev.sta = IOT_STA_INIT;
    }
    else
    {
        DBG_MQTT_Print("receive error,err_code=%d\r\n", rc);
    }

    return rc;
}

/*·Ç×èÈûÄ£Ê½*/
int transport_getdatanb(unsigned char* buf, int count)
{
	int rc = recv(mysock, (char*)buf, count, MSG_DONTWAIT);
    if(rc>0)
    {
        //DBG_SERVER_Print(("received %d bytes count,data:", (int)count));
		/*
        for(int i=0;i<rc;i++)
        {
            DBG_SERVER_Print(("0x%02X ",buf[i]));
        }
        */
        //DBG_SERVER_Print(("\r\n"));
    }
	else if(rc==-8)
    {
        DBG_MQTT_Print("Waiting for cmd, MQTT alive!\r\n");
    }
    else if(rc==-14)
    {//BSD_ECONNRESET
        DBG_MQTT_Print("Connection reset by the peer");
		MQTT_OFF_LINE = TRUE;
        iotDev.sta = IOT_STA_INIT;
		iotRTCMDev.sta = IOT_STA_INIT;
		iotCmdDev.sta = IOT_STA_INIT;
    }
    else
    {
       DBG_MQTT_Print("Waiting for cmd, MQTT alive!\r\n");
    }
    

	return rc;
}


/**
return >=0 for a socket descriptor, <0 for an error code
@todo Basically moved from the sample without changes, should accomodate same usage for 'sock' for clarity,
removing indirections
*/
int transport_open(SOCKADDR_IN addr)
{
#if 0
    int* sock = &mysock;
        int type = SOCK_STREAM;
        struct sockaddr_in address;
    #if defined(AF_INET6)
        struct sockaddr_in6 address6;
    #endif
        int rc = -1;
    #if defined(WIN32)
        short family;
    #else
        sa_family_t family = AF_INET;
    #endif
        struct addrinfo *result = NULL;
        struct addrinfo hints = {0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL};
        static struct timeval tv;

        *sock = -1;
        if (addr[0] == '[')
        ++addr;

        if ((rc = getaddrinfo(addr, NULL, &hints, &result)) == 0)
        {
            struct addrinfo* res = result;

            /* prefer ip4 addresses */
            while (res)
            {
                if (res->ai_family == AF_INET)
                {
                    result = res;
                    break;
                }
                res = res->ai_next;
            }

    #if defined(AF_INET6)
            if (result->ai_family == AF_INET6)
            {
                address6.sin6_port = htons(port);
                address6.sin6_family = family = AF_INET6;
                address6.sin6_addr = ((struct sockaddr_in6*)(result->ai_addr))->sin6_addr;
            }
            else
    #endif
            if (result->ai_family == AF_INET)
            {
                address.sin_port = htons(port);
                address.sin_family = family = AF_INET;
                address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
            }
            else
                rc = -1;

            freeaddrinfo(result);
        }

        if (rc == 0)
        {
            *sock =	socket(family, type, 0);
            if (*sock != -1)
            {
    #if defined(NOSIGPIPE)
                int opt = 1;

                if (setsockopt(*sock, SOL_SOCKET, SO_NOSIGPIPE, (void*)&opt, sizeof(opt)) != 0)
                    Log(TRACE_MIN, -1, "Could not set SO_NOSIGPIPE for socket %d", *sock);
    #endif

                if (family == AF_INET)
                    rc = connect(*sock, (struct sockaddr*)&address, sizeof(address));
        #if defined(AF_INET6)
                else
                    rc = connect(*sock, (struct sockaddr*)&address6, sizeof(address6));
        #endif
            }
        }
        if (mysock == INVALID_SOCKET)
            return rc;

        tv.tv_sec = 1;  /* 1 second Timeout */
        tv.tv_usec = 0;  
        setsockopt(mysock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
        return mysock;
#endif

	int* sock = &mysock;
    int timeout = 20000;

	*sock = socket (AF_INET, SOCK_STREAM, 0);
	
	connect (*sock, (SOCKADDR *)&addr, sizeof (addr));

    setsockopt (*sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof (timeout));
	setsockopt (*sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof (timeout));
	
	return mysock;
}

int transport_close(int sock)
{
#if 0
	int rc;

	rc = shutdown(sock, SHUT_WR);
	rc = recv(sock, NULL, (size_t)0, 0);
	rc = close(sock);

	return rc;
#endif
	
	int rc;
	rc = closesocket (sock);
    if(rc<0)
    {
        ERR_PRINT(("close socket failed!!!,sock=%d",sock));
    }
	delay_ms(1100);

	return rc;
}
