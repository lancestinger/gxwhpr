#ifndef _DNS_H__
#define _DNS_H__

#include "project_def.h"
#include "pubdef.h"


typedef struct
{
	U8 dns_flag;
	char dns_ip[20];
	
}DNS_struct;

extern DNS_struct DNS_buff;

extern U8 DNS_GetHost(const char* host_name);
extern void dns_cbfunc (netDNSc_Event event, const NET_ADDR *addr);

#endif

