#include "DNS.h"
#include "rl_net.h"
#include "project_def.h"
#include "pubdef.h"

DNS_struct DNS_buff;

void dns_cbfunc(netDNSc_Event event, const NET_ADDR *addr)
{
  switch(event){
    case netDNSc_EventSuccess:
      // Host Address successfully resolved
      netIP_ntoa (addr->addr_type, addr->addr, DNS_buff.dns_ip, sizeof(DNS_buff.dns_ip));
      GLOBAL_PRINT(("IP Address is: %s\r\n", DNS_buff.dns_ip));
      break;
 
    case netDNSc_EventNotResolved:
      ERR_PRINT(("Error, host name does not exist in DNS record database\r\n"));
      break;
 
    case netDNSc_EventTimeout:
      ERR_PRINT(("Error, DNS resolver timeout expired\r\n"));
      break;
 
    case netDNSc_EventError:
      ERR_PRINT(("Error, DNS protocol error occurred\r\n"));
      break;
  }
  DNS_buff.dns_flag = TRUE;
}


U8 DNS_GetHost(const char* host_name)
{
	U8 ret = FALSE;

	GLOBAL_MEMSET(DNS_buff.dns_ip,0x0,sizeof(DNS_buff.dns_ip));
	DNS_buff.dns_flag = FALSE;

	if(netDNSc_GetHostByName(host_name, NET_ADDR_IP4, dns_cbfunc) == netOK)
	{
		GLOBAL_PRINT(("DNS Client Started!!\r\n"));//will complete on callback notification
		ret = TRUE;
	}
	return ret;
}


