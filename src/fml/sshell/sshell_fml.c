/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : sshell_drv.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年5月30日
  最近修改   :
  功能描述   : SSHELL驱动程序
  函数列表   :
              _sshell_add_list
              _sshell_excute_cmd
              _sshell_excute_debug_cmd
              _sshell_excute_set_cmd
              _sshell_excute_show_cmd
              _sshell_find_nextlist
              _sshell_find_prevlist
              sshell_fml_init
              _sshell_print_backspace
              _sshell_print_char
              _sshell_print_clear
              _sshell_print_enter
              _sshell_print_head
              _sshell_print_help_info
              _sshell_print_return
              _sshell_print_string
              _sshell_proc_rcv_data
              _sshell_thread
  修改历史   :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/

/*------------------------------头文件------------------------------*/
#include "sshell/sshell_fml.h"
#include "uart/uart_fml.h"
#include "w25qxx/w25qxx_drv.h"
#include "project_def.h"
#include "gpio/gpio_drv.h"
#include "gnss/ubx.h"
#include "update/update_apl.h"
#include "drv/meas/meas_distance.h"
#include "fml/gnss/nmea.h"
#include "apl/server/server_apl.h"
#include "apl/uwb/uwb.h"
#include "iotclient/iotclient.h"
#include "apl/Ntrip/Ntrip.h"
#include "drv/socket/socket_drv.h"

/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/
#define ROOT_NAME	"rtx"
#define SYS_NAME	"sshell"

#define SAVE_LIST_MAX_NUM		10		//保存最大链表数
#define SSHELL_RCV_BUF_NUM		256		//单条可接收命令字符个数
#define SSHELL_TAIL_CHAR		'\r'	//结束符
#define SSHELL_BACKSPACE		'\b'	//退格符
#define SSHELL_UP				24	    //向上,需要在crt上设置映射值,对应ascii码表
#define SSHELL_DOWN			    25	    //向下
#define SSHELL_LEFT			    27	    //向左
#define SSHELL_RIGHT			26	    //向右
#define SSHELL_ESC				0x1B	//ESC键，暂停显示
#define SSHELL_EXIT			    0x03	//Ctrl+C，用于清除所有循环操作

typedef enum
{
	SPECIAL_PROC_FPGA_CONT_RD = 0x0,		//FPGA循环读取
	SPECIAL_PROC_END,
}SSHELL_SPECIAL_PROC_Type;
/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
static U64 thread_sshell_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_sshell_attr = {
  .stack_mem  = &thread_sshell_stk[0],
  .stack_size = sizeof(thread_sshell_stk),
  .priority = osPriorityBelowNormal7,
};
static const U8 key_up[3] = {0x1B, 0x5B, 0x41};
static const U8 key_down[3] = {0x1B, 0x5B, 0x42};
static const CHAR_T sshell_help_info1[] = {
	"***Command***                     ***Detail***\r\n"
	"  sysinfo                       -show sysinfo\r\n"
	"  dbg on/off                    -debug option\r\n"
	"    position                    -debug position\r\n"	
	"    position_warn               -debug position_warn\r\n"	
	"    server                      -debug mqtt server\r\n"
	"    mqtt                        -enable or disable show MQTT info\r\n"
	"    ntrip                       -enable or disable show Ntrip info\r\n"
	"    socket                      -enable or disable show Socket info\r\n"
	"    all                       -enable or disable show Position and Server info\r\n"		
	"  update                        -update arm firmware\r\n"
	"  format                        -format ext spi flash\r\n"
	"  disconnect                    -disconnect mqtt connect\r\n"
	"  sub                           -subscribe\r\n"
	"  open/close ranging            -open/close uwb ranging\r\n"
	"  open/close det_sig            -open/close detection uwb signal\r\n"
	"  open/close det_sig            -open/close detection uwb signal\r\n"
	"  open/close intf_sig           -open/close send uwb signal\r\n"
	"  start/stop uwb                -start/stop uwb tx and rx\r\n"
	"  start tx_power xxx xxx xxx xxx -start auto choose tx power\r\n"
	"  stop tx_power                 -stop auto choose tx power\r\n"	
	"  show                          -show option\r\n"
	"    ip                          -show ip info\r\n"	
	"    position                    -show uwb position info\r\n"
	
};

static const CHAR_T sshell_help_info2[] =
{
	"  set                       -set option\r\n"
	"    ip xxx.xxx.xxx.xxx      -set ip data\r\n"
  "    mask xxx.xxx.xxx.xxx    -set mask address\r\n"    
  "    gate xxx.xxx.xxx.xxx    -set gateway address\r\n"
	"    device_type xxx         -set device_type data\r\n"
	"    ant_tx_delay xxx        -set ant_tx_delay data\r\n"
	"    ant_rx_delay xxx        -set ant_rx_delay data\r\n"
	"    tag_id xxx              -set tag_id data\r\n"
	"    position xxx xxx xxx    -set position latitude longitude height\r\n"
	"    tag_h xxx               -set tag_h data\r\n"
	"    tx_power xxx            -set tx_power data\r\n"
	"    anchor_idle_num xxx     -set anchor_idle_num data\r\n"		
};


static const CHAR_T sshell_help_info4[] =
{
	"  save                          - save para in flash\r\n"
	"  reset                         - reset para to default and save\r\n"
	"  reboot                        - reboot system\r\n"
	"  Ctrl+C                        - clear all period info\r\n"
	"  clear/c                       - clear screen\r\n"
	"  help, ?                       - display this help\r\n"
	"  tcpstat                       - display a tcp status(only telnet use)\r\n"
	"  rinfo                         - display remote machine info(only telnet use)\r\n"
	"  bye,\r\n"
	"  <ESC>                         - swap protocol(debug or ctrl)\r\n"
	"  <BS>                          - delete Character left\r\n"
	"  <UP>,<DOWN>                   - recall Command History\r\n"

};

static U8 sshell_special_procflag[SPECIAL_PROC_END];
typedef struct
{
	U32 addr;
	U32 num;
}FPGA_CONT_RD_t;
typedef struct save_node
{
	struct save_node *prev;		//前一个链表地址
	U8 *ptr;					//数据指针
	struct save_node *next;		//后一个链表地址
}SaveNode_t;

FPGA_CONT_RD_t fpga_cont_rd;
static SaveNode_t *p_head_node = NULL, *p_cur_save_node = NULL, *p_find_node = NULL;	//保存头结点，当前节点指针
static U8 sshell_rcv_buf[SSHELL_RCV_BUF_NUM + 1];
U16 sshell_rcv_index = 0;
U8 sshell_list_index = 0;
U8 g_auth_need_reboot = FALSE;
extern UPDATE_HANDLE update_handle_g;
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/
#define SSHELL_HELP_INFO() GLOBAL_PRINT(("%s", sshell_help_info1));delay_ms(300);\
							GLOBAL_PRINT(("%s", sshell_help_info2));delay_ms(300);\
							GLOBAL_PRINT(("%s", sshell_help_info4));delay_ms(300);\

/*------------------------------函数声明------------------------------*/

//打印帮助信息
static void _sshell_print_help_info(void)
{
	SSHELL_HELP_INFO();
}

//打印root头部
static void _sshell_print_head(void)
{
	GLOBAL_PRINT(("[%s@%s]# ", SYS_NAME, ROOT_NAME));
}

//打印回车换行符
static void _sshell_print_enter(void)
{
	GLOBAL_PRINT(("\r\n"));
}

//回显字符
static void _sshell_print_char(U8 data)
{
	GLOBAL_PRINT(("%c", data));
}

//回显字符串
static void _sshell_print_string(U8* ptr)
{
	GLOBAL_PRINT(("%s", ptr));
}

//顶格显示 '\r'
static void _sshell_print_return(void)
{
	GLOBAL_PRINT(("\r[%s@%s]# ", SYS_NAME, ROOT_NAME));
}

//退格显示
static void _sshell_print_backspace(void)
{
	GLOBAL_PRINT(("\b \b"));
}

//CLEAR 显示
static void _sshell_print_clear(void)
{
	GLOBAL_PRINT(("%s", ANSI_CLEAR));
}


/*****************************************************************************
 函 数 名  : sshell_add_list
 功能描述  : 增加链表
 输入参数  : U8* ptr
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static void _sshell_add_list(U8* ptr)
{
	SaveNode_t* pList = NULL;
	if(p_cur_save_node)
	{
		if(!GLOBAL_STRCASECMP(ptr, p_cur_save_node->ptr))
		{
			p_find_node = p_cur_save_node;
			return;
		}
	}
	if((++sshell_list_index) > SAVE_LIST_MAX_NUM)
	{
		sshell_list_index = SAVE_LIST_MAX_NUM;
		//pList = save_head;
		p_head_node = p_head_node->next;
		GLOBAL_FREE(p_head_node->prev->ptr);
		GLOBAL_FREE(p_head_node->prev);
		p_head_node->prev = NULL;
	}

	{
		//创建新链表
		pList = (SaveNode_t*)GLOBAL_MALLOC(sizeof(SaveNode_t));
		pList->ptr = GLOBAL_MALLOC(GLOBAL_STRLEN(ptr) + 1);
		GLOBAL_STRCPY(pList->ptr, ptr);
		pList->next = NULL;
		pList->prev = NULL;
		if(p_cur_save_node == NULL)	//首次添加链表
		{
			p_cur_save_node = pList;
			p_head_node = pList;
			p_find_node = p_head_node;
		}
		else
		{
			//追加链表
			pList->prev = p_cur_save_node;
			p_cur_save_node->next = pList;
			p_cur_save_node = p_cur_save_node->next;
			p_cur_save_node->next = NULL;

			/*pList->prev = p_find_node;
			p_find_node->next = pList;
			p_find_node = p_find_node->next;
			p_find_node->next = NULL;*/
			p_find_node = p_cur_save_node;
		}
	}

}

/*****************************************************************************
 函 数 名  : sshell_find_prevlist
 功能描述  : 查找上一个保存节点
 输入参数  : void
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static SaveNode_t* _sshell_find_prevlist(void)
{
	SaveNode_t* pList = NULL;
	if(p_find_node)
	{
		pList = p_find_node;
		//p_find_node->next = p_find_node;
		if(p_find_node->prev)
		{
			p_find_node = p_find_node->prev;
		}
		GLOBAL_MEMSET(sshell_rcv_buf, 0x0, sizeof(sshell_rcv_buf));
		GLOBAL_STRCPY(sshell_rcv_buf, pList->ptr);
		sshell_rcv_index = GLOBAL_STRLEN(pList->ptr);
		return pList;
	}
	else
	{
		return NULL;
	}
}

/*****************************************************************************
 函 数 名  : sshell_find_nextlist
 功能描述  : 查找下一个保存节点
 输入参数  : void
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static SaveNode_t* _sshell_find_nextlist(void)
{
	SaveNode_t* pList = NULL;
	if(p_find_node)
	{
		pList = p_find_node;
		if(p_find_node->next)
		{
			p_find_node = p_find_node->next;
		}
		GLOBAL_MEMSET(sshell_rcv_buf, 0x0, sizeof(sshell_rcv_buf));
		GLOBAL_STRCPY(sshell_rcv_buf, pList->ptr);
		sshell_rcv_index = GLOBAL_STRLEN(pList->ptr);
		return pList;
	}
	else
	{
		return NULL;
	}
}

/*****************************************************************************
 函 数 名  : _sshell_excute_set_cmd
 功能描述  : 设置命令
 输入参数  : U8* ptr
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static BOOL _sshell_excute_set_cmd(U8* ptr)
{
	BOOL ret = FALSE;

  if(!GLOBAL_STRNCASECMP(ptr, "ip ", 3))
	{
		U8 ip_buf[NET_ADDR_IP4_LEN];

		netIP_aton((const char*)ptr+3, NET_ADDR_IP4, ip_buf);
		if(net_ip_is_legal(ip_buf))
		{
			sys_set_ipaddr(ptr+3);
            sys_para_save();
            GLOBAL_PRINT(("IP set up and save\r\n"));
			ret = TRUE;
		}
		else
		{
			ERR_PRINT(("输入IP非法!!!!\r\n"));
		}
	}
  else if(!GLOBAL_STRNCASECMP(ptr, "mac ", 4))
	{
		U8 mac_buf[10];

		netMAC_aton((char*)ptr+4, mac_buf);
		if(net_mac_is_legal(mac_buf))
		{
			sys_set_macaddr(ptr+4);
            sys_para_save();
            GLOBAL_PRINT(("MAC set up and save\r\n"));
			ret = TRUE;
		}
		else
		{
			ERR_PRINT(("输入MAC非法!!!!\r\n"));
		}
	}	
  else if(!GLOBAL_STRNCASECMP(ptr, "mask ", 5))
	{
		U8 ip_buf[NET_ADDR_IP4_LEN];

		netIP_aton((const char*)ptr+5, NET_ADDR_IP4, ip_buf);
		if(net_ip_is_legal(ip_buf))
		{
			sys_set_maskaddr(ptr+5);
            sys_para_save();
            GLOBAL_PRINT(("MASK set up and save\r\n"));
			ret = TRUE;
		}
		else
		{
			ERR_PRINT(("输入MASK非法!!!!\r\n"));
		}
	} 
  else if(!GLOBAL_STRNCASECMP(ptr, "gate ", 5))
	{
		U8 ip_buf[NET_ADDR_IP4_LEN];

		netIP_aton((const char*)ptr+5, NET_ADDR_IP4, ip_buf);
		if(net_ip_is_legal(ip_buf))
		{
			sys_set_gatewayaddr(ptr+5);
            sys_para_save();
            GLOBAL_PRINT(("Gateway set up and save\r\n"));
			ret = TRUE;
		}
		else
		{
			ERR_PRINT(("输入Gateway非法!!!!\r\n"));
		}
	}
  else if(!GLOBAL_STRNCASECMP(ptr, "device_type ", 12))
	{
		U8 buf[3] = {0};
		GLOBAL_STRNCPY(buf, ptr+12, 1);
		g_device_config.device_type = GLOBAL_ATOI(buf);
		if(g_device_config.device_type != TAG)
		{
			g_device_config.device_type = ANCHOR;
		}		
		save_device_para();
		ret = TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "ant_tx_delay ", 13))
	{
		g_device_config.ant_tx_delay = GLOBAL_STRTOUL(ptr+13, NULL, 10);
		dwt_setrxantennadelay(g_device_config.ant_tx_delay);   
		save_device_para();
		ret = TRUE;
	} 
	else if(!GLOBAL_STRNCASECMP(ptr, "ant_rx_delay ", 13))
	{	
		g_device_config.ant_rx_delay = GLOBAL_STRTOUL(ptr+13, NULL, 10);
		dwt_setrxantennadelay(g_device_config.ant_rx_delay);   
		save_device_para();
		ret = TRUE;

	} 	
	else if(!GLOBAL_STRNCASECMP(ptr, "anchor_id ", 10))
	{
		g_device_config.anchor_id = GLOBAL_STRTOUL(ptr+10, NULL, 10);
		save_device_para();
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "position ", 9))
	{
		char *p_end;

		g_device_config.position[0] = GLOBAL_STRTOD(ptr+9, &p_end);
		g_device_config.position[1] = GLOBAL_STRTOD(p_end, &p_end);
		g_device_config.position[2] = GLOBAL_STRTOD(p_end, &p_end);

		save_device_para();
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "tag_h ", 6))
	{
		char *p_end;
		g_device_config.tag_h = GLOBAL_STRTOD(ptr+6, &p_end);

		save_device_para();
		ret = TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "anchor_h ", 9))
	{
		char *p_end;
		g_device_config.anchor_h = GLOBAL_STRTOD(ptr+9, &p_end);

		save_device_para();
		ret = TRUE;
	}		
	else if(!GLOBAL_STRNCASECMP(ptr, "on_left ", 8))
	{
		g_device_config.on_left = GLOBAL_STRTOUL(ptr+8, NULL, 10);
		save_device_para();
		ret = TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "tag_id ", 7))
	{
		g_device_config.tag_id = GLOBAL_STRTOUL(ptr+7, NULL, 10);
		save_device_para();
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "tx_power ", 9))
	{
		u32 tx_power, reg_value;

		tx_power = calc_tx_power_config_value(ptr+9);
		GLOBAL_PRINT(("tx_power = %d\r\n",tx_power));
		
		dwt_write32bitreg(TX_POWER_ID, tx_power);
		delay_ms(1);
		reg_value = dwt_read32bitreg(TX_POWER_ID);
		if(reg_value == tx_power)
		{		
			float db;
			g_device_config.tx_power = tx_power;
			save_device_para();	
			db = calc_tx_power_config_value_to_db(g_device_config.tx_power);
			GLOBAL_PRINT(("tx_power config is suc, tx_power: %2.1f db\r\n",  db));
			ret = TRUE;
		}
		else
		{
			ERR_PRINT(("tx_power config is failed\r\n"));
		}
				
	}

	else if(!GLOBAL_STRNCASECMP(ptr, "anchor_idle_num ", 16))
	{
		g_device_config.anchor_idle_num = GLOBAL_STRTOUL(ptr+16, NULL, 10);
		if(g_device_config.anchor_idle_num < 1 || g_device_config.anchor_idle_num > 2)
		{
			g_device_config.anchor_idle_num = 1;
		}
		
		save_device_para();
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "chan ", 5))
	{
		u32 chan;
		chan = GLOBAL_STRTOUL(ptr+5, NULL, 10);
		if(chan < 1 || chan  > 7 || chan == 6)
		{
			ERR_PRINT(("config dw1000 chan is fail, value is invalid\r\n"));
		}
		else
		{
			osStatus_t status = osThreadSuspend(thread_UWB_id);
			if(status == osOK)
			{
				GLOBAL_PRINT(("stop uwb is suc\r\n"));
			}
			else
			{
				GLOBAL_PRINT(("stop uwb is fail,osThreadSuspend return %d\r\n", status));
				return TRUE;
			}			
			
			g_device_config.chan = chan;
			save_device_para();			
			DWM1000_init();

			status = osThreadResume(thread_UWB_id);
			if(status == osOK)
			{
				GLOBAL_PRINT(("start uwb is suc\r\n"));
			}
			else
			{
				GLOBAL_PRINT(("start uwb is fail,osThreadResume return %d\r\n", status));
			}
			
		}
	
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "mxt ", 4))
	{
		uart_send_data(MXT_COM,(U8*)ptr+4,strlen(ptr+4));
		uart_send_data(MXT_COM,"\r\n",2);
		delay_ms(10);
		GLOBAL_PRINT(("MXT config Over!!\r\n"));
		ret = TRUE;
	}

	


	

//  ret = TRUE;

	return ret;
}

/*****************************************************************************
 函 数 名  : _sshell_excute_show_cmd
 功能描述  : 执行显示命令
 输入参数  : U8* ptr
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static BOOL _sshell_excute_show_cmd(U8* ptr)
{
	BOOL ret = FALSE;

	if(!GLOBAL_STRCASECMP(ptr, "ip"))
	{
		char temp_str[50] = {0};
		netIP_ntoa(NET_ADDR_IP4, main_handle_g.cfg.net.net_ip, temp_str, 16);
		GLOBAL_PRINT((" IP ADDR:\t%s\r\n", temp_str));
		netIP_ntoa(NET_ADDR_IP4, main_handle_g.cfg.net.net_mask, temp_str, 16);
		GLOBAL_PRINT((" MASK ADDR:\t%s\r\n", temp_str));
		netIP_ntoa(NET_ADDR_IP4, main_handle_g.cfg.net.net_gateway, temp_str, 16);
		GLOBAL_PRINT((" GATE ADDR:\t%s\r\n", temp_str));
		netMAC_ntoa(main_handle_g.cfg.net.net_mac, temp_str, 20);
		GLOBAL_PRINT((" MAC ADDR:\t%s\r\n", temp_str));
		//GLOBAL_PRINT((" MAC ADDR:\t%s\r\n", mac_ntoa(eth0_mac_addr)));
		ret = TRUE;
	}
	else if(!GLOBAL_STRCASECMP(ptr, "position"))
	{
		show_position_para();
	}

	return ret;
}



/*****************************************************************************
 函 数 名  : _sshell_excute_debug_cmd
 功能描述  : 执行调试设置指令
 输入参数  : U8* ptr
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static BOOL _sshell_excute_debug_cmd(U8* ptr)
{
	BOOL ret = FALSE;

	if(!GLOBAL_STRNCASECMP(ptr, "on ", 3))
	{
		ret = TRUE;
		if(!GLOBAL_STRCASECMP(ptr+3, "position"))
		{
			sys_debug_set_type(SYS_DEBUG_POST);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "position_warn"))
		{
			sys_debug_set_type(SYS_DEBUG_POST_WARNING);
		}			
	    else if(!GLOBAL_STRCASECMP(ptr+3, "server"))
	    {
	        sys_debug_set_type(SYS_DEBUG_SERVER);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "mqtt"))
	    {
	        sys_debug_set_type(SYS_DEBUG_MQTT);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "socket"))
	    {
	        sys_debug_set_type(SYS_DEBUG_SOCKET);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "ntrip"))
	    {
	        sys_debug_set_type(SYS_DEBUG_NTRIP);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "all"))
		{
			sys_debug_set_type(SYS_DEBUG_POST);
			sys_debug_set_type(SYS_DEBUG_SERVER);
		}
		else
		{
			ret = FALSE;
		}		
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "off ", 4))
	{
		ret = TRUE;
		if(!GLOBAL_STRCASECMP(ptr+4, "position"))
		{
			sys_debug_clear_type(SYS_DEBUG_POST);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "position_warn"))
		{
			sys_debug_clear_type(SYS_DEBUG_POST_WARNING);
		}		
    	else if(!GLOBAL_STRCASECMP(ptr+4, "server"))
		{
			sys_debug_clear_type(SYS_DEBUG_SERVER);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "mqtt"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_MQTT);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+4, "socket"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_SOCKET);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+4, "ntrip"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_NTRIP);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+4, "all"))
		{
			sys_debug_clear_all();
		}
		else
		{
			ret = FALSE;
		}
	}
	
#if 0
	if(!GLOBAL_STRNCASECMP(ptr, "on ", 3))
	{
		ret = TRUE;
		if(!GLOBAL_STRCASECMP(ptr+3, "test"))
		{
			sys_debug_set_type(SYS_DEBUG_TEST);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "warn"))
		{
			sys_debug_set_type(SYS_DEBUG_WARN);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "error"))
		{
			sys_debug_set_type(SYS_DEBUG_ERROR);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "note"))
		{
			sys_debug_set_type(SYS_DEBUG_NOTE);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "ctrl"))
		{
			sys_debug_set_type(SYS_DEBUG_CTRL);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "gnss"))
		{
			sys_debug_set_type(SYS_DEBUG_GNSS);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "ocxo"))
		{
			sys_debug_set_type(SYS_DEBUG_OCXO);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "tdc"))
		{
			sys_debug_set_type(SYS_DEBUG_TDC);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "socket"))
		{
			sys_debug_set_type(SYS_DEBUG_SOCKET);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "pl"))
		{
			sys_debug_set_type(SYS_DEBUG_PL);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "ant"))
		{
			sys_debug_set_type(SYS_DEBUG_ANT);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "nav"))
		{
			sys_debug_set_type(SYS_DEBUG_NAV);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "fpga"))
		{
			sys_debug_set_type(SYS_DEBUG_FPGA);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "heart"))
		{
			//gxw_protocol_send_heart_msg();
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "hand"))
		{
			//gxw_protocol_send_handshake_msg();
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "eph gps"))
		{
			sys_debug_set_type(SYS_DEBUG_EPH_GPS);
		}
        else if(!GLOBAL_STRCASECMP(ptr+3, "eph bds"))
		{
			sys_debug_set_type(SYS_DEBUG_EPH_BDS);
		}
        else if(!GLOBAL_STRCASECMP(ptr+3, "eph glo"))
		{
			sys_debug_set_type(SYS_DEBUG_EPH_GLO);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "ad936x"))
		{
			sys_debug_set_type(SYS_DEBUG_AD936X);
		}
        else if(!GLOBAL_STRCASECMP(ptr+3, "ubx"))
        {
            sys_debug_set_type(SYS_DEBUG_UBX);
        }
		else if(!GLOBAL_STRCASECMP(ptr+3, "ubx sat"))
		{
			sys_debug_set_type(SYS_DEBUG_UBX_SAT);
		}
        else if(!GLOBAL_STRCASECMP(ptr+3, "temp"))
		{
			sys_debug_set_type(SYS_DEBUG_TEMP);
		}
        else if(!GLOBAL_STRCASECMP(ptr+3, "bcode"))
		{
			sys_debug_set_type(SYS_DEBUG_BCODE);
		}
        else if(!GLOBAL_STRCASECMP(ptr+3, "json"))
		{
			sys_debug_set_type(SYS_DEBUG_JSON);
		}
        else if(!GLOBAL_STRCASECMP(ptr+3, "usartctrl"))
		{
			sys_debug_set_type(SYS_DEBUG_USARTCTRL);
		}
        else if(!GLOBAL_STRCASECMP(ptr+3, "sat"))
        {
            sys_debug_set_type(SYS_DEBUG_SAT);
        }
		else
		{
			ret = FALSE;
		}
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "off ", 4))
	{
		ret = TRUE;
		if(!GLOBAL_STRCASECMP(ptr+4, "test"))
		{
			sys_debug_clear_type(SYS_DEBUG_TEST);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "warn"))
		{
			sys_debug_clear_type(SYS_DEBUG_WARN);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "error"))
		{
			sys_debug_clear_type(SYS_DEBUG_ERROR);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "note"))
		{
			sys_debug_clear_type(SYS_DEBUG_NOTE);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "ctrl"))
		{
			sys_debug_clear_type(SYS_DEBUG_CTRL);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "gnss"))
		{
			sys_debug_clear_type(SYS_DEBUG_GNSS);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "ocxo"))
		{
			sys_debug_clear_type(SYS_DEBUG_OCXO);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "socket"))
		{
			sys_debug_clear_type(SYS_DEBUG_SOCKET);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "tdc"))
		{
			sys_debug_clear_type(SYS_DEBUG_TDC);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "nav"))
		{
			sys_debug_clear_type(SYS_DEBUG_NAV);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "pl"))
		{
			sys_debug_clear_type(SYS_DEBUG_PL);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "ant"))
		{
			sys_debug_clear_type(SYS_DEBUG_ANT);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "fpga"))
		{
			sys_debug_clear_type(SYS_DEBUG_FPGA);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "eph gps"))
		{
			sys_debug_clear_type(SYS_DEBUG_EPH_GPS);
		}
        else if(!GLOBAL_STRCASECMP(ptr+4, "eph bds"))
		{
			sys_debug_clear_type(SYS_DEBUG_EPH_BDS);
		}
        else if(!GLOBAL_STRCASECMP(ptr+4, "eph glo"))
		{
			sys_debug_clear_type(SYS_DEBUG_EPH_GLO);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "ad936x"))
		{
			sys_debug_clear_type(SYS_DEBUG_AD936X);
		}
        else if(!GLOBAL_STRCASECMP(ptr+4, "ubx"))
        {
            sys_debug_clear_type(SYS_DEBUG_UBX);
        }
		else if(!GLOBAL_STRCASECMP(ptr+4, "ubx sat"))
        {
            sys_debug_clear_type(SYS_DEBUG_UBX_SAT);
        }
        else if(!GLOBAL_STRCASECMP(ptr+4, "temp"))
        {
            sys_debug_clear_type(SYS_DEBUG_TEMP);
        }
        else if(!GLOBAL_STRCASECMP(ptr+4, "bcode"))
		{
			sys_debug_clear_type(SYS_DEBUG_BCODE);
		}
        else if(!GLOBAL_STRCASECMP(ptr+4, "json"))
		{
			sys_debug_clear_type(SYS_DEBUG_JSON);
		}
        else if(!GLOBAL_STRCASECMP(ptr+4, "usartctrl"))
		{
			sys_debug_clear_type(SYS_DEBUG_USARTCTRL);
		}
        else if(!GLOBAL_STRCASECMP(ptr+4, "sat"))
		{
			sys_debug_clear_type(SYS_DEBUG_SAT);
		}        
		else
		{
			ret = FALSE;
		}
	}

#endif
	return ret;
}



/*****************************************************************************
 函 数 名  : _sshell_excute_cmd
 功能描述  : 执行sshell命令
 输入参数  : U8* ptr
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
 BOOL sshell_excute_cmd(U8* ptr)
{

	if(GLOBAL_STRCASECMP(ptr, "sysinfo") == 0)
	{
		SYSINFO_PRINT();
		return TRUE;
	}	
	else if(GLOBAL_STRCASECMP(ptr, "clear") == 0)
	{
		_sshell_print_clear();
		return TRUE;
	}
    else if(!GLOBAL_STRCASECMP(ptr, "format"))
	{
		w25qxx_erase_fs_sector();
		return TRUE;
	}
  else if(!GLOBAL_STRCASECMP(ptr, "disconnect"))
  {
    WARN_PRINT(("disconnect mqtt connect!!!\r\n"));
    mqttDisconnect(&iotDev);
    return TRUE;
  }
  else if(!GLOBAL_STRCASECMP(ptr, "sub cmd"))
  {
     deviceSubscribe(MQTT_CMD);
     return TRUE;
  }
	else if(GLOBAL_STRCASECMP(ptr, "reset") == 0)
	{
		sys_para_reset();
		reset_position_default_para();
    sys_para_save();     //added by sunj 2019-10-18 14:31
    GLOBAL_PRINT(("all parameters have been reset and saved\r\n"));
		return TRUE;
	}
	else if(GLOBAL_STRCASECMP(ptr, "reboot") == 0)
	{
		GLOBAL_PRINT(("\r\n\r\n\r\nPlease waiting for system rebooting!!!!!!\r\n\r\n\r\n"));
		delay_ms(100);
		NVIC_SystemReset();
		return TRUE;
	}
	else if(!GLOBAL_STRCASECMP(ptr, "help") || !GLOBAL_STRCASECMP(ptr, "?"))
	{
		_sshell_print_help_info();
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "dbg ", 4))
	{
		if(_sshell_excute_debug_cmd(ptr+4) == TRUE)
		{
			return TRUE;
		}
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "set ", 4))
	{
		if(_sshell_excute_set_cmd(ptr+4) == TRUE)
		{
			return TRUE;
		}
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "open ranging", 12))
	{	
		g_ranging_flag = 1;
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "close ranging", 13))
	{		
		g_ranging_flag = 0;	
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "open det_sig", 12))
	{ 	
		g_detection_signal_flag = 1; 
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "close det_sig", 13))
	{ 	
		g_detection_signal_flag = 0; 
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "open intf_sig", 13))
	{ 	
		g_interference_signal_flag = 1; 
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "close intf_sig", 14))
	{ 	
		g_interference_signal_flag = 0; 
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "start tx_power", 14))
	{	
		char *p_end;
		
		g_dwt_auto_tx_power_config.auto_choose_tx_power = 1;
		g_dwt_auto_tx_power_config.main_device_rssi_threshold = GLOBAL_STRTOD(ptr+14, &p_end);
		g_dwt_auto_tx_power_config.sub_device_rssi_threshold = GLOBAL_STRTOD(p_end, &p_end);
		g_dwt_auto_tx_power_config.main_device_rssi_rang = GLOBAL_STRTOD(p_end, &p_end);
		g_dwt_auto_tx_power_config.sub_device_rssi_rang = GLOBAL_STRTOD(p_end, &p_end);
		
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "stop tx_power", 13))
	{		
		g_dwt_auto_tx_power_config.auto_choose_tx_power = 0;
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
				
		return TRUE;
	}		
	else if(!GLOBAL_STRNCASECMP(ptr, "stop uwb", 8))
	{ 	
		osStatus_t status = osThreadSuspend(thread_UWB_id);
		if(status == osOK)
		{
			GLOBAL_PRINT(("stop uwb is suc\r\n"));
			return TRUE;
		}
		else
		{
			GLOBAL_PRINT(("stop uwb is fail,osThreadSuspend return %d\r\n", status));
		}	
	} 
	else if(!GLOBAL_STRNCASECMP(ptr, "start uwb", 9))
	{
		osStatus_t status = osThreadResume(thread_UWB_id);
		if(status == osOK)
		{
			GLOBAL_PRINT(("start uwb is suc\r\n"));
			return TRUE;
		}
		else
		{
			GLOBAL_PRINT(("start uwb is fail,osThreadResume return %d\r\n", status));
		}
	
		
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "show ", 5))
	{
		if(_sshell_excute_show_cmd(ptr+5) == TRUE)
		{
			return TRUE;
		}
	}
	else if((GLOBAL_STRLEN(ptr) == 1) && (!GLOBAL_STRCASECMP(ptr, "c")))
	{
		GLOBAL_MEMSET(sshell_special_procflag, 0x0, sizeof(sshell_special_procflag));
		sys_debug_clear_all();
		// _sshell_print_enter();
		// _sshell_print_head();
		return TRUE;
	}
	else if(!GLOBAL_STRCASECMP(ptr, "save"))
	{
		sys_para_save();
		save_device_para();
		return TRUE;
	}
	else if(!GLOBAL_STRCASECMP(ptr, "update"))
	{
		rtc_bakup_write(BKUP_BOOT_UPGRADE);
		//update_flag_write(BKUP_BOOT_UPGRADE);
        GLOBAL_PRINT(("updated arm firmware,need to reboot into effect\r\n"));
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "nmea off", 8))
	{
		NMEA_OPEN_SWITCH = 0;
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "nmea on", 7))
	{
		NMEA_OPEN_SWITCH = 1;
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "stop Ntrip", 10))
	{
		RTCM_SWITCH = MQTT_GET_RTCM;
		GLOBAL_PRINT(("Stop Ntrip is OK!\r\n"));
		return TRUE;	
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "start Ntrip", 11))
	{
		RTCM_SWITCH = NTRIP_GET_RTCM;
		GLOBAL_PRINT(("Resume Ntrip is OK!\r\n"));
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "mqtt log on", 11))
	{
		uart_set_dbg_outmode(DBG_OUT_MQTT);
		return TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "mqtt log off", 12))
	{
		uart_set_dbg_outmode(DBG_OUT_USART);
		return TRUE;
	}
	

	if(uart_get_dbg_outmode() == DBG_OUT_USART)
	{
		ERR_PRINT(("Unknow cmd: %s\r\n", ptr));

		_sshell_print_enter();
	}

	return FALSE;
}

/*****************************************************************************
 函 数 名  : _sshell_proc_rcv_data
 功能描述  : 处理接收数据
 输入参数  : U8 data
 输出参数  : 无
 返 回 值  : static
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static void _sshell_proc_rcv_data(U8 data)
{
	static U8 rcvConsoleCount = 0;
	static U8 rcvConsoleBuf[10];

	if(sshell_rcv_index >= SSHELL_RCV_BUF_NUM)
	{
		return;
	}

	if(data == SSHELL_ESC || rcvConsoleCount)	//控制符，例如上下组合键
	{
		rcvConsoleBuf[rcvConsoleCount++] = data;
		if(rcvConsoleCount >= 3)
		{
			rcvConsoleCount = 0;
			if(!GLOBAL_MEMCMP(rcvConsoleBuf, key_up, 3))	//向上按键
			{
				SaveNode_t* pList = NULL;
				if(sshell_rcv_index > 0)
				{
					//_sshell_add_list(sshell_rcv_buf);	//增加至保存链表中
				}

				pList = _sshell_find_prevlist();
				if(pList && (pList->ptr))
				{
					_sshell_print_return();
					_sshell_print_string(pList->ptr);
					GLOBAL_PRINT(("%c%c%c", 0x1B, 0x5B, 0x4A));
				}
			}
			else if(!GLOBAL_MEMCMP(rcvConsoleBuf, key_down, 3))	//向下按键
			{
				SaveNode_t* pList = NULL;
				#if 1
				if(sshell_rcv_index > 0)
				{
					//_sshell_add_list(sshell_rcv_buf);	//增加至保存链表中
				}

				pList = _sshell_find_nextlist();
				if(pList && (pList->ptr))
				{
					_sshell_print_return();
					_sshell_print_string(pList->ptr);
					GLOBAL_PRINT(("%c%c%c", 0x1B, 0x5B, 0x4A));
				}
				#endif
			}
		}
	}
 /* BEGIN: Added by Winlab, 2018/3/24 */
	else if(data == SSHELL_EXIT)
	{
		GLOBAL_MEMSET(sshell_special_procflag, 0x0, sizeof(sshell_special_procflag));
		sys_debug_clear_all();
		_sshell_print_enter();
		_sshell_print_head();
	}
 /* END:   Added by Winlab, 2018/3/24   PN: */
	else if(data == SSHELL_TAIL_CHAR)	//结束符
	{
		_sshell_print_enter();
		if(sshell_rcv_index > 0)
		{
			_sshell_add_list(sshell_rcv_buf);	//增加至保存链表中
			sshell_excute_cmd(sshell_rcv_buf);
			sshell_rcv_index = 0;
			GLOBAL_MEMSET(sshell_rcv_buf, 0x0, sizeof(sshell_rcv_buf));
		}
		_sshell_print_head();
	}
	else if(data == SSHELL_BACKSPACE)	//退格符
	{
		if(sshell_rcv_index > 0)
		{
			sshell_rcv_buf[--sshell_rcv_index] = '\0';
			_sshell_print_backspace();			//退格显示
		}
		//_sshell_print_return();
		//_sshell_print_string(sshell_rcv_buf);
	}
	else if(data == SSHELL_UP)			//向上
	{
		SaveNode_t* pList = NULL;
		if(sshell_rcv_index > 0)
		{
			//_sshell_add_list(sshell_rcv_buf);	//增加至保存链表中
		}

		pList = _sshell_find_prevlist();
		if(pList && (pList->ptr))
		{
			_sshell_print_return();
			_sshell_print_string(pList->ptr);
			GLOBAL_PRINT(("%c%c%c", 0x1B, 0x5B, 0x4A));
		}
	}
	else if(data == SSHELL_DOWN)			//向下
	{
		SaveNode_t* pList = NULL;
		#if 1
		if(sshell_rcv_index > 0)
		{
			//_sshell_add_list(sshell_rcv_buf);	//增加至保存链表中
		}

		pList = _sshell_find_nextlist();
		if(pList && (pList->ptr))
		{
			_sshell_print_return();
			_sshell_print_string(pList->ptr);
			GLOBAL_PRINT(("%c%c%c", 0x1B, 0x5B, 0x4A));
		}
		#endif
	}
	/*else if(data == SSHELL_LEFT)			//向左
	{
		//busyboxEchoLeft();
	}
	else if(data == SSHELL_RIGHT)			//向右
	{

	}
	*/
	else if((data >= 32) && (data <= 127))
	{
		_sshell_print_char(data);
		sshell_rcv_buf[sshell_rcv_index++] = data;
	}
}

/*****************************************************************************
 函 数 名  : _sshell_thread
 功能描述  : sshell处理线程
 输入参数  : void const * parg
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void _sshell_thread(void* arg)
{
	static U32 count = 0;

	while(1)
	{
		CHAR_T* pData = NULL;

		pData = uart_fetch_sshell_buf();
		if(pData)
		{
			_sshell_proc_rcv_data(*(pData));
		}
		else
		{
			delay_ms(5);
		}

		count++;
	}
}

/*****************************************************************************
 函 数 名  : sshell_fml_init
 功能描述  : sshell模块初始化
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2019年5月30日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void sshell_fml_init(void)
{
	osThreadId_t thread_sshell_id = 0;

	thread_sshell_id = osThreadNew(_sshell_thread, NULL, &thread_sshell_attr);
	GLOBAL_HEX(thread_sshell_id);
	GLOBAL_PRINT(("\r\n\r\n软件编译日期: %s %s. \r\n", BUILD_DATE, BUILD_TIME));
}

/*eof*/
