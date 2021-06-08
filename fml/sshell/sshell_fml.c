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
#include "uwb_post/uwb_post.h"
#include "fml/gnss/nmea.h"
#include "apl/server/server_apl.h"
#include "apl/uwb/uwb.h"
#include "iotclient/iotclient.h"
#include "apl/Ntrip/Ntrip.h"
#include "drv/socket/socket_drv.h"
#include "fml/dnsclient/DNS.h"

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
	"    efs_gen                     -debug efs general info\r\n"	
	"    efs_post                    -debug efs position info\r\n"
	"    efs_post_note               -debug efs position note info\r\n"
	"    efs_rg                      -debug efs ranging info\r\n"	
	"    efs_rssi                    -debug efs signal rssi\r\n"
	"    efs_err                     -debug efs error info\r\n"
	"    server                      -debug mqtt server\r\n"	
	"  update                        -update arm firmware\r\n"
	"  format                        -format ext spi flash\r\n"
	"  disconnect                    -disconnect mqtt connect\r\n"
	"  sub                           -subscribe\r\n"
	"  open/close rg                 -open/close efs ranging\r\n"
	"  open/close det_sig            -open/close detection efs signal\r\n"
	"  open/close intf_sig           -open/close send efs signal\r\n"
	"  open/close filter             -open/close filter\r\n"
	"  open/close efs                -open/close efs tx and rx\r\n"
	"  open tx_power xxx xxx xxx xxx -open auto choose tx power\r\n"
	"  close tx_power                -close auto choose tx power\r\n"
	"  open ant_cal xxx xxx          -open auto calibration of antenna delay\r\n"
	"  close ant_cal                 -close auto calibration of antenna delay\r\n"
	"  show                          -show option\r\n"
	"    ip                          -show ip info\r\n"	
	"    post_para                   -show post para\r\n"
	"    anchor_state                -show anchor head or tail state\r\n"	
};

static const CHAR_T sshell_help_info2[] =
{
	"  set                       -set option\r\n"
	"    ip xxx.xxx.xxx.xxx      -set ip data\r\n"
	"    mask xxx.xxx.xxx.xxx    -set mask address\r\n"    
	"    gate xxx.xxx.xxx.xxx    -set gateway address\r\n"
	"    device_type xxx         -set device_type data\r\n"
	"    ant_delay xxx           -set ant_delay data\r\n"
	"    tag_id xxx              -set tag_id data\r\n"
	"    anchor_coord xxx xxx xxx -set anchor_coord latitude longitude height\r\n"
	"    ref_coord xxx xxx xxx   -set ref_coord latitude longitude height\r\n"
	"    sig_qa_thr xxx          -set sig_qa_thr data\r\n" 
	"    t2wall_d xxx            -set t2wall_d data\r\n"	
	"    t2wall_thr xxx          -set t2wall_thr data\r\n"
	"    t2wall_fl_num xxx       -set t2wall_fl_num data\r\n"
	"    tx_power xxx            -set tx_power data\r\n"
	"    anchor_idle_num xxx     -set anchor_idle_num data\r\n"	
	"    rtkip xxx.xxx.xxx.xxx      -set RTK ip or realm\r\n"
	"    MSip  xxx.xxx.xxx.xxx      -set Ministry Standard ip\r\n"
	"    mqttip xxx.xxx.xxx.xxx     -set MQTT server ip or realm\r\n"
	"    rtkport xxxx               -set RTK port\r\n"
	"    MSport  xxxx               -set Ministry Standard port\r\n"
	"    MQTTport xxxx              -set MQTT server port\r\n"
};


static const CHAR_T sshell_help_info3[] = {
	"  dbg on/off                    -debug option\r\n"	
	"    socket                      -enable or disable show Socket info\r\n"
	"    all                         -enable or disable show Position and Server info\r\n"		
	"    GGA                         -debug NMEA GGA data\r\n"
	"    RMC                         -debug NMEA RMC data\r\n"
	"    RTK                         -debug RTK Position data\r\n"
	"    ENH                         -debug Enhance Position data\r\n"
	"    mqtt                        -enable or disable show MQTT info\r\n"
	"    ntrip                       -enable or disable show Ntrip info\r\n"
	"    IMU                         -enable or disable show IMU info\r\n"
	"    IMURAW                      -enable or disable show IMU raw_data\r\n"
	"  start/stop Ntrip              -start/stop Ntrip thread\r\n"		  
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
							GLOBAL_PRINT(("%s", sshell_help_info3));delay_ms(300);\
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
	else if(!GLOBAL_STRNCASECMP(ptr, "ant_delay ", 10))
	{
		u16 ant_delay_dist;
		
		ant_delay_dist = GLOBAL_STRTOUL(ptr+10, NULL, 10);
		g_device_config.ant_delay = antDist2antDelay(ant_delay_dist);
		GLOBAL_PRINT(("ant_delay cnt: %u\r\n", g_device_config.ant_delay));
		dwt_setrxantennadelay(g_device_config.ant_delay);
		dwt_settxantennadelay(g_device_config.ant_delay);
		save_device_para();
		ret = TRUE;
	} 	
	else if(!GLOBAL_STRNCASECMP(ptr, "anchor_id ", 10))
	{
		g_device_config.anchor_id = GLOBAL_STRTOUL(ptr+10, NULL, 10);
		save_device_para();
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "anchor_coord ", 13))
	{
		char *p_end;

		g_device_config.position[0] = GLOBAL_STRTOD(ptr+13, &p_end);
		g_device_config.position[1] = GLOBAL_STRTOD(p_end, &p_end);
		g_device_config.position[2] = GLOBAL_STRTOD(p_end, &p_end);

		save_device_para();
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "ref_coord ", 10))
	{
		char *p_end;

		g_device_config.ref_position[0] = GLOBAL_STRTOD(ptr+10, &p_end);
		g_device_config.ref_position[1] = GLOBAL_STRTOD(p_end, &p_end);
		g_device_config.ref_position[2] = GLOBAL_STRTOD(p_end, &p_end);

		save_device_para();
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "t2wall_d ", 9))
	{
		char *p_end;
		g_device_config.t2wall_actual_dist = GLOBAL_STRTOD(ptr+9, &p_end);

		save_device_para();
		ret = TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "t2wall_thr ", 11))
	{
		char *p_end;
		g_device_config.t2wall_threshold = GLOBAL_STRTOD(ptr+11, &p_end);

		save_device_para();
		ret = TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "t2wall_fl_num ", 14))
	{
		char *p_end;
		u8 t2wall_fl_num;
		
		reset_queue();
		t2wall_fl_num = GLOBAL_STRTOD(ptr+14, &p_end);
		if(t2wall_fl_num <= 0)
		{
			GLOBAL_PRINT(("set value is invalid,no allow zero\r\n"));
		}
		else
		{
			g_device_config.t2wall_fl_num = t2wall_fl_num;
		}

		save_device_para();
		ret = TRUE;
	}		
	else if(!GLOBAL_STRNCASECMP(ptr, "max_tag_num ", 12))
	{
		char *p_end;
		u16 max_tag_num;

		max_tag_num = GLOBAL_STRTOD(ptr+12, &p_end);
		if(max_tag_num <= 0)
		{
			GLOBAL_PRINT(("set value is invalid,no allow zero\r\n"));
		}
		else
		{
			g_device_config.max_allow_tag_num = max_tag_num;
		}

		save_device_para();
		ret = TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "dyn_slot_long ", 14))
	{
		char *p_end;
		u16 dyn_slot_long;

		dyn_slot_long = GLOBAL_STRTOD(ptr+14, &p_end);
		if(dyn_slot_long <= 0)
		{
			GLOBAL_PRINT(("set value is invalid,no allow zero\r\n"));
		}
		else
		{
			g_device_config.dyn_slot_long = dyn_slot_long;
		}

		save_device_para();
		ret = TRUE;
	}		
	else if(!GLOBAL_STRNCASECMP(ptr, "rg_slot_long ", 13))
	{
		char *p_end;
		u16 rg_slot_long;

		rg_slot_long = GLOBAL_STRTOD(ptr+13, &p_end);
		if(rg_slot_long <= 0)
		{
			GLOBAL_PRINT(("set value is invalid,no allow zero\r\n"));
		}
		else
		{
			g_device_config.ranging_slot_long = rg_slot_long;
		}

		save_device_para();
		ret = TRUE;
	}		


	
	else if(!GLOBAL_STRNCASECMP(ptr, "sig_qa_thr ", 11))
	{
		char *p_end;
		g_device_config.sig_qa_thr = GLOBAL_STRTOD(ptr+11, &p_end);

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
		}
		else
		{
			ERR_PRINT(("tx_power config is failed\r\n"));
		}

		ret = TRUE;
				
	}

	else if(!GLOBAL_STRNCASECMP(ptr, "anchor_idle_num ", 16))
	{
		g_device_config.anchor_idle_num = GLOBAL_STRTOUL(ptr+16, NULL, 10);
		if(g_device_config.anchor_idle_num < 1 || g_device_config.anchor_idle_num > 5)
		{
			g_device_config.anchor_idle_num = 2;
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
				GLOBAL_PRINT(("stop efs is suc\r\n"));
			}
			else
			{
				GLOBAL_PRINT(("stop efs is fail,osThreadSuspend return %d\r\n", status));
				return TRUE;
			}			
			
			g_device_config.chan = chan;
			save_device_para();			
			DWM1000_init();

			status = osThreadResume(thread_UWB_id);
			if(status == osOK)
			{
				GLOBAL_PRINT(("start efs is suc\r\n"));
			}
			else
			{
				GLOBAL_PRINT(("start efs is fail,osThreadResume return %d\r\n", status));
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
	else if(!GLOBAL_STRNCASECMP(ptr, "mqttip ", 7))//MQTT修改域名/解析
	{
		U8 ip_buf[NET_ADDR_IP4_LEN];
		U32 timeout = 100;
		DNS_GetHost((const char*)ptr+7);//testing
		while(timeout--)
		{
			if(DNS_buff.dns_flag)
			{
				netIP_aton((const char*)DNS_buff.dns_ip, NET_ADDR_IP4, ip_buf);
				if(net_ip_is_legal(ip_buf))
				{
					GLOBAL_MEMSET(main_handle_g.cfg.net.server_ip,0x0,sizeof(main_handle_g.cfg.net.server_ip));
					GLOBAL_MEMCPY(main_handle_g.cfg.net.server_ip,ip_buf,sizeof(main_handle_g.cfg.net.server_ip));
					sys_para_save();
					GLOBAL_PRINT(("MQTT_server IP set up and save\r\n"));
					GLOBAL_PRINT(("MQTT_now Reconnecting!!\r\n"));
					MQTT_OFF_LINE = TRUE;
	                iotDev.sta = IOT_STA_INIT;
					iotRTCMDev.sta = IOT_STA_INIT;
					iotCmdDev.sta= IOT_STA_INIT;
				}
				break;
			}
			delay_ms(100);
			if(timeout<=1)
			{
				ERR_PRINT(("MQTT IP set Time out!!\r\n"));
			}
		}
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "mqttport ", 9))
	{
		main_handle_g.cfg.net.server_port = GLOBAL_STRTOUL(ptr+9, NULL, 10);
		sys_para_save();
		GLOBAL_PRINT(("MQTT_server Port set up and save\r\n"));
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "rtkip ", 6))
	{
		U8 ip_buf[NET_ADDR_IP4_LEN];
		U32 timeout = 100;
		DNS_GetHost((const char*)ptr+6);//testing
		while(timeout--)
		{
			if(DNS_buff.dns_flag)
			{
				netIP_aton((const char*)DNS_buff.dns_ip, NET_ADDR_IP4, ip_buf);
				if(net_ip_is_legal(ip_buf))
				{
					GLOBAL_MEMSET(main_handle_g.cfg.net.ntrip_ip,0x0,sizeof(main_handle_g.cfg.net.ntrip_ip));
					GLOBAL_MEMCPY(main_handle_g.cfg.net.ntrip_ip,ip_buf,sizeof(main_handle_g.cfg.net.ntrip_ip));
					sys_para_save();
					GLOBAL_PRINT(("RTK_ntrip IP set up and save\r\n"));
					GLOBAL_PRINT(("RTK_ntrip now Reconnecting!!\r\n"));
					Ntrip_State = NTRIP_INIT;
				}
			}
			delay_ms(100);
			if(timeout<=1)
			{
				ERR_PRINT(("RTK IP set Time out!!\r\n"));
			}
		}
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "rtkport ", 8))
	{
		main_handle_g.cfg.net.ntrip_port = GLOBAL_STRTOUL(ptr+8, NULL, 10);
		sys_para_save();
		GLOBAL_PRINT(("RTK_ntrip Port set up and save\r\n"));
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "MSip ", 5))
	{
		U8 ip_buf[NET_ADDR_IP4_LEN];

		netIP_aton((const char*)ptr+5, NET_ADDR_IP4, ip_buf);
		if(net_ip_is_legal(ip_buf))
		{			
			GLOBAL_MEMSET(main_handle_g.cfg.net.MS_ip,0x0,sizeof(main_handle_g.cfg.net.MS_ip));
			GLOBAL_MEMCPY(main_handle_g.cfg.net.MS_ip,ip_buf,sizeof(main_handle_g.cfg.net.MS_ip));
			sys_para_save();
			GLOBAL_PRINT(("UDP IP set up and save\r\n"));
		}
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "MSport ", 7))
	{
		main_handle_g.cfg.net.MS_port = GLOBAL_STRTOUL(ptr+7, NULL, 10);
		sys_para_save();
		GLOBAL_PRINT(("UDP Port set up and save\r\n"));
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "mount ", 6))
	{
		GLOBAL_MEMSET(main_handle_g.cfg.net.Ntrip_mount,0x0,strlen(main_handle_g.cfg.net.Ntrip_mount));
		GLOBAL_MEMCPY(main_handle_g.cfg.net.Ntrip_mount,ptr+6,strlen(ptr+6));
		GLOBAL_PRINT(("set Ntrip_Mount = %s\r\n",main_handle_g.cfg.net.Ntrip_mount));
		sys_para_save();
		ret = TRUE;
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "userpass ", 9))
	{
		GLOBAL_MEMSET(main_handle_g.cfg.net.Ntrip_usr_pass,0x0,sizeof(main_handle_g.cfg.net.Ntrip_usr_pass));
		GLOBAL_MEMCPY(main_handle_g.cfg.net.Ntrip_usr_pass,ptr+9,strlen(ptr+9));
		GLOBAL_PRINT(("set Ntrip_user_pass = %s\r\n",main_handle_g.cfg.net.Ntrip_usr_pass));
		sys_para_save();
		ret = TRUE;
	}


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
	else if(!GLOBAL_STRCASECMP(ptr, "post_para"))
	{
		show_position_para();
		ret = TRUE;
	}
	else if(!GLOBAL_STRCASECMP(ptr, "anchor_state"))
	{
		show_anchor_state();
		ret = TRUE;
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
		if(!GLOBAL_STRCASECMP(ptr+3, "efs_post"))
		{
			sys_debug_set_type(SYS_DEBUG_EFS_POST);
		}
		else if(!GLOBAL_STRCASECMP(ptr+3, "efs_gen"))
		{
			sys_debug_set_type(SYS_DEBUG_EFS_GENERAL);
		}	
		else if(!GLOBAL_STRCASECMP(ptr+3, "efs_err"))
		{
			sys_debug_set_type(SYS_DEBUG_EFS_ERR);
		}		
		else if(!GLOBAL_STRCASECMP(ptr+3, "efs_post_note"))
		{
			sys_debug_set_type(SYS_DEBUG_EFS_POST_NOTE);
		}		
		else if(!GLOBAL_STRCASECMP(ptr+3, "efs_rssi"))
		{
			sys_debug_set_type(SYS_DEBUG_EFS_RSSI);
		}		
		else if(!GLOBAL_STRCASECMP(ptr+3, "efs_rg"))
		{
			sys_debug_set_type(SYS_DEBUG_EFS_RANGING);
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
		else if(!GLOBAL_STRCASECMP(ptr+3, "rtk"))
	    {
	        sys_debug_set_type(SYS_DEBUG_RTK);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "GGA"))
	    {
	        sys_debug_set_type(SYS_DEBUG_GGA);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "RMC"))
	    {
	        sys_debug_set_type(SYS_DEBUG_RMC);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "ENH"))
	    {
	        sys_debug_set_type(SYS_DEBUG_ENHANCE);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "IMU"))
	    {
	        sys_debug_set_type(SYS_DEBUG_IMU);
	    }
	    else if(!GLOBAL_STRCASECMP(ptr+3, "IMURAW"))
	    {
	        sys_debug_set_type(SYS_DEBUG_IMU_RAW);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+3, "all"))
		{
			sys_debug_set_type(SYS_DEBUG_ENHANCE);
			sys_debug_set_type(SYS_DEBUG_GGA);
			sys_debug_set_type(SYS_DEBUG_RMC);
			sys_debug_set_type(SYS_DEBUG_RTK);
		}
		else
		{
			ret = FALSE;
		}		
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "off ", 4))
	{
		ret = TRUE;
		if(!GLOBAL_STRCASECMP(ptr+4, "efs_post"))
		{
			sys_debug_clear_type(SYS_DEBUG_EFS_POST);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "efs_gen"))
		{
			sys_debug_clear_type(SYS_DEBUG_EFS_GENERAL);
		}	
		else if(!GLOBAL_STRCASECMP(ptr+4, "efs_err"))
		{
			sys_debug_clear_type(SYS_DEBUG_EFS_ERR);
		}		
		else if(!GLOBAL_STRCASECMP(ptr+4, "efs_post_note"))
		{
			sys_debug_clear_type(SYS_DEBUG_EFS_POST_NOTE);
		}		
		else if(!GLOBAL_STRCASECMP(ptr+4, "efs_rssi"))
		{
			sys_debug_clear_type(SYS_DEBUG_EFS_RSSI);
		}		
		else if(!GLOBAL_STRCASECMP(ptr+4, "efs_rg"))
		{
			sys_debug_clear_type(SYS_DEBUG_EFS_RANGING);
		}		
    	else if(!GLOBAL_STRCASECMP(ptr+4, "server"))
		{
			sys_debug_clear_type(SYS_DEBUG_SERVER);
		}
		else if(!GLOBAL_STRCASECMP(ptr+4, "rtk"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_RTK);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+4, "GGA"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_GGA);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+4, "RMC"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_RMC);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+4, "ENH"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_ENHANCE);
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
		else if(!GLOBAL_STRCASECMP(ptr+4, "IMU"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_IMU);
	    }
		else if(!GLOBAL_STRCASECMP(ptr+4, "IMURAW"))
	    {
	        sys_debug_clear_type(SYS_DEBUG_IMU_RAW);
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
    save_device_para();
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
	else if(!GLOBAL_STRNCASECMP(ptr, "open filter", 11))
	{	
		g_open_filter_flag = 1;
			
		return TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "close filter", 12))
	{		
		g_open_filter_flag = 0;
			
		return TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "open rg", 7))
	{	
		g_ranging_flag = 1;
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "close rg", 8))
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
	else if(!GLOBAL_STRNCASECMP(ptr, "open tx_power", 13))
	{	
		char *p_end;
		
		g_dwt_auto_tx_power_config.auto_choose_tx_power = 1;
		g_dwt_auto_tx_power_config.main_device_rssi_threshold = GLOBAL_STRTOD(ptr+13, &p_end);
		g_dwt_auto_tx_power_config.sub_device_rssi_threshold = GLOBAL_STRTOD(p_end, &p_end);
		g_dwt_auto_tx_power_config.main_device_rssi_rang = GLOBAL_STRTOD(p_end, &p_end);
		g_dwt_auto_tx_power_config.sub_device_rssi_rang = GLOBAL_STRTOD(p_end, &p_end);
		
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
			
		return TRUE;
	}	
	else if(!GLOBAL_STRNCASECMP(ptr, "close tx_power", 14))
	{		
		g_dwt_auto_tx_power_config.auto_choose_tx_power = 0;
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
				
		return TRUE;
	}		
	else if(!GLOBAL_STRNCASECMP(ptr, "open efs", 8))
	{ 	
		osStatus_t status = osThreadResume(thread_UWB_id);
		if(status == osOK)
		{
			GLOBAL_PRINT(("open efs is suc\r\n"));
			return TRUE;
		}
		else
		{
			GLOBAL_PRINT(("open efs is fail,osThreadResume return %d\r\n", status));
		}				
	} 
	else if(!GLOBAL_STRNCASECMP(ptr, "close efs", 9))
	{
		osStatus_t status = osThreadSuspend(thread_UWB_id);
		if(status == osOK)
		{
			GLOBAL_PRINT(("close efs is suc\r\n"));
			return TRUE;
		}
		else
		{
			GLOBAL_PRINT(("close efs is fail,osThreadSuspend return %d\r\n", status));
		}		
	}
	else if(!GLOBAL_STRNCASECMP(ptr, "open ant_cal", 12))
	{ 
		char *p_end;
		g_dwt_auto_ant_cal.salve_ant_cal_en = GLOBAL_STRTOUL(ptr+12, &p_end, 10);
		g_dwt_auto_ant_cal.actual_dist_cm = GLOBAL_STRTOUL(p_end, &p_end, 10);
		g_dwt_auto_ant_cal.auto_ant_cal_open = 1;	
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
		return TRUE;
	} 
	else if(!GLOBAL_STRNCASECMP(ptr, "close ant_cal", 13))
	{		
		g_dwt_auto_ant_cal.auto_ant_cal_open = 0;
		osEventFlagsSet(evt_id, DM1000_TXRX_FLAG);
		return TRUE;
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
	else if(!GLOBAL_STRNCASECMP(ptr, "ntrip reboot", 12))
	{
		Ntrip_State = NTRIP_INIT;
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
		//osThreadYield();
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
