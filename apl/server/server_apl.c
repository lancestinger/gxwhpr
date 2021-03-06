#include "server_apl.h"
#include "json/cJSON.h"
#include "iotclient/iotclient.h"
#include "uwb_post/uwb_post.h"
#include "fml/gnss/nmea.h"
#include "drv/log.h"
#include "drv/socket/socket_drv.h"
#include "update/update_apl.h"
#include "fml/sshell/sshell_fml.h"
#include "fml/uart/uart_fml.h"
#include "drv/rngbuf/rngbuf.h"
#include "apl/imu/Coordi_transfer.h"
#include "apl/EH_uwb/EH_uwb.h"

Server_Data server_Data;
Pos_count Pos_time;
Server_Ori_Data serOriData;


static u8 g_mqtt_print_tmpbuf[1024];
static u8 g_mqtt_print_buf[1300];
//static u8 g_mqtt_print_tmpbuf[2048];
//static u8 g_mqtt_print_buf[2200];


extern UPDATE_HANDLE update_handle_g;
int SERVER_RTK=0; 				  /* RTK数据Ready标志 */
int SERVER_UWB=0; 				  /* UWB数据Ready标志 */
int SERVER_GNSS=0;                /* UWB数据Ready标志 */
int SERVER_ERROR=0;			      /* 定位数据错误标志 */

double UWB_angle = 0;//UWB推算角度
double UWB_Veloc = 0;//UWB推算速度

U8 UPLOAD_VAL=0;                  /*上报标志*/

static WGS LLA_pos;
static WGS ORIGN_pos;
static ECEF ECEF_pos;
static ENU ENU_pos;


// 全局常量定义
//const char * base64char = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";//标准base64表
const char * base64char = BASE64_LIST;

/*
U8 UDP_DST[5]={
						192,
						168,
						10,
						158,
				};
int UDP_PORT = 8088;
*/
static U64 thread_Server_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_Server_attr = {
  .stack_mem  = &thread_Server_stk[0],
  .stack_size = sizeof(thread_Server_stk),
  .priority = osPriorityNormal,
};

/*****************************************************************************
 函 数 名  : print_preallocated
 功能描述  : MQTT上报数据传输函数
 输入参数  : cJSON *root  
 输出参数  : 无
 返 回 值  : int
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年10月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
/* Create a bunch of objects as demonstration. */
static int print_preallocated(IOT_Device_t* iotDev, cJSON *root)
{
    /* declarations */
    char *out = NULL;
    char *buf = NULL;
	char *base64_buf = NULL;
    size_t len = 0;

    /* formatted print */
    out = cJSON_PrintUnformatted(root);//cJSON_Print

    /* create buffer to succeed */
    /* the extra 5 bytes are because of inaccuracies when reserving memory */
    len = strlen(out) + 5;
    buf = (char*)malloc(len);
    if (buf == NULL)
    {
        GLOBAL_PRINT(("Failed to allocate memory.\n"));
        //exit(1);
    }

    /* Print to buffer */
    if (!cJSON_PrintPreallocated(root, buf, (int)len, 0)) 
    {
        GLOBAL_PRINT(("cJSON_PrintPreallocated failed!\n"));
        if(strcmp(out, buf) != 0)
        {
            GLOBAL_PRINT(("cJSON_PrintPreallocated not the same as cJSON_Print!\n"));
            GLOBAL_PRINT(("cJSON_Print result:\n%s\n", out));
            GLOBAL_PRINT(("cJSON_PrintPreallocated result:\n%s\n", buf));
        }
        free(out);
        free(buf);
        return -1;
    }

    DBG_MQTT_Print("json str:%s\r\n",buf);
	base64_buf = (char*)malloc(len*2);
	if (base64_buf == NULL)
    {
        GLOBAL_PRINT(("Allocate base64_buf failed.\n"));
    }
	else
	{
		//len = base64_encode(buf,base64_buf,len-5);
	}
	//iotclient_publish(iotDev, base64_buf, len);
    iotclient_publish(iotDev, buf, len-5);

	free(base64_buf);
    free(out);
    free(buf);
    return 0;
}


/*****************************************************************************
 函 数 名  : upload_hpr_state_to_server
 功能描述  : 上报接收机状态函数(心跳包)
 输入参数  : int mode
 			* RTK_MODE
 			* UWB_MODE
 			* GNSS_MODE
 			* DATA_ERROR
 输出参数  : 无
 返 回 值  : U8
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月
    作    者   : sunj
    修改内容   : 创建
  1.日	  期   : 2021年3月
    作	  者   : zxf
    修改内容   : 增加模式选择，重组协议

*****************************************************************************/
//向服务器上传标签状态
U8 upload_hpr_state_to_server(void)
{
    cJSON *root = NULL;
    cJSON *data_obj = NULL;
	char version_str[30]={0};
	static u32 cnt_num = 0;

	if(g_device_config.device_type != TAG)
		return TRUE;
	
	cnt_num++;
		
    root = cJSON_CreateObject();
    data_obj = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "cmd", 601);

	cJSON_AddNumberToObject(data_obj, "latitude", server_Data.latitude);
	cJSON_AddNumberToObject(data_obj, "longitude", server_Data.longitude);
	cJSON_AddNumberToObject(data_obj, "altitude", server_Data.height);
	cJSON_AddNumberToObject(data_obj, "mode", server_Data.mode);
	cJSON_AddNumberToObject(data_obj, "fpRxRssiDiffThr", g_device_config.sig_qa_thr);
	cJSON_AddNumberToObject(data_obj, "rssi", g_uwb_rg_ssi);
	
	if(cnt_num%10 == 0)
	{
		sprintf(version_str,"GXWHPR_v%u.%u.%u",SW_VERSION_H,SW_VERSION_M,SW_VERSION_L);
		cJSON_AddStringToObject(data_obj, "version",(char*)version_str);
		cJSON_AddNumberToObject(data_obj, "tagH", g_device_config.tag_h);
		cJSON_AddNumberToObject(data_obj, "txPower", calc_tx_power_config_value_to_db(g_device_config.tx_power));
		cJSON_AddNumberToObject(data_obj, "id", TAG_ID_SUM);
		cJSON_AddNumberToObject(data_obj, "iTranAntDelay", g_device_config.ant_delay);  
		cJSON_AddNumberToObject(data_obj, "iRecvAntDelay", g_device_config.ant_delay);
		cJSON_AddStringToObject(data_obj, "ms_id",(char*)UDPBuff.RX_pData);
	}
	
    cJSON_AddItemToObject(root, "data", data_obj);   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    /* Print to text */
    if(print_preallocated(&iotDev,root) != 0)
    {
        cJSON_Delete(root);
        return FALSE;
    }
    cJSON_Delete(root);

    return TRUE;
}

/*****************************************************************************
 函 数 名  : upload_hpr_cmd_feedback_to_server
 功能描述  : 服务器命令回复回执函数
 输入参数  : Server_Ori_Data data, int state 
 输出参数  : 无
 返 回 值  : U8
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
static U8 upload_hpr_cmd_feedback_to_server(Server_Ori_Data data, int state)
{
    cJSON *root = NULL;
    cJSON *data_obj = NULL;


    root = cJSON_CreateObject();
    data_obj = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "cmd", data.cmd);
    cJSON_AddNumberToObject(root, "response", (data.response - 1));
    cJSON_AddStringToObject(root, "task_uuid", data.task_uuid);
    cJSON_AddStringToObject(root, "cmd_uuid", data.cmd_uuid);
    
    cJSON_AddNumberToObject(data_obj, "iState", state);

    cJSON_AddItemToObject(root, "data", data_obj);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    /* Print to text */
    if(print_preallocated(&iotDev,root) != 0)
    {
        cJSON_Delete(root);
        //ERR_PRINT(("send Set Position Cmd Feedback Error!!!!\r\n"));
        return FALSE;
    }
    cJSON_Delete(root);

    return TRUE;
}


/*****************************************************************************
 函 数 名  : upload_hpr_update_feedback_to_server
 功能描述  : 向服务器发送升级结果
 输入参数  : int state 
 
 * state: 
 * 0：升级成功
 * 1：操作超时
 * 2：登录错误,用户名/密码无效
 * 3：文件不允许访问
 * 4：文件未发现
 * 5：工作路径未发现
 * 6：本地文件读/写错误
 * 7：FTP客户端错误
 * 8：校验失败
 
 输出参数  : 无
 返 回 值  : U8
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
U8 upload_hpr_update_feedback_to_server(Server_Ori_Data data,int state)
{
    cJSON *root = NULL;
    cJSON *data_obj = NULL;

    root = cJSON_CreateObject();
    data_obj = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "cmd", 102);
    cJSON_AddNumberToObject(root, "response", (data.response - 1));
    cJSON_AddStringToObject(root, "task_uuid", data.task_uuid);
    cJSON_AddStringToObject(root, "cmd_uuid", data.cmd_uuid);

    
    cJSON_AddNumberToObject(data_obj, "status", state);

    cJSON_AddItemToObject(root, "data", data_obj);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    /* Print to text */
    if(print_preallocated(&iotDev,root) != 0)
    {
        cJSON_Delete(root);
        return FALSE;
    }
    cJSON_Delete(root);

    return TRUE;    
}

U8 upload_hpr_cmd_log_to_server(void)
{
    cJSON *root = NULL;
    cJSON *data_obj = NULL;
		S32 len, data_len;	
		static S32 last_remaining_len = 0;	
		S32 i;	
		static U8 *p_tmep;
		extern RNGBUF_t * print_rngbuf;	
		U32 time;
		S8 buf[15] = {0};
	 
		time = get_cur_time() / 1000000;

		 if(uart_get_dbg_outmode() == DBG_OUT_MQTT)
		 {		 
			 if(!rng_is_empty(print_rngbuf))
			 {	 		
					len = rng_get_buf(print_rngbuf, (char*)g_mqtt_print_tmpbuf, 1024);
					if(g_mqtt_print_tmpbuf[len-2] != '\r' && g_mqtt_print_tmpbuf[len-1] != '\n')
					{
						for(i=(len-1); i >= 1; i--)
						{
							if(g_mqtt_print_tmpbuf[i-1] == '\r' && g_mqtt_print_tmpbuf[i] == '\n')
							{							
								data_len = i+1;
								p_tmep = g_mqtt_print_buf + last_remaining_len;		
								memcpy(p_tmep, g_mqtt_print_tmpbuf, data_len);
								data_len += last_remaining_len;
								if(data_len >= 1300)
								{
									data_len = 1300 - 1;
								}

								g_mqtt_print_buf[data_len] = 0;

								last_remaining_len = len - i - 1;
								break;																				
							}
						}
					}
					else
					{
							p_tmep = g_mqtt_print_buf + last_remaining_len;		
							memcpy(p_tmep, g_mqtt_print_tmpbuf, len);
							data_len = last_remaining_len + len;
							if(data_len >= 1300)
							{
								data_len = 1300- 1;
							}						
							g_mqtt_print_buf[data_len] = 0;
							last_remaining_len = 0;
					}
						
					
			 }
			 else
			 {
					return TRUE;
			 }

	    root = cJSON_CreateObject();

	 		sprintf(buf, "%u", time);
			cJSON_AddStringToObject(root, "time", buf);
			cJSON_AddStringToObject(root, "log", g_mqtt_print_buf);	
	                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
	    /* Print to text */
	    if(print_preallocated(&iotCmdDev, root) != 0)
	    {
	        cJSON_Delete(root);
	        //ERR_PRINT(("send Set Position Cmd Feedback Error!!!!\r\n"));
	        return FALSE;
	    }
	    cJSON_Delete(root);

			if(last_remaining_len > 0)
				memcpy(g_mqtt_print_buf, &g_mqtt_print_tmpbuf[i+1], last_remaining_len);

		}


    return TRUE;
}




/*****************************************************************************
 函 数 名  : upload_efs_all_para_to_server
 功能描述  : 向服务器发送命令反馈所有参数信息函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : U8
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月
    作    者   : wangxuan
    修改内容   : 创建

*****************************************************************************/
static U8 upload_hpr_all_para_to_server(void)
{
    cJSON *root = NULL;
    cJSON *data_obj = NULL;
	char temp_str[30] = {0};
	
    root = cJSON_CreateObject();
    data_obj = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "cmd", 601);	

	sprintf(temp_str,"GXWHPR_v%u.%u.%u",SW_VERSION_H,SW_VERSION_M,SW_VERSION_L);
	cJSON_AddStringToObject(data_obj, "SOFTWARE VERSION", (char*)temp_str);
	cJSON_AddStringToObject(data_obj, "HARDWARE VERSION", HW_VERSION);


	if(g_device_config.device_type == ANCHOR)
	{
		cJSON_AddNumberToObject(data_obj, "id", g_device_config.anchor_id);	
	}
	else
	{	
		cJSON_AddNumberToObject(data_obj, "id", g_device_config.tag_id); 
		cJSON_AddNumberToObject(data_obj, "tagH", g_device_config.tag_h);
	}
	
	cJSON_AddNumberToObject(data_obj, "lat", g_device_config.position[0]);
	cJSON_AddNumberToObject(data_obj, "lon", g_device_config.position[1]);  
    cJSON_AddNumberToObject(data_obj, "alt", g_device_config.position[2]);
	cJSON_AddNumberToObject(data_obj, "device_type", g_device_config.device_type);
	cJSON_AddNumberToObject(data_obj, "iTranAntDelay", g_device_config.ant_delay);  
    cJSON_AddNumberToObject(data_obj, "iRecvAntDelay", g_device_config.ant_delay);		 
    //cJSON_AddNumberToObject(data_obj, "txPower", g_device_config.tx_power);
	cJSON_AddNumberToObject(data_obj, "txPower", calc_tx_power_config_value_to_db(g_device_config.tx_power));
	cJSON_AddNumberToObject(data_obj, "chan", g_device_config.chan);
	cJSON_AddNumberToObject(data_obj, "orientation", g_device_config.on_left);
	cJSON_AddNumberToObject(data_obj, "dyn_slot_long", g_device_config.dyn_slot_long);
	cJSON_AddNumberToObject(data_obj, "ranging_slot_long", g_device_config.ranging_slot_long);		

	
	netIP_ntoa(NET_ADDR_IP4, main_handle_g.cfg.net.net_ip, temp_str, 16);
	cJSON_AddStringToObject(data_obj, "IP ADDR", temp_str);
	netIP_ntoa(NET_ADDR_IP4, main_handle_g.cfg.net.net_mask, temp_str, 16);
	cJSON_AddStringToObject(data_obj, "MASK ADDR", temp_str);
	netIP_ntoa(NET_ADDR_IP4, main_handle_g.cfg.net.net_gateway, temp_str, 16);
	cJSON_AddStringToObject(data_obj, "GATE ADDR", temp_str);
	netMAC_ntoa(main_handle_g.cfg.net.net_mac, temp_str, 20);
	cJSON_AddStringToObject(data_obj, "MAC ADDR", temp_str);

    cJSON_AddItemToObject(root, "data", data_obj);   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    /* Print to text */
    if(print_preallocated(&iotDev,root) != 0)
    {
        cJSON_Delete(root);
        return FALSE;
    }
    cJSON_Delete(root);

    return TRUE;
}


/*****************************************************************************
 函 数 名  : parse_server_data
 功能描述  : 解析服务器下发数据函数
 输入参数  : U8* buf, int len 
 输出参数  : 无
 返 回 值  : U8
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年12月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
U8 parse_server_data(U8* buf, int len)
{
    cJSON *receive_obj=NULL;
    cJSON *item_obj=NULL;
    cJSON *cmd_obj=NULL;
    cJSON *data_obj=NULL;
    U8 ip_buf[NET_ADDR_IP4_LEN];

    int cmd = 0;

	GLOBAL_MEMSET(&serOriData, 0x0, sizeof(serOriData));

    receive_obj = cJSON_Parse((char *)buf);
    if(receive_obj==NULL)
    {
        ERR_PRINT(("MQTT cmd receive_obj is NULL,str:%s\r\n",buf));
        return FALSE;
    }
	else
	{
		GLOBAL_PRINT(("receive_obj is str:%s\r\n",buf));
	}

    cmd_obj = cJSON_GetObjectItem(receive_obj,"cmd");
    if(cmd_obj)
    {
        cmd = cmd_obj->valueint;
		serOriData.cmd = cmd_obj->valueint;
        GLOBAL_PRINT(("cmd=%d\r\n",cmd));
    }
    else
    {
        DBG_MQTT_Print("未提取到cmd对象!!!\r\n");
        cJSON_Delete(receive_obj);
        return FALSE;
    }

	serOriData.response = cJSON_GetObjectItem(receive_obj,"response")->valueint;
    strcpy(serOriData.task_uuid, cJSON_GetObjectItem(receive_obj,"task_uuid")->valuestring);
    strcpy(serOriData.cmd_uuid, cJSON_GetObjectItem(receive_obj,"cmd_uuid")->valuestring);
    
    data_obj = cJSON_GetObjectItem(receive_obj,"data");
	
    switch(cmd)
    {
        case 100://开始升级
            item_obj = data_obj->child;
            while(item_obj)
            {
                if(!strcmp(item_obj->string,"ftpServer"))
                {
                    strcpy(update_handle_g.aFtpServerAddr, cJSON_GetObjectItem(data_obj,"ftpServer")->valuestring);
                }
                else if(!strcmp(item_obj->string,"userName"))
                {
                    strcpy(update_handle_g.userName, cJSON_GetObjectItem(data_obj,"userName")->valuestring);
                }
                else if(!strcmp(item_obj->string,"password"))
                {
                    strcpy(update_handle_g.password, cJSON_GetObjectItem(data_obj,"password")->valuestring);
                }
                else if(!strcmp(item_obj->string,"workDir"))
                {
                    strcpy(update_handle_g.workDir, cJSON_GetObjectItem(data_obj,"workDir")->valuestring);
                }
                else if(!strcmp(item_obj->string,"fileName"))
                {
                    strcpy(update_handle_g.fileName, cJSON_GetObjectItem(data_obj,"fileName")->valuestring);
                }
                
                item_obj = item_obj->next;
            }
            
            netIP_aton((const char*)update_handle_g.aFtpServerAddr, NET_ADDR_IP4, ip_buf);
            if(net_ip_is_legal(ip_buf))
            {
                update_handle_g.ftpServerAddr[0] = ip_buf[0];
                update_handle_g.ftpServerAddr[1] = ip_buf[1];
                update_handle_g.ftpServerAddr[2] = ip_buf[2];
                update_handle_g.ftpServerAddr[3] = ip_buf[3];
            }
            else
            {
                ERR_PRINT(("ftpServer格式非法!!!!\r\n"));
            }

            update_handle_g.updateSta = updateSta_EventDownloadStart;
        break;

        case 103://停止升级
        break;

        case 602://重启
            //发送重启命令反馈
            upload_hpr_cmd_feedback_to_server(serOriData,1);
			MQTT_OFF_LINE = TRUE;
            NOTE_PRINT(("\r\nPlease waiting for system rebooting!!!!!!\r\n\r\n"));
            //delay_ms(1000);
            NVIC_SystemReset();
        break;

        case 603://参数设置
            item_obj = data_obj->child;
 
            while(item_obj)
            {	
				if(!strcmp(item_obj->string,"tagH"))
                {
					g_device_config.tag_h = item_obj->valuedouble;
					save_device_para();
                }		
				else if(!strcmp(item_obj->string,"txPower"))
                {
					u8 buf[10] = {0};
					sprintf(buf,"%2.1f",item_obj->valuedouble);										
                	g_device_config.tx_power = calc_tx_power_config_value(buf);
					save_device_para();
                }
                else if(!strcmp(item_obj->string,"iTranAntDelay"))
                {
					g_device_config.ant_delay = item_obj->valueint;
					dwt_settxantennadelay(g_device_config.ant_delay);
					save_device_para();
                }
				else if(!strcmp(item_obj->string,"iRecvAntDelay"))
                {
					g_device_config.ant_delay = item_obj->valueint;
					dwt_setrxantennadelay(g_device_config.ant_delay);
					save_device_para();
                }
									
                item_obj = item_obj->next;
				//DBG_MQTT_Print("item_obj = %s\r\n",item_obj->string);
            }
			
			GLOBAL_INTVAL(g_device_config.ant_delay);
			GLOBAL_FLTVAL(g_device_config.tag_h);
			GLOBAL_HEX(g_device_config.tx_power);
			
            upload_hpr_cmd_feedback_to_server(serOriData,1);
        break;

		case 10000:
			item_obj = data_obj->child;
			sshell_excute_cmd(item_obj->string);
			upload_hpr_cmd_feedback_to_server(serOriData,1);
			break;
			
		case 10001:
			upload_hpr_all_para_to_server();
			break;

        default:
            WARN_PRINT(("未知cmd[%d]!!!\r\n",cmd));
        break;
    }

    cJSON_Delete(receive_obj);
	
	return TRUE;
}


U8 Cmd_parse_data(U8* buf, int len)
{
	cJSON *receive_obj=NULL;
    cJSON *item_obj=NULL;
    cJSON *cmd_obj=NULL;
    cJSON *data_obj=NULL;
    
    Server_Ori_Data serOriData;
    GLOBAL_MEMSET(&serOriData, 0x0, sizeof(serOriData));

    receive_obj = cJSON_Parse((char *)buf);
    if(receive_obj==NULL)
    {
        ERR_PRINT(("receive_obj is NULL,str:%s\r\n",buf));
        return FALSE;
    }

	uart_set_dbg_outmode(DBG_OUT_MQTT);
	
	cmd_obj = cJSON_GetObjectItem(receive_obj,"cmd");
	if(cmd_obj)
	{			
		DBG_MQTT_Print("cmd: %s\r\n", cmd_obj->valuestring);
		sshell_excute_cmd(cmd_obj->valuestring);
		cJSON_Delete(receive_obj);
		return TRUE;
	}
	else
	{
		WARN_PRINT(("未提取到cmd对象!!!\r\n"));
		cJSON_Delete(receive_obj);
		return FALSE;
	}
	
	

}

/*****************************************************************************
 函 数 名  : TCP_preallocated
 功能描述  : TCP上报数据传输函数
 输入参数  : cJSON *root  
 输出参数  : 无
 返 回 值  : int
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年4月
    作    者   : zxf
    修改内容   : 创建

*****************************************************************************/
static int TCP_preallocated(cJSON *root)
{
    /* declarations */
    char *out = NULL;
    char *buf = NULL;
    int len = 0;
	int len_out = 0;
	int Over_time_send=0;

    /* formatted print */
    out = cJSON_PrintUnformatted(root);//cJSON_Print

    /* create buffer to succeed */
    /* the extra 5 bytes are because of inaccuracies when reserving memory */
	len_out = strlen(out);
    len = strlen(out)+5;
    buf = (char*)malloc(len);
    if (buf == NULL)
    {
        WARN_PRINT(("Failed to allocate memory.\n"));
    }

    /* Print to buffer */
    if (!cJSON_PrintPreallocated(root, buf, (int)len, 0)) 
    {
        WARN_PRINT(("cJSON_PrintPreallocated failed!\n"));
        if(strcmp(out, buf) != 0)
        {
            WARN_PRINT(("cJSON_PrintPreallocated not the same as cJSON_Print!\n"));
            WARN_PRINT(("cJSON_Print result:\n%s\n", out));
            WARN_PRINT(("cJSON_PrintPreallocated result:\n%s\n", buf));
        }
        free(out);
        free(buf);
        return -1;
    }
    GLOBAL_PRINT(("json str:%s\r\n",buf));

	while(0 >= socket_TCP_send_msg(SOCKET_4,(U8*)out,len_out,0))
	{
		Over_time_send++;
		GLOBAL_PRINT(("Sending EH_UWB...\r\n"));
		delay_ms(200);
		if(Over_time_send>=3)
		{
			break;
		}
	}
	if(Over_time_send >= 3)
	{
		Over_time_send = 0;
		GLOBAL_PRINT(("Send EH_UWB error, Socket 4 Restart!!\r\n"));
		free(out);
        free(buf);
        return -1;
	}
    
    free(out);
    free(buf);
    return 0;
}



/*****************************************************************************
 函 数 名  : UDP_preallocated
 功能描述  : UDP上报数据传输函数
 输入参数  : cJSON *root  
 输出参数  : 无
 返 回 值  : int
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月
    作    者   : zxf
    修改内容   : 创建

*****************************************************************************/
static int UDP_preallocated(cJSON *root)
{
    /* declarations */
    char *out = NULL;
    char *buf = NULL;
    size_t len = 0;
	size_t len_out = 0;

	//SOCKADDR_IN* UDP_dst = (SOCKADDR_IN*)GLOBAL_MALLOC(sizeof(SOCKADDR_IN));
	//GLOBAL_MEMSET(UDP_dst,0x0,sizeof(SOCKADDR_IN));

    /* formatted print */
    out = cJSON_PrintUnformatted(root);//cJSON_Print

    /* create buffer to succeed */
    /* the extra 5 bytes are because of inaccuracies when reserving memory */
	len_out = strlen(out);
    len = strlen(out)+5;
    buf = (char*)malloc(len);
    if (buf == NULL)
    {
        WARN_PRINT(("Failed to allocate memory.\n"));
    }

    /* Print to buffer */
    if (!cJSON_PrintPreallocated(root, buf, (int)len, 0)) 
    {
        WARN_PRINT(("cJSON_PrintPreallocated failed!\n"));
        if(strcmp(out, buf) != 0)
        {
            WARN_PRINT(("cJSON_PrintPreallocated not the same as cJSON_Print!\n"));
            WARN_PRINT(("cJSON_Print result:\n%s\n", out));
            WARN_PRINT(("cJSON_PrintPreallocated result:\n%s\n", buf));
        }
		//free(UDP_dst);
        free(out);
        free(buf);
        return -1;
    }
    //GLOBAL_PRINT(("json str:%s\r\n",buf));
	DBG_Socket_Print("UDP SENT = %s\r\n",out);

    udp_send_ms(out,len_out);

	//free(UDP_dst);
    free(out);
    free(buf);
    return 0;
}


/*****************************************************************************
 函 数 名  : UDP_upload_hpr_location
 功能描述  : UDP上报定位模式选择函数
 输入参数  : int mode   
 			* GNSS_MODE/RTK_MODE/UWB_MODE/DATA_ERROR
 输出参数  : 无
 返 回 值  : U8
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月
    作    者   : zxf
    修改内容   : 创建

*****************************************************************************/
U8 UDP_upload_hpr_location(void)
{
	cJSON *root_udp = NULL;
	static char Str[20]={0};

	if(g_device_config.device_type != TAG)
		return TRUE;
	
	GLOBAL_MEMSET(Str,0x0,20);
    root_udp = cJSON_CreateObject();
	
	cJSON_AddStringToObject(root_udp, "title","position");
    cJSON_AddStringToObject(root_udp, "date",Sys_Date);
	cJSON_AddStringToObject(root_udp, "utc",Sys_UTC);
	sprintf(Str,"%.8f",server_Data.latitude);
	cJSON_AddStringToObject(root_udp, "lat",(char*)Str);
	sprintf(Str,"%.8f",server_Data.longitude);
	cJSON_AddStringToObject(root_udp, "lon",(char*)Str);
	sprintf(Str,"%.3f",server_Data.height);
	cJSON_AddStringToObject(root_udp, "alt",(char*)Str);
	cJSON_AddNumberToObject(root_udp, "mode",server_Data.mode);
	                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    /* Print to text */
    if(UDP_preallocated(root_udp) != 0)
    {
        cJSON_Delete(root_udp);
        return FALSE;
    }
    cJSON_Delete(root_udp);

    return TRUE;

}

U8 EH_UWB_upload(netStruct netbuf)
{
	cJSON * pJsonRoot = NULL;
	cJSON * pSubJson = NULL;	  // 创建子节点
	cJSON * pSSubJson = NULL;	   //创建子节点
	cJSON * pSSSubJson = NULL;		//创建数组
	cJSON * pSSSSubJson = NULL; //在数组上添加对象


	pJsonRoot = cJSON_CreateObject();
	cJSON_AddStringToObject(pJsonRoot, "version", "1.2");
	cJSON_AddNumberToObject(pJsonRoot, "type", netbuf.type);
	cJSON_AddNumberToObject(pJsonRoot, "tagid", (READ_REG(*((uint32_t *)0x1FF1E800)))/1000+520);
	
    pSubJson = cJSON_CreateObject();
    cJSON_AddNumberToObject(pSubJson,"x",netbuf.uwb->uwbParse.tag.x);
    cJSON_AddNumberToObject(pSubJson,"y",netbuf.uwb->uwbParse.tag.y);
	cJSON_AddNumberToObject(pSubJson,"z",netbuf.uwb->uwbParse.tag.z);
    cJSON_AddItemToObject(pJsonRoot,"tag",pSubJson);  // 子节点的“键”，子节点挂在父节点上

    pSSubJson = cJSON_CreateObject();
    cJSON_AddNumberToObject(pSSubJson, "x", ORIGN_eh_ECEF.x);//-2776227.23743292
    cJSON_AddNumberToObject(pSSubJson, "y", ORIGN_eh_ECEF.y);//4760647.343173147
    cJSON_AddNumberToObject(pSSubJson, "z", ORIGN_eh_ECEF.z);//3200098.2119018384
    cJSON_AddItemToObject(pJsonRoot, "origin", pSSubJson);  // 子节点的“键”，子节点挂在父节点上

    pSSSubJson = cJSON_CreateArray();
    cJSON_AddItemToObject(pJsonRoot, "apInfo", pSSSubJson);  // 数组在父节点上  
    
	for (int i = 0; i < netbuf.uwb->uwbParse.uwbdata.data.num; ++i)
	{
		pSSSSubJson = cJSON_CreateObject();
		
		cJSON_AddNumberToObject(pSSSSubJson, "id", netbuf.uwb->uwbParse.uwbdata.data.info[i].addr);
		cJSON_AddNumberToObject(pSSSSubJson, "dist", netbuf.uwb->uwbParse.uwbdata.data.info[i].dist/100.0);
		cJSON_AddNumberToObject(pSSSSubJson, "confidence", netbuf.uwb->uwbParse.uwbdata.data.info[i].deg);
		cJSON_AddNumberToObject(pSSSSubJson, "rssi", netbuf.uwb->uwbParse.uwbdata.data.info[i].RSSI);
		cJSON_AddItemToArray(pSSSubJson, pSSSSubJson);
	}
	
    /* Print to text */
    if(TCP_preallocated(pJsonRoot) != 0)
    {
        cJSON_Delete(pJsonRoot);
		
        return FALSE;
    }
    cJSON_Delete(pJsonRoot);
	
	return TRUE;

}



/*****************************************************************************
 函 数 名  : num_strchr
 功能描述  : 查询特定字符串函数
 输入参数  : const char *str, char c
			* const char *str ，字符串
			* char c，要查找的字符
 输出参数  : 无
 返 回 值  : int pindex - str
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年10月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
int num_strchr(const char *str, char c) // 
{
    const char *pindex = strchr(str, c);
    if (NULL == pindex){
        return -1;
    }
    return pindex - str;
}


/*****************************************************************************
 函 数 名  : base64_decode
 功能描述  : base64解码函数
 输入参数  : const char * base64, unsigned char * dedata
			* const char * base64 码字
			* unsigned char * dedata， 解码恢复的数据
 输出参数  : 无
 返 回 值  : int j
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2020年10月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
int base64_decode(const char * base64, unsigned char * dedata)
{
    int i = 0, j=0;
    int trans[4] = {0,0,0,0};
    for (;base64[i]!='\0';i+=4)
    {
        // 每四个一组，译码成三个字符
        trans[0] = num_strchr(base64char, base64[i]);
        trans[1] = num_strchr(base64char, base64[i+1]);
        // 1/3
        dedata[j++] = ((trans[0] << 2) & 0xfc) | ((trans[1]>>4) & 0x03);

        if (base64[i+2] == '='){
            continue;
        }
        else{
            trans[2] = num_strchr(base64char, base64[i + 2]);
        }
        // 2/3
        dedata[j++] = ((trans[1] << 4) & 0xf0) | ((trans[2] >> 2) & 0x0f);

        if (base64[i + 3] == '='){
            continue;
        }
        else{
            trans[3] = num_strchr(base64char, base64[i + 3]);
        }

        // 3/3
        dedata[j++] = ((trans[2] << 6) & 0xc0) | (trans[3] & 0x3f);
    }

    dedata[j] = '\0';

    return j;
}

/*****************************************************************************
 函 数 名  : base64_encode
 功能描述  : base64加密编码函数
 输入参数  : const unsigned char * bindata, char * base64, int binlength
 输出参数  : 无
 返 回 值  : int j
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月
    作    者   : zxf
    修改内容   : 创建

*****************************************************************************/
int base64_encode( const unsigned char * bindata, char * base64, int binlength )
{
    int i, j;
    unsigned char current;

    for ( i = 0, j = 0 ; i < binlength ; i += 3 )
    {
        current = (bindata[i] >> 2) ;
        current &= (unsigned char)0x3F;
        base64[j++] = base64char[(int)current];

        current = ( (unsigned char)(bindata[i] << 4 ) ) & ( (unsigned char)0x30 ) ;
        if ( i + 1 >= binlength )
        {
            base64[j++] = base64char[(int)current];
            base64[j++] = '=';
            base64[j++] = '=';
            break;
        }
        current |= ( (unsigned char)(bindata[i+1] >> 4) ) & ( (unsigned char) 0x0F );
        base64[j++] = base64char[(int)current];

        current = ( (unsigned char)(bindata[i+1] << 2) ) & ( (unsigned char)0x3C ) ;
        if ( i + 2 >= binlength )
        {
            base64[j++] = base64char[(int)current];
            base64[j++] = '=';
            break;
        }
        current |= ( (unsigned char)(bindata[i+2] >> 6) ) & ( (unsigned char) 0x03 );
        base64[j++] = base64char[(int)current];

        current = ( (unsigned char)bindata[i+2] ) & ( (unsigned char)0x3F ) ;
        base64[j++] = base64char[(int)current];
    }
    base64[j] = '\0';
	
    return j;
}

void UWB_Vel_Angle_calc(void)
{
	double Abs_value = 0;
	double angle_ptr = 0;

	LLA_pos.latitude = server_Data.latitude;
	LLA_pos.longitude = server_Data.longitude;
	LLA_pos.height = server_Data.height;
	//经纬度坐标转ECEF坐标
	WGSToECEF(&LLA_pos, &ECEF_pos);
	//ECEF坐标转东北天坐标
	ECEFToENU(&ECEF_pos, &ORIGN_pos, &ENU_pos);//得到延迟前的东北天坐标
	
	UWB_Veloc = (sqrt(ENU_pos.northing*ENU_pos.northing + ENU_pos.easting*ENU_pos.easting)/(g_pos_info.position_interval/1000000))/KNToMS;//

	Abs_value = fabs(ENU_pos.easting/ENU_pos.northing);
	angle_ptr = atan(Abs_value)*180/PI;

	if(ENU_pos.northing>0&&ENU_pos.easting>=0)
	{
		UWB_angle = angle_ptr;
	}
	else if(ENU_pos.northing>0&&ENU_pos.easting<=0)
	{
		UWB_angle = 360 - angle_ptr;
	}
	else if(ENU_pos.northing<0&&ENU_pos.easting<=0)
	{
		UWB_angle = 180 + angle_ptr;
	}
	else if(ENU_pos.northing<0&&ENU_pos.easting>=0)
	{
		UWB_angle = 180 - angle_ptr;
	}
	//GLOBAL_PRINT(("UWB_Veloc = %f\r\nUWB_angle = %f\r\n",UWB_Veloc,UWB_angle));

	ORIGN_pos.latitude = server_Data.latitude;
	ORIGN_pos.longitude = server_Data.longitude;
	ORIGN_pos.height = server_Data.height;

}


/*****************************************************************************
 函 数 名  : Server_output_init
 功能描述  : Server output初始化函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年4月
    作    者   : zxf
    修改内容   : 创建

*****************************************************************************/
static void Server_output_init(void)
{
	server_Data.mode=0;
	
	NMEA_Data_ptr.lat = 0;
	NMEA_Data_ptr.lon = 0;
	NMEA_Data_ptr.alt = 0;
	Pos_time.RTK_TIME = 0;
	
	g_pos_info.tag_position[0] = 0;
	g_pos_info.tag_position[1] = 0;
	g_pos_info.tag_position[2] = 0;
	Pos_time.UWB_TIME = 0;
}


/*****************************************************************************
 函 数 名  : _Server_thread
 功能描述  : 数据上报线程
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月
    作    者   : zxf
    修改内容   : 创建

*****************************************************************************/
static void _Server_thread(void * arg)
{	
	static U16 loop_cnt = 0;
	static U8 flag=0, First_flag=0;
	static U32 CNT_num=0;

	First_flag = TRUE;
	
	while(1)
	{
		loop_cnt++;
		if(loop_cnt>32700)
			loop_cnt = 0;
		
		if(SERVER_RTK==POS_VALID && NMEA_Data_ptr.GGA_DATA_READY==TRUE)//RTK定位有效
		{
			//取RTK定位数值与时间
			Pos_time.RTK_TIME = osKernelGetTickCount();
			DBG_RTK_Print("\r\nHPR_OUTPUT[RTK],%d,%lf,%lf,%lf,[MODE],%d\r\n\r\n",\
							Pos_time.RTK_TIME,\
							NMEA_Data_ptr.lat,NMEA_Data_ptr.lon,NMEA_Data_ptr.alt,\
							NMEA_Data_ptr.RTK_mode);
			SERVER_RTK = POS_VALUE_VALID;
		}
		if(g_pos_info.tag_position_valid_flag == POS_VALID)//UWB定位有效
		{
			CNT_num++;
			//取UWB定位数值与时间		
			DBG_ENHANCE_Print("\r\nHPR_OUTPUT[UWB],%d,%d,%lf,%lf,%lf,%d,%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf,[VALID],%d\r\n\r\n",\
							CNT_num,\
							g_pos_info.pos_Time,\
							g_pos_info.tag_position[0],g_pos_info.tag_position[1],g_pos_info.tag_position[2],\
							g_pos_info.main_anchor_id,\
							g_pos_info.D0,\
							g_pos_info.sub_anchor_id,\
							g_pos_info.D1,\
							g_pos_info.t2ref_dist,\
							g_pos_info.t2wall_dist,\
							g_pos_info.t2main_dist,\
							g_pos_info.main_rssi,\
							g_pos_info.sub_rssi,\
							g_pos_info.tag_position_valid_flag);
			
			g_pos_info.tag_position_valid_flag = POS_VALUE_VALID;
		}

		if(SERVER_RTK==POS_VALUE_VALID||g_pos_info.tag_position_valid_flag==POS_VALUE_VALID)
		{
			//判断定位有效状态，按RTK>UWB>GNSS优先级进行上报
			if(SERVER_RTK==POS_VALUE_VALID && NMEA_Data_ptr.RTK_mode == 4)//RTK数据有效
			{		
				server_Data.latitude  = NMEA_Data_ptr.lat;
				server_Data.longitude = NMEA_Data_ptr.lon;
				server_Data.height    = NMEA_Data_ptr.alt;
				server_Data.mode = NMEA_Data_ptr.RTK_mode;
				NMEA_Data_ptr.GGA_DATA_READY = FALSE;
				SERVER_RTK = POS_VALUE_INVALID;
			}
			else if(g_pos_info.tag_position_valid_flag==POS_VALUE_VALID && NMEA_Data_ptr.RTK_mode != 4)//RTK无效且UWB数据有效
			{
				server_Data.latitude  = g_pos_info.tag_position[0];
				server_Data.longitude = g_pos_info.tag_position[1];
				server_Data.height    = g_pos_info.tag_position[2];
				server_Data.mode = 7;
				g_pos_info.tag_position_valid_flag = POS_VALUE_INVALID;
				
				if(First_flag)
				{
					ORIGN_pos.latitude = server_Data.latitude;
					ORIGN_pos.longitude = server_Data.longitude;
					ORIGN_pos.height = server_Data.height;
					First_flag = FALSE;
				}
				else
				{
					UWB_Vel_Angle_calc();
				}
				
			}
			else if(g_pos_info.tag_position_valid_flag==POS_INVALID && NMEA_Data_ptr.RTK_mode != 4)//RTK与UWB均无效且GNSS有效
			{	
				server_Data.latitude  = NMEA_Data_ptr.lat;
				server_Data.longitude = NMEA_Data_ptr.lon;
				server_Data.height    = NMEA_Data_ptr.alt;
				server_Data.mode = NMEA_Data_ptr.RTK_mode;
				NMEA_Data_ptr.GGA_DATA_READY = TRUE;
				SERVER_RTK = POS_VALUE_INVALID;
			}
		}
		if(loop_cnt%100 == 0)
		{
			if(MQTT_OFF_LINE == FALSE)
			{
				if(!upload_hpr_state_to_server())
					WARN_PRINT(("Heart Messege Sent Error!!\r\n"));
				if(!upload_hpr_cmd_log_to_server())
					WARN_PRINT(("CMD_log Messege Sent Error!!\r\n"));
			}
			if(!UDP_upload_hpr_location())
				WARN_PRINT(("UDP Messege Sent Error!!\r\n"));

			if(!flag){
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
				flag = TRUE;
			}else{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
				flag = FALSE;
			}
		}
		if(g_pos_info.tag_position_valid_flag == POS_INVALID)
		{
			FIRST_UWB_GGA = TRUE;
			FIRST_UWB_RMC = TRUE;
			First_flag = TRUE;
		}
		delay_ms(10);
	}
}


/*****************************************************************************
 函 数 名  : Server_apl_init
 功能描述  : 数据上报线程初始化函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年3月
    作    者   : zxf
    修改内容   : 创建

*****************************************************************************/
void Server_apl_init(void)
{
	osThreadId_t thread_Server_id = 0;

	Server_output_init();
	
	thread_Server_id = osThreadNew(_Server_thread, NULL, &thread_Server_attr);
	GLOBAL_HEX(thread_Server_id);
}

