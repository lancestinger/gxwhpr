#include "server_apl.h"
#include "json/cJSON.h"
#include "iotclient/iotclient.h"
#include "meas/meas_distance.h"
#include "fml/gnss/nmea.h"
#include "drv/log.h"
#include "drv/socket/socket_drv.h"
#include "update/update_apl.h"
#include "fml/sshell/sshell_fml.h"
#include "fml/uart/uart_fml.h"
#include "drv/rngbuf/rngbuf.h"


Server_Data server_Data;
Pos_count Pos_time;

static u8 g_mqtt_print_tmpbuf[1024];
static u8 g_mqtt_print_buf[1300];
//static u8 g_mqtt_print_tmpbuf[2048];
//static u8 g_mqtt_print_buf[2200];


extern UPDATE_HANDLE update_handle_g;
int SERVER_RTK=0; 				  /* RTK����Ready��־ */
int SERVER_UWB=0; 				  /* UWB����Ready��־ */
int SERVER_GNSS=0;                /* UWB����Ready��־ */
int SERVER_ERROR=0;			      /* ��λ���ݴ����־ */

U8 UPLOAD_VAL=0;                  /*�ϱ���־*/

// ȫ�ֳ�������
const char * base64char = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
const char padding_char = '=';

U8 UDP_DST[5]={
						192,
						168,
						10,
						158,
				};
int UDP_PORT = 8088;

static U64 thread_Server_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_Server_attr = {
  .stack_mem  = &thread_Server_stk[0],
  .stack_size = sizeof(thread_Server_stk),
  .priority = osPriorityNormal,
};

/*****************************************************************************
 �� �� ��  : print_preallocated
 ��������  : MQTT�ϱ����ݴ��亯��
 �������  : cJSON *root  
 �������  : ��
 �� �� ֵ  : int
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��10��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
/* Create a bunch of objects as demonstration. */
static int print_preallocated(IOT_Device_t* iotDev, cJSON *root)
{
    /* declarations */
    char *out = NULL;
    char *buf = NULL;
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
    iotclient_publish(iotDev, buf, len-5);
    
    free(out);
    free(buf);
    return 0;
}


/*****************************************************************************
 �� �� ��  : upload_hpr_state_to_server
 ��������  : �ϱ����ջ�״̬����(������)
 �������  : int mode
 			* RTK_MODE
 			* UWB_MODE
 			* GNSS_MODE
 			* DATA_ERROR
 �������  : ��
 �� �� ֵ  : U8
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��12��
    ��    ��   : sunj
    �޸�����   : ����
  1.��	  ��   : 2021��3��
    ��	  ��   : zxf
    �޸�����   : ����ģʽѡ������Э��

*****************************************************************************/
//��������ϴ���ǩ״̬
U8 upload_hpr_state_to_server(void)
{
    cJSON *root = NULL;
    cJSON *data_obj = NULL;
	static u32 cnt_num = 0;

	if(g_device_config.device_type != TAG)
		return TRUE;
	
	cnt_num++;
		
    root = cJSON_CreateObject();
    data_obj = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "cmd", 601);
    cJSON_AddNumberToObject(root, "response", 0);
    cJSON_AddStringToObject(root, "task_uuid","1234567890123456");
    cJSON_AddStringToObject(root, "cmd_uuid","1234567890123456");

	
	cJSON_AddNumberToObject(data_obj, "latitude", server_Data.latitude);
	cJSON_AddNumberToObject(data_obj, "longitude", server_Data.longitude);
	cJSON_AddNumberToObject(data_obj, "altitude", server_Data.height);
	cJSON_AddNumberToObject(data_obj, "mode", server_Data.mode);	
	
	if(cnt_num%10 == 0)
	{
		cJSON_AddStringToObject(data_obj, "version",PROJECT_NAME);
		cJSON_AddNumberToObject(data_obj, "tagH", g_device_config.tag_h);
		cJSON_AddNumberToObject(data_obj, "txPower", calc_tx_power_config_value_to_db(g_device_config.tx_power));
		cJSON_AddNumberToObject(data_obj, "id", TAG_ID_SUM);
		cJSON_AddNumberToObject(data_obj, "iTranAntDelay", g_device_config.ant_tx_delay);  
	  	cJSON_AddNumberToObject(data_obj, "iRecvAntDelay", g_device_config.ant_rx_delay);
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
 �� �� ��  : upload_hpr_cmd_feedback_to_server
 ��������  : ����������ظ���ִ����
 �������  : Server_Ori_Data data, int state 
 �������  : ��
 �� �� ֵ  : U8
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��12��
    ��    ��   : sunj
    �޸�����   : ����

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
 �� �� ��  : upload_hpr_update_feedback_to_server
 ��������  : ������������������
 �������  : int state 
 
 * state: 
 * 0�������ɹ�
 * 1��������ʱ
 * 2����¼����,�û���/������Ч
 * 3���ļ����������
 * 4���ļ�δ����
 * 5������·��δ����
 * 6�������ļ���/д����
 * 7��FTP�ͻ��˴���
 * 8��У��ʧ��
 
 �������  : ��
 �� �� ֵ  : U8
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��12��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
U8 upload_hpr_update_feedback_to_server(int state)
{
    cJSON *root = NULL;
    cJSON *data_obj = NULL;

    root = cJSON_CreateObject();
    data_obj = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "cmd", 102);
    cJSON_AddNumberToObject(root, "response", 0);
    cJSON_AddStringToObject(root, "task_uuid", "1123456789123456");
    cJSON_AddStringToObject(root, "cmd_uuid", "0123456789123456");
    
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
 �� �� ��  : upload_efs_all_para_to_server
 ��������  : �������������������в�����Ϣ����
 �������  : ��
 �������  : ��
 �� �� ֵ  : U8
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��
    ��    ��   : wangxuan
    �޸�����   : ����

*****************************************************************************/
U8 upload_efs_all_para_to_server(void)
{
    cJSON *root = NULL;
    cJSON *data_obj = NULL;
	char temp_str[50] = {0};
	
    root = cJSON_CreateObject();
    data_obj = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "cmd", 601);	
    cJSON_AddNumberToObject(root, "response", 0);
    cJSON_AddStringToObject(root, "task_uuid","1234567890123456");
    cJSON_AddStringToObject(root, "cmd_uuid","1234567890123456");

	cJSON_AddStringToObject(data_obj, "SOFTWARE VERSION", HW_VERSION);
	sprintf(temp_str, "%d.%d", SW_VERSION_H, SW_VERSION_L);
	cJSON_AddStringToObject(data_obj, "HARDWARE VERSION", temp_str);


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
	cJSON_AddNumberToObject(data_obj, "iTranAntDelay", g_device_config.ant_tx_delay);  
    cJSON_AddNumberToObject(data_obj, "iRecvAntDelay", g_device_config.ant_rx_delay);		 
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
 �� �� ��  : parse_server_data
 ��������  : �����������·����ݺ���
 �������  : U8* buf, int len 
 �������  : ��
 �� �� ֵ  : U8
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��12��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
U8 parse_server_data(U8* buf, int len)
{
    cJSON *receive_obj=NULL;
    cJSON *item_obj=NULL;
    cJSON *cmd_obj=NULL;
    cJSON *data_obj=NULL;
    U8 ip_buf[NET_ADDR_IP4_LEN];

    int cmd = 0;

	Server_Ori_Data serOriData;
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
        DBG_MQTT_Print("δ��ȡ��cmd����!!!\r\n");
        cJSON_Delete(receive_obj);
        return FALSE;
    }

	serOriData.response = cJSON_GetObjectItem(receive_obj,"response")->valueint;
    strcpy(serOriData.task_uuid, cJSON_GetObjectItem(receive_obj,"task_uuid")->valuestring);
    strcpy(serOriData.cmd_uuid, cJSON_GetObjectItem(receive_obj,"cmd_uuid")->valuestring);
    
    data_obj = cJSON_GetObjectItem(receive_obj,"data");

    switch(cmd)
    {
        case 100://��ʼ����
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
                ERR_PRINT(("ftpServer��ʽ�Ƿ�!!!!\r\n"));
            }

            update_handle_g.updateSta = updateSta_EventDownloadStart;
        break;

        case 103://ֹͣ����
        break;

        case 602://����
            //�������������
            upload_hpr_cmd_feedback_to_server(serOriData,1);
			MQTT_OFF_LINE = TRUE;
            NOTE_PRINT(("\r\nPlease waiting for system rebooting!!!!!!\r\n\r\n"));
            //delay_ms(1000);
            NVIC_SystemReset();
        break;

        case 603://��������
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
					g_device_config.ant_tx_delay = item_obj->valueint;
					save_device_para();
                }
				else if(!strcmp(item_obj->string,"iRecvAntDelay"))
                {
					g_device_config.ant_rx_delay = item_obj->valueint;
					save_device_para();
                }
									
                item_obj = item_obj->next;
            }
			
			GLOBAL_INTVAL(g_device_config.ant_tx_delay);
			GLOBAL_INTVAL(g_device_config.ant_rx_delay);
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
			upload_efs_all_para_to_server();
			break;

        default:
            WARN_PRINT(("δ֪cmd[%d]!!!\r\n",cmd));
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
		DBG_SERVER_Print("cmd: %s\r\n", cmd_obj->valuestring);
		sshell_excute_cmd(cmd_obj->valuestring);
		cJSON_Delete(receive_obj);
		return TRUE;
	}
	else
	{
		WARN_PRINT(("δ��ȡ��cmd����!!!\r\n"));
		cJSON_Delete(receive_obj);
		return FALSE;
	}
	
	

}


/*****************************************************************************
 �� �� ��  : UDP_preallocated
 ��������  : UDP�ϱ����ݴ��亯��
 �������  : cJSON *root  
 �������  : ��
 �� �� ֵ  : int
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��
    ��    ��   : zxf
    �޸�����   : ����

*****************************************************************************/
static int UDP_preallocated(cJSON *root)
{
    /* declarations */
    char *out = NULL;
    char *buf = NULL;
    size_t len = 0;
	size_t len_out = 0;

	SOCKADDR_IN* UDP_dst = (SOCKADDR_IN*)GLOBAL_MALLOC(sizeof(SOCKADDR_IN));
	GLOBAL_MEMSET(UDP_dst,0x0,sizeof(SOCKADDR_IN));

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
		free(UDP_dst);
        free(out);
        free(buf);
        return -1;
    }
    //GLOBAL_PRINT(("json str:%s\r\n",buf));
	DBG_Socket_Print("UDP SENT = %s\r\n",out);
	
	UDP_dst->sin_family = PF_INET;
	UDP_dst->sin_addr.s_b1 = UDP_DST[0];
	UDP_dst->sin_addr.s_b2 = UDP_DST[1];
	UDP_dst->sin_addr.s_b3 = UDP_DST[2];
	UDP_dst->sin_addr.s_b4 = UDP_DST[3];
	UDP_dst->sin_port = htons(UDP_PORT);
	
    socket_UDP_send_msg(SOCKET_JSON,UDP_dst,(U8*)out,len_out);

	free(UDP_dst);
    free(out);
    free(buf);
    return 0;
}


/*****************************************************************************
 �� �� ��  : UDP_upload_hpr_location
 ��������  : UDP�ϱ���λģʽѡ����
 �������  : int mode   
 			* GNSS_MODE/RTK_MODE/UWB_MODE/DATA_ERROR
 �������  : ��
 �� �� ֵ  : U8
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��
    ��    ��   : zxf
    �޸�����   : ����

*****************************************************************************/
U8 UDP_upload_hpr_location(void)
{
	cJSON *root = NULL;

	if(g_device_config.device_type != TAG)
		return TRUE;

    root = cJSON_CreateObject();
	
	cJSON_AddNumberToObject(root, "lat", server_Data.latitude);
	cJSON_AddNumberToObject(root, "lon", server_Data.longitude);
	                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    /* Print to text */
    if(UDP_preallocated(root) != 0)
    {
        cJSON_Delete(root);
        return FALSE;
    }
    cJSON_Delete(root);

    return TRUE;

}


/*****************************************************************************
 �� �� ��  : num_strchr
 ��������  : ��ѯ�ض��ַ�������
 �������  : const char *str, char c
			* const char *str ���ַ���
			* char c��Ҫ���ҵ��ַ�
 �������  : ��
 �� �� ֵ  : int pindex - str
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��10��
    ��    ��   : sunj
    �޸�����   : ����

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
 �� �� ��  : base64_decode
 ��������  : base64���뺯��
 �������  : const char * base64, unsigned char * dedata
			* const char * base64 ����
			* unsigned char * dedata�� ����ָ�������
 �������  : ��
 �� �� ֵ  : int j
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2020��10��
    ��    ��   : sunj
    �޸�����   : ����

*****************************************************************************/
int base64_decode(const char * base64, unsigned char * dedata)
{
    int i = 0, j=0;
    int trans[4] = {0,0,0,0};
    for (;base64[i]!='\0';i+=4)
    {
        // ÿ�ĸ�һ�飬����������ַ�
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
 �� �� ��  : base64_encode
 ��������  : base64���ܱ��뺯��
 �������  : const unsigned char * bindata, char * base64, int binlength
 �������  : ��
 �� �� ֵ  : int j
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��
    ��    ��   : zxf
    �޸�����   : ����

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

/*****************************************************************************
 �� �� ��  : Server_output_init
 ��������  : Server output��ʼ������
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��4��
    ��    ��   : zxf
    �޸�����   : ����

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
 �� �� ��  : _Server_thread
 ��������  : �����ϱ��߳�
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��
    ��    ��   : zxf
    �޸�����   : ����

*****************************************************************************/
static void _Server_thread(void * arg)
{	
	static U16 loop_cnt = 0;
	static U8 flag=0, PRINT_ret=0;

	while(1)
	{
		loop_cnt++;
		if(loop_cnt>32700)
			loop_cnt = 0;
		
		if(SERVER_RTK==POS_VALID && GGA_DATA_READY>=1)//RTK��λ��Ч
		{
			//ȡRTK��λ��ֵ��ʱ��
			Pos_time.RTK_TIME = osKernelGetTickCount();
			DBG_SERVER_Print("\r\nHPR_OUTPUT[RTK],%d,%lf,%lf,%lf,[MODE],%d\r\n\r\n",\
							Pos_time.RTK_TIME,\
							NMEA_Data_ptr.lat,NMEA_Data_ptr.lon,NMEA_Data_ptr.alt,\
							NMEA_Data_ptr.RTK_mode);
			SERVER_RTK = POS_VALUE_VALID;
		}
		if(g_pos_info.tag_position_valid_flag == POS_VALID)//UWB��λ��Ч
		{
			//ȡUWB��λ��ֵ��ʱ��
			Pos_time.UWB_TIME = osKernelGetTickCount();		
			DBG_SERVER_Print("\r\nHPR_OUTPUT[UWB],%d,%lf,%lf,%lf,%d,%lf,%d,%lf,%lf,%lf,%lf,[VALID],%d\r\n\r\n",\
							Pos_time.UWB_TIME,\
							g_pos_info.tag_position[0],g_pos_info.tag_position[1],g_pos_info.tag_position[2],\
							g_pos_info.main_anchor_id,\
							g_pos_info.D0,\
							g_pos_info.sub_anchor_id,\
							g_pos_info.D1,\
							g_pos_info.t2ref_dist,\
							g_pos_info.t2wall_dist,\
							g_pos_info.rssi,\
							g_pos_info.tag_position_valid_flag);
			
			g_pos_info.tag_position_valid_flag = POS_VALUE_VALID;
		}

		if(SERVER_RTK==POS_VALUE_VALID||g_pos_info.tag_position_valid_flag==POS_VALUE_VALID)
		{
			//�ж϶�λ��Ч״̬����RTK>UWB>GNSS���ȼ������ϱ�
			if(SERVER_RTK==POS_VALUE_VALID && NMEA_Data_ptr.RTK_mode == 4)//RTK������Ч
			{		
				server_Data.latitude  = NMEA_Data_ptr.lat;
				server_Data.longitude = NMEA_Data_ptr.lon;
				server_Data.height    = NMEA_Data_ptr.alt;
				server_Data.mode = NMEA_Data_ptr.RTK_mode;
				GGA_DATA_READY = 0;
				SERVER_RTK = POS_VALUE_INVALID;
			}
			else if(g_pos_info.tag_position_valid_flag==POS_VALUE_VALID && NMEA_Data_ptr.RTK_mode != 4)//RTK��Ч��UWB������Ч
			{
				server_Data.latitude  = g_pos_info.tag_position[0];
				server_Data.longitude = g_pos_info.tag_position[1];
				server_Data.height    = g_pos_info.tag_position[2];
				server_Data.mode = 7;
				g_pos_info.tag_position_valid_flag = POS_VALUE_INVALID;
			}
			else if(g_pos_info.tag_position_valid_flag==POS_INVALID && NMEA_Data_ptr.RTK_mode != 4)//RTK��UWB����Ч��GNSS��Ч
			{	
				server_Data.latitude  = NMEA_Data_ptr.lat;
				server_Data.longitude = NMEA_Data_ptr.lon;
				server_Data.height    = NMEA_Data_ptr.alt;
				server_Data.mode = NMEA_Data_ptr.RTK_mode;
				GGA_DATA_READY = 0;
				SERVER_RTK = POS_VALUE_INVALID;
			}
		}
		if(loop_cnt%100 == 0)
		{
		/*
			if(server_Data.mode == 0)
			{
				server_Data.latitude  = 0;
				server_Data.longitude = 0;
				server_Data.height    = 0;
			}
		*/
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
		delay_ms(10);
	}
}


/*****************************************************************************
 �� �� ��  : Server_apl_init
 ��������  : �����ϱ��̳߳�ʼ������
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2021��3��
    ��    ��   : zxf
    �޸�����   : ����

*****************************************************************************/
void Server_apl_init(void)
{
	osThreadId_t thread_Server_id = 0;

	Server_output_init();
	
	thread_Server_id = osThreadNew(_Server_thread, NULL, &thread_Server_attr);
	GLOBAL_HEX(thread_Server_id);
}

