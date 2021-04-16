
#include "project_def.h"
#include "crc/crc.h"





main_handle_t main_handle_g;	//����������
extra_para_t ex_para_g;
license_handle_t license_handle_g;  //license��������
hn_handle_t hn_handle_g;    //hn���ò�������

osEventFlagsId_t os_event_id_g = NULL;		//ȫ���¼�ID
static netStatus set_status = netOK;


device_enum g_gnss_work_model = DEV_GPS_GLO;          //����ģʽ  0��ʾGPS&GLONASS  1��ʾGPS&BDS&GLONASS

DBG_OUT_USART_WORK_MODEL g_dbg_out_usart_model = DBG_OUT_USART_NORMAL_MODEL;     //���Դ��ڹ���ģʽ

static const cfg_para_t default_cfg_para = 
{
	1,										//UBX
	18,										//������
	// 8,										//ʱ��
	// 0,										//�ϵ�����
	// 0,								//�ο�Դѡ��
	// 0,							//�ο�Դ�л�ģʽ
	// 0,						//Ĭ��ѹ�ص�ѹ
	// {30.736571, 103.96061, 519.2},		    //λ��
	// 5,										//������
	// 60,										//����ٶ�
	// {80, 80, 80, 80},						//˥��ֵ
	// {0, 0, 0, 0},							//ʱ��
	// {0, 1, 0},								//Բ���˶�����
	// {0, 0},									//�ٶ�
	// {0, 0},									//���ٶ�
	{
		0,									//DHCP
		192,168,10,235,					    //IP
		255,255,255,0,						//MASK
		192,168,10,1,						//GATEWAY
		1,2,3,0,5,6,
        182,61,51,239,
        1883
	}, 
    // 1,                                      //�����Զ���ʱʹ��
    // 0,                                      //ʹ�õ�ǰ��λλ����ΪԲ�Ľ���Բ���˶�ʹ��
    // 192,168,10,100,                         //����IP1
    // 192,168,10,219,                         //����IP2
    // 5,                                      //����״̬Ѳ����Ϣ����
    // {0, 0, 0, 0}                            //ͨ������˥����С����
};

static cfg_para_t cfg_para_temp;
static license_handle_t license_temp;

void license_reset(void)
{
    license_handle_g.license = 0;
}

void license_save(void)
{
    FILE* fd = 0;

	GLOBAL_MEMCPY(&license_temp, &license_handle_g, sizeof(license_temp));

	license_temp.crc = crc32((U8*)&license_temp, (U8 *)&(license_temp.crc) - (U8 *)&license_temp);

	if((fd = fopen(LICENSE_FILE, "w")) != NULL)
	{
		fwrite(&license_temp, sizeof(license_temp), 1, fd);
		fclose(fd);
		NOTE_PRINT(("���֤��Ȩ�뱣��ɹ�!\r\n"));
	}
	else
	{
		ERR_PRINT(("���֤��Ȩ�뱣��ʧ��!\r\n"));
	} 
}

//HN������Ϣ��λ
void hn_cfg_reset(void)
{
    hn_handle_g.type = 1;
}

//HN������Ϣ�������ļ�
void hn_cfg_save(void)
{
    FILE* fd = 0;
    hn_handle_t hn_cfg_temp;

    GLOBAL_MEMCPY(&hn_cfg_temp, &hn_handle_g, sizeof(hn_handle_t));

    hn_cfg_temp.crc = crc32((U8*)&hn_cfg_temp, (U8*)&(hn_cfg_temp.crc) - (U8*)&hn_cfg_temp);

    if((fd = fopen(HN_CFG_FILE, "w")) != NULL)
	{
		fwrite(&hn_cfg_temp, sizeof(hn_cfg_temp), 1, fd);
		fclose(fd);
		NOTE_PRINT(("HN CFG �ļ�����ɹ�!\r\n"));
	}
	else
	{
		ERR_PRINT(("HN CFG �ļ�����ʧ��!\r\n"));
	} 
}



/*****************************************************************************
 �� �� ��  : sys_para_reset
 ��������  : ������λ
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��9��16��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_para_reset(void)
{
	GLOBAL_MEMCPY(&main_handle_g.cfg, &default_cfg_para, sizeof(cfg_para_t));
}

/*****************************************************************************
 �� �� ��  : sys_reset_para_save
 ��������  : ��λ�������棬���ʹ�ã��������һ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��10��17��
    ��    ��   : sunj
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_reset_para_save(void)
{
   	FILE* fd = 0;

	GLOBAL_MEMCPY(&cfg_para_temp, &main_handle_g.cfg, sizeof(cfg_para_temp));

	cfg_para_temp.crc = crc32((U8*)&cfg_para_temp, (U8 *)&(cfg_para_temp.crc) - (U8 *)&cfg_para_temp);
	if((fd = fopen(SYS_PARA_FILE, "w")) != NULL)
	{
		fwrite(&cfg_para_temp, sizeof(cfg_para_temp), 1, fd);
		fclose(fd);
		NOTE_PRINT(("��λϵͳ��������ɹ�!\r\n"));
	}
	else
	{
		ERR_PRINT(("��λϵͳ��������ʧ��!\r\n"));
	} 
}

/*****************************************************************************
 �� �� ��  : sys_para_save
 ��������  : ��������
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��9��15��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_para_save(void)
{
	FILE* fd = 0;

	GLOBAL_MEMCPY(&cfg_para_temp, &main_handle_g.cfg, sizeof(cfg_para_temp));
	cfg_para_temp.crc = crc32((U8*)&cfg_para_temp, (U8 *)&(cfg_para_temp.crc) - (U8 *)&cfg_para_temp);
	if((fd = fopen(SYS_PARA_FILE, "w")) != NULL)
	{
		fwrite(&cfg_para_temp, sizeof(cfg_para_temp), 1, fd);
		fclose(fd);
		NOTE_PRINT(("ϵͳ��������ɹ�!\r\n"));
	}
	else
	{
		ERR_PRINT(("ϵͳ��������ʧ��!\r\n"));
	}

	if((fd = fopen(NET_PARA_FILE, "w")) != NULL)
	{
		fwrite(&cfg_para_temp.net, sizeof(cfg_para_temp.net), 1, fd);
		fclose(fd);
		NOTE_PRINT(("net��������ɹ�!\r\n"));
	}
	else
	{
		ERR_PRINT(("net��������ʧ��!\r\n"));
	}
}

/*****************************************************************************
 �� �� ��  : sys_para_load
 ��������  : ϵͳ��������
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��9��16��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_para_load(void)
{
	FILE* fd = 0;
	U32 calc_crc = 0;

	if((fd = fopen(SYS_PARA_FILE, "r")) != NULL)
	{
		fread(&main_handle_g.cfg, sizeof(main_handle_g.cfg), 1, fd);
		fclose(fd);
	}
	calc_crc = crc32((U8 *)&main_handle_g.cfg,(U8 *)&(main_handle_g.cfg.crc) - (U8 *)&main_handle_g.cfg);
	
	if((calc_crc != main_handle_g.cfg.crc) || (fd == NULL))
	{
		GLOBAL_HEX(calc_crc);
		GLOBAL_HEX(main_handle_g.cfg.crc);
		ERR_PRINT(("���ݿ�У����󣬻ָ�ȱʡ����!\r\n"));
		sys_para_reset();
		sys_reset_para_save();
	}
	else
	{
		sys_para_reset();
		sys_reset_para_save();
		NOTE_PRINT(("ϵͳ�������سɹ�!!!!\r\n"));
	}

//    g_gnss_work_model = (device_enum)main_handle_g.cfg.work_model;   //added by sunj 2019-10-16 14:31

    #ifdef VERSION_HUANUO
        #ifdef BDS_INCLUDE
            g_gnss_work_model = DEV_GPS_BDS_GLO;  //��BDS GPS+BDS+GLO
        #else
            g_gnss_work_model = DEV_GPS_GLO;      //��BDS GPS+GLO
        #endif
    #endif

//    main_handle_g.cfg.el = 5;   //�ϵ��ֹ��Ĭ������Ϊ5��

	if((fd = fopen(NET_PARA_FILE, "r")) != NULL)
	{
		fread(&main_handle_g.cfg.net, sizeof(main_handle_g.cfg.net), 1, fd);
		fclose(fd);
	}

}

/*****************************************************************************
 �� �� ��  : sys_para_init
 ��������  : ϵͳ������ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��11��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_para_init(void)
{
	GLOBAL_MEMSET(&main_handle_g, 0, sizeof(main_handle_g));
	/*main_handle_g.cfg.usr_lla.lat = 30.736571;
	main_handle_g.cfg.usr_lla.lon = 103.96061;
	main_handle_g.cfg.usr_lla.alt = 519.2;
	main_handle_g.cfg.ubx_enable = TRUE;
	main_handle_g.cfg.gps_leap_sec = 18;
	main_handle_g.cfg.net.net_ip[0] = 192;
	main_handle_g.cfg.net.net_ip[1] = 168;
	main_handle_g.cfg.el = 5;
	main_handle_g.cfg.time_zone = 8;
	main_handle_g.cfg.ocxo_voltage[OCXO_1] = OCXO_DFT_VOLTAGE;*/
	sys_para_load();
  //main_handle_g.cfg.antsup = TRUE;

	// os_event_id_g = osEventFlagsNew(NULL);
	// if(!os_event_id_g)
	// {
	// 	ERR_PRINT(("ȫ���¼�ID��ʼ��ʧ��!!!!\r\n"));
	// }
}

/*****************************************************************************
 �� �� ��  : sys_para_get_gps_leapsec
 ��������  : ��ȡGPS������
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��23��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
S8 sys_para_get_gps_leapsec(void)
{
	return main_handle_g.cfg.gps_leap_sec;
}

/*****************************************************************************
 �� �� ��  : sys_set_ipaddr
 ��������  : ����IP��ַ
 �������  : U8* ip_buf  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��9��15��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_set_ipaddr(U8* ip_buf)
{
	U8 tbuf[10];
	
	netIF_Option opt = netIF_OptionIP4_Address;
	netIP_aton ((char*)ip_buf, NET_ADDR_IP4, tbuf);
	set_status = netIF_SetOption (NET_IF_CLASS_ETH, opt, tbuf, NET_ADDR_IP4_LEN);
	if(set_status == netOK)
	{
		memcpy(main_handle_g.cfg.net.net_ip, tbuf, sizeof(main_handle_g.cfg.net.net_ip));
	}
}

/*****************************************************************************
 �� �� ��  : sys_set_maskaddr
 ��������  : ������������
 �������  : U8* ip_buf  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��9��15��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_set_maskaddr(U8* ip_buf)
{
	U8 tbuf[10];
	netIF_Option opt = netIF_OptionIP4_SubnetMask;
	netIP_aton ((char*)ip_buf, NET_ADDR_IP4, tbuf);
	set_status = netIF_SetOption (NET_IF_CLASS_ETH, opt, tbuf, NET_ADDR_IP4_LEN);
	if(set_status == netOK)
	{
		memcpy(main_handle_g.cfg.net.net_mask, tbuf, sizeof(main_handle_g.cfg.net.net_mask));
	}
}

/*****************************************************************************
 �� �� ��  : sys_set_gatewayaddr
 ��������  : ��������
 �������  : U8* ip_buf  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��9��15��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_set_gatewayaddr(U8* ip_buf)
{
	U8 tbuf[10];
	netIF_Option opt = netIF_OptionIP4_DefaultGateway;
	netIP_aton ((char*)ip_buf, NET_ADDR_IP4, tbuf);
	set_status = netIF_SetOption (NET_IF_CLASS_ETH, opt, tbuf, NET_ADDR_IP4_LEN);
	if(set_status == netOK)
	{
		memcpy(main_handle_g.cfg.net.net_gateway, tbuf, sizeof(main_handle_g.cfg.net.net_gateway));
	}
}

/*****************************************************************************
 �� �� ��  : sys_set_macaddr
 ��������  : ����MAC��ַ
 �������  : U8* ip_buf  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��9��3��
    ��    ��   : sunj
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_set_macaddr(U8* ip_buf)
{
	U8 tbuf[10];
	netIF_Option opt = netIF_OptionMAC_Address;
	netMAC_aton ((char*)ip_buf, tbuf);
	set_status = netIF_SetOption (NET_IF_CLASS_ETH, opt, tbuf, NET_ADDR_ETH_LEN);
	if(set_status == netOK)
	{
		memcpy(main_handle_g.cfg.net.net_mac, tbuf, sizeof(main_handle_g.cfg.net.net_mac));
	}
}

/*****************************************************************************
 �� �� ��  : sys_para_set_authcode
 ��������  : ������Ȩ��
 �������  : U32 code  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2018��9��5�� ������
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
U8 sys_para_set_authcode(U32 code)
{
    license_handle_g.license = code;
    GLOBAL_HEX(license_handle_g.license);
	return TRUE;
}


void sys_net_cfg_start(void)
{
	//�������
	netIF_SetOption (NET_IF_CLASS_ETH, netIF_OptionIP4_Address, main_handle_g.cfg.net.net_ip, NET_ADDR_IP4_LEN); //IP����
	netIF_SetOption (NET_IF_CLASS_ETH, netIF_OptionIP4_SubnetMask, main_handle_g.cfg.net.net_mask, NET_ADDR_IP4_LEN); 	//MASK����
	netIF_SetOption (NET_IF_CLASS_ETH, netIF_OptionIP4_DefaultGateway, main_handle_g.cfg.net.net_gateway, NET_ADDR_IP4_LEN);	//IP����
	if(!net_mac_is_legal(main_handle_g.cfg.net.net_mac))
	{
		NOTE_PRINT(("ʹ��ϵͳ��������MAC!!!\r\n"));
		memcpy(main_handle_g.cfg.net.net_mac, main_handle_g.p_monitor->dev_mac, NET_ADDR_ETH_LEN);
	}
	netIF_SetOption (NET_IF_CLASS_ETH, netIF_OptionMAC_Address, main_handle_g.cfg.net.net_mac, NET_ADDR_ETH_LEN);	//MAC����
}



/*eof*/
