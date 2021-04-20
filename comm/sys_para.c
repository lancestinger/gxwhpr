
#include "project_def.h"
#include "crc/crc.h"





main_handle_t main_handle_g;	//主操作对象
extra_para_t ex_para_g;
license_handle_t license_handle_g;  //license操作对象
hn_handle_t hn_handle_g;    //hn配置操作对象

osEventFlagsId_t os_event_id_g = NULL;		//全局事件ID
static netStatus set_status = netOK;


device_enum g_gnss_work_model = DEV_GPS_GLO;          //工作模式  0表示GPS&GLONASS  1表示GPS&BDS&GLONASS

DBG_OUT_USART_WORK_MODEL g_dbg_out_usart_model = DBG_OUT_USART_NORMAL_MODEL;     //调试串口工作模式

static const cfg_para_t default_cfg_para = 
{
	1,										//UBX
	18,										//闰秒数
	// 8,										//时区
	// 0,										//上电自启
	// 0,								//参考源选择
	// 0,							//参考源切换模式
	// 0,						//默认压控电压
	// {30.736571, 103.96061, 519.2},		    //位置
	// 5,										//截至角
	// 60,										//最大速度
	// {80, 80, 80, 80},						//衰减值
	// {0, 0, 0, 0},							//时延
	// {0, 1, 0},								//圆周运动参数
	// {0, 0},									//速度
	// {0, 0},									//加速度
	{
		0,									//DHCP
		192,168,10,235,					    //IP
		255,255,255,0,						//MASK
		192,168,10,1,						//GATEWAY
		1,2,3,0,5,6,
        182,61,51,239,
        1883
	}, 
    // 1,                                      //整点自动授时使能
    // 0,                                      //使用当前定位位置作为圆心进行圆周运动使能
    // 192,168,10,100,                         //发送IP1
    // 192,168,10,219,                         //发送IP2
    // 5,                                      //发送状态巡检信息周期
    // {0, 0, 0, 0}                            //通道功率衰减最小设置
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
		NOTE_PRINT(("许可证授权码保存成功!\r\n"));
	}
	else
	{
		ERR_PRINT(("许可证授权码保存失败!\r\n"));
	} 
}

//HN配置信息复位
void hn_cfg_reset(void)
{
    hn_handle_g.type = 1;
}

//HN配置信息保存至文件
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
		NOTE_PRINT(("HN CFG 文件保存成功!\r\n"));
	}
	else
	{
		ERR_PRINT(("HN CFG 文件保存失败!\r\n"));
	} 
}



/*****************************************************************************
 函 数 名  : sys_para_reset
 功能描述  : 参数复位
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月16日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void sys_para_reset(void)
{
	GLOBAL_MEMCPY(&main_handle_g.cfg, &default_cfg_para, sizeof(cfg_para_t));
}

/*****************************************************************************
 函 数 名  : sys_reset_para_save
 功能描述  : 复位参数保存，如果使用，仅会调用一次
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年10月17日
    作    者   : sunj
    修改内容   : 新生成函数

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
		NOTE_PRINT(("复位系统参数保存成功!\r\n"));
	}
	else
	{
		ERR_PRINT(("复位系统参数保存失败!\r\n"));
	} 
}

/*****************************************************************************
 函 数 名  : sys_para_save
 功能描述  : 参数保存
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月15日
    作    者   : wzq
    修改内容   : 新生成函数

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
		NOTE_PRINT(("系统参数保存成功!\r\n"));
	}
	else
	{
		ERR_PRINT(("系统参数保存失败!\r\n"));
	}

	if((fd = fopen(NET_PARA_FILE, "w")) != NULL)
	{
		fwrite(&cfg_para_temp.net, sizeof(cfg_para_temp.net), 1, fd);
		fclose(fd);
		NOTE_PRINT(("net参数保存成功!\r\n"));
	}
	else
	{
		ERR_PRINT(("net参数保存失败!\r\n"));
	}
}

/*****************************************************************************
 函 数 名  : sys_para_load
 功能描述  : 系统参数加载
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月16日
    作    者   : wzq
    修改内容   : 新生成函数

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
		ERR_PRINT(("数据库校验错误，恢复缺省数据!\r\n"));
		sys_para_reset();
		sys_reset_para_save();
	}
	else
	{
		sys_para_reset();
		sys_reset_para_save();
		NOTE_PRINT(("系统参数加载成功!!!!\r\n"));
	}

//    g_gnss_work_model = (device_enum)main_handle_g.cfg.work_model;   //added by sunj 2019-10-16 14:31

    #ifdef VERSION_HUANUO
        #ifdef BDS_INCLUDE
            g_gnss_work_model = DEV_GPS_BDS_GLO;  //有BDS GPS+BDS+GLO
        #else
            g_gnss_work_model = DEV_GPS_GLO;      //无BDS GPS+GLO
        #endif
    #endif

//    main_handle_g.cfg.el = 5;   //上电截止角默认设置为5度

	if((fd = fopen(NET_PARA_FILE, "r")) != NULL)
	{
		fread(&main_handle_g.cfg.net, sizeof(main_handle_g.cfg.net), 1, fd);
		fclose(fd);
	}

}

/*****************************************************************************
 函 数 名  : sys_para_init
 功能描述  : 系统参数初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月11日
    作    者   : wzq
    修改内容   : 新生成函数

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
	// 	ERR_PRINT(("全局事件ID初始化失败!!!!\r\n"));
	// }
}

/*****************************************************************************
 函 数 名  : sys_para_get_gps_leapsec
 功能描述  : 获取GPS闰秒数
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月23日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
S8 sys_para_get_gps_leapsec(void)
{
	return main_handle_g.cfg.gps_leap_sec;
}

/*****************************************************************************
 函 数 名  : sys_set_ipaddr
 功能描述  : 设置IP地址
 输入参数  : U8* ip_buf  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月15日
    作    者   : wzq
    修改内容   : 新生成函数

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
 函 数 名  : sys_set_maskaddr
 功能描述  : 设置子网掩码
 输入参数  : U8* ip_buf  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月15日
    作    者   : wzq
    修改内容   : 新生成函数

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
 函 数 名  : sys_set_gatewayaddr
 功能描述  : 网关设置
 输入参数  : U8* ip_buf  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月15日
    作    者   : wzq
    修改内容   : 新生成函数

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
 函 数 名  : sys_set_macaddr
 功能描述  : 设置MAC地址
 输入参数  : U8* ip_buf  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年9月3日
    作    者   : sunj
    修改内容   : 新生成函数

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
 函 数 名  : sys_para_set_authcode
 功能描述  : 设置授权码
 输入参数  : U32 code  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年9月5日 星期三
    作    者   : wenzhiquan
    修改内容   : 新生成函数

*****************************************************************************/
U8 sys_para_set_authcode(U32 code)
{
    license_handle_g.license = code;
    GLOBAL_HEX(license_handle_g.license);
	return TRUE;
}


void sys_net_cfg_start(void)
{
	//网络参数
	netIF_SetOption (NET_IF_CLASS_ETH, netIF_OptionIP4_Address, main_handle_g.cfg.net.net_ip, NET_ADDR_IP4_LEN); //IP设置
	netIF_SetOption (NET_IF_CLASS_ETH, netIF_OptionIP4_SubnetMask, main_handle_g.cfg.net.net_mask, NET_ADDR_IP4_LEN); 	//MASK设置
	netIF_SetOption (NET_IF_CLASS_ETH, netIF_OptionIP4_DefaultGateway, main_handle_g.cfg.net.net_gateway, NET_ADDR_IP4_LEN);	//IP设置
	if(!net_mac_is_legal(main_handle_g.cfg.net.net_mac))
	{
		NOTE_PRINT(("使用系统生成配置MAC!!!\r\n"));
		memcpy(main_handle_g.cfg.net.net_mac, main_handle_g.p_monitor->dev_mac, NET_ADDR_ETH_LEN);
	}
	netIF_SetOption (NET_IF_CLASS_ETH, netIF_OptionMAC_Address, main_handle_g.cfg.net.net_mac, NET_ADDR_ETH_LEN);	//MAC设置
}



/*eof*/
