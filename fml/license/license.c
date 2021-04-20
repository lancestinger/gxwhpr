
// #include "pubdef.h"
// #include "system.h"
// #include "GpsGlobal_Var.h"

#include "project_def.h"

#ifdef NEED_LICENSE
U8 license_valid = FALSE;	//许可证有效标志，如果使能许可证

#define KEY 0x6513A07
static U32 license_calc_generate_code(U32 sn_crc)
{
	U8* ptemp8 = NULL;
	U32 lic = 0;
	U64 temp64 = 0;
	U8 i = 0;
	
	ptemp8 = (U8*)&sn_crc;
	for(i = 0; i < 4; i++)
	{
		ptemp8[i] = ((ptemp8[i]<<4)&0xF0)|((ptemp8[i]>>4)&0x0F);
	}
	//printf("1:chipID = 0x%08X\n", chipID);
	//2.向左循环移位5bit
	temp64 = sn_crc & 0xFFFFFFFF;
	temp64 = temp64 << 5;
	sn_crc = ((temp64 >> 32) | temp64) & 0xFFFFFFFF;
	//printf("2:chipID = 0x%08X\n", chipID);
	//3.与特定秘钥(KEY)做异或运算
	lic = sn_crc ^ KEY;
	return lic;
}

/*****************************************************************************
 函 数 名  : licenseInit
 功能描述  : 许可证初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2018年5月7日 星期一
    作    者   : wenzhiquan
    修改内容   : 新生成函数

*****************************************************************************/
void license_init(void)
{
	U32 sn ;//= U32Swap((U32*)&(main_handle_g.p_monitor->dev_sn[4]));
	if(license_calc_generate_code(sn) == license_handle_g.license)
	{
		license_valid = TRUE;
		NOTE_PRINT(("许可证授权成功!!!!!!\r\n"));
	}
	else
	{
		license_valid = FALSE;
		ERR_LICENSE(("许可证授权失败!!!!!!\r\n"));
	}
}

/*****************************************************************************
 函 数 名  : sys_get_auth_state
 功能描述  : 获取授权状态
 输入参数  : void  
 输出参数  : 
 返 回 值  : 授权状态  TRUE---已授权    FALSE---未授权
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019-10-17
    作    者   : sunj
    修改内容   : 新生成函数

*****************************************************************************/
U8 sys_get_auth_state(void)
{
    return license_valid;
}



#endif
