
/*****************************************************************************
 函 数 名  : include "pubdefh"
 功能描述  : 调试模块
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月21日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
#include "pubdef.h"

static U8 sys_dbg_flag[SYS_DEBUG_NUM];

/*****************************************************************************
 函 数 名  : sys_debug_init
 功能描述  : 初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月21日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
void sys_debug_init(void)
{
	GLOBAL_MEMSET(sys_dbg_flag, 0, sizeof(sys_dbg_flag));
	sys_dbg_flag[SYS_DEBUG_NOTE] = 1;
	sys_dbg_flag[SYS_DEBUG_WARN] = 1;
	sys_dbg_flag[SYS_DEBUG_ERROR] = 1;
	//sys_dbg_flag[SYS_DEBUG_POST] = 1;
	//sys_dbg_flag[SYS_DEBUG_SOCKET] = 1;
	// sys_dbg_flag[SYS_DEBUG_PL] = 1;  //modified by sunj 2019/12/18 10:50
}

/*****************************************************************************
 函 数 名  : sys_debug_set_type
 功能描述  : 使能调试选项
 输入参数  : SYS_DEBUG_TYPE_Enum type  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月21日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
void sys_debug_set_type(SYS_DEBUG_TYPE_Enum type)
{
	sys_dbg_flag[type] = TRUE;
}

/*****************************************************************************
 函 数 名  : sys_debug_clear_type
 功能描述  : 清楚调试选项
 输入参数  : SYS_DEBUG_TYPE_Enum type  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月21日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
void sys_debug_clear_type(SYS_DEBUG_TYPE_Enum type)
{
	sys_dbg_flag[type] = FALSE;
}

/*****************************************************************************
 函 数 名  : sys_debug_get_type
 功能描述  : 获取调试选项状态
 输入参数  : SYS_DEBUG_TYPE_Enum type  
 输出参数  : 无
 返 回 值  : unsigned
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年7月21日
    作    者   : Winlab
    修改内容   : 新生成函数

*****************************************************************************/
unsigned char sys_debug_get_type(SYS_DEBUG_TYPE_Enum type)
{
	return sys_dbg_flag[type];
}

/*****************************************************************************
 函 数 名  : sys_debug_clear_all
 功能描述  : 清除调试状态
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月15日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void sys_debug_clear_all(void)
{
	U8 i = 0;

	for(i = 0; i < SYS_DEBUG_NUM; i++)
	{
		if((i == SYS_DEBUG_WARN) || (i == SYS_DEBUG_ERROR) || (i == SYS_DEBUG_NOTE))
		{
			continue;
		}
		sys_dbg_flag[i] = FALSE;
	}
}


/*eof*/


