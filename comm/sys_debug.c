
/*****************************************************************************
 �� �� ��  : include "pubdefh"
 ��������  : ����ģ��
 �������  : ��
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��21��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
#include "pubdef.h"

static U8 sys_dbg_flag[SYS_DEBUG_NUM];

/*****************************************************************************
 �� �� ��  : sys_debug_init
 ��������  : ��ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��21��
    ��    ��   : Winlab
    �޸�����   : �����ɺ���

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
 �� �� ��  : sys_debug_set_type
 ��������  : ʹ�ܵ���ѡ��
 �������  : SYS_DEBUG_TYPE_Enum type  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��21��
    ��    ��   : Winlab
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_debug_set_type(SYS_DEBUG_TYPE_Enum type)
{
	sys_dbg_flag[type] = TRUE;
}

/*****************************************************************************
 �� �� ��  : sys_debug_clear_type
 ��������  : �������ѡ��
 �������  : SYS_DEBUG_TYPE_Enum type  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��21��
    ��    ��   : Winlab
    �޸�����   : �����ɺ���

*****************************************************************************/
void sys_debug_clear_type(SYS_DEBUG_TYPE_Enum type)
{
	sys_dbg_flag[type] = FALSE;
}

/*****************************************************************************
 �� �� ��  : sys_debug_get_type
 ��������  : ��ȡ����ѡ��״̬
 �������  : SYS_DEBUG_TYPE_Enum type  
 �������  : ��
 �� �� ֵ  : unsigned
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��7��21��
    ��    ��   : Winlab
    �޸�����   : �����ɺ���

*****************************************************************************/
unsigned char sys_debug_get_type(SYS_DEBUG_TYPE_Enum type)
{
	return sys_dbg_flag[type];
}

/*****************************************************************************
 �� �� ��  : sys_debug_clear_all
 ��������  : �������״̬
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��8��15��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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

