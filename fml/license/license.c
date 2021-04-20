
// #include "pubdef.h"
// #include "system.h"
// #include "GpsGlobal_Var.h"

#include "project_def.h"

#ifdef NEED_LICENSE
U8 license_valid = FALSE;	//���֤��Ч��־�����ʹ�����֤

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
	//2.����ѭ����λ5bit
	temp64 = sn_crc & 0xFFFFFFFF;
	temp64 = temp64 << 5;
	sn_crc = ((temp64 >> 32) | temp64) & 0xFFFFFFFF;
	//printf("2:chipID = 0x%08X\n", chipID);
	//3.���ض���Կ(KEY)���������
	lic = sn_crc ^ KEY;
	return lic;
}

/*****************************************************************************
 �� �� ��  : licenseInit
 ��������  : ���֤��ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2018��5��7�� ����һ
    ��    ��   : wenzhiquan
    �޸�����   : �����ɺ���

*****************************************************************************/
void license_init(void)
{
	U32 sn ;//= U32Swap((U32*)&(main_handle_g.p_monitor->dev_sn[4]));
	if(license_calc_generate_code(sn) == license_handle_g.license)
	{
		license_valid = TRUE;
		NOTE_PRINT(("���֤��Ȩ�ɹ�!!!!!!\r\n"));
	}
	else
	{
		license_valid = FALSE;
		ERR_LICENSE(("���֤��Ȩʧ��!!!!!!\r\n"));
	}
}

/*****************************************************************************
 �� �� ��  : sys_get_auth_state
 ��������  : ��ȡ��Ȩ״̬
 �������  : void  
 �������  : 
 �� �� ֵ  : ��Ȩ״̬  TRUE---����Ȩ    FALSE---δ��Ȩ
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019-10-17
    ��    ��   : sunj
    �޸�����   : �����ɺ���

*****************************************************************************/
U8 sys_get_auth_state(void)
{
    return license_valid;
}



#endif
