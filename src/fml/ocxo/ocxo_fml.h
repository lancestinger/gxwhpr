
#ifndef _OCXO_FML_H_
#define _OCXO_FML_H_

#include "pubdef.h"

#define OCXO_DFT_VOLTAGE	(2.1/5.0*(1<<20))//(1 << 19)
#define OCXO_MAX_VOLTAGE	(4.5/5.0*(1<<20))	// 3.5V
#define OCXO_MIN_VOLTAGE 	(1.0/5.0*(1<<20))	// 1.2V
#define OCXO_FULL_VOLTAGE	(1<<20)

typedef enum
{
	OCXO_START_COLD = 0x0,	//������
	OCXO_START_HEAT,		//������
}ocxo_start_enum;

typedef enum
{
	OCXO_PREHEAT = 0,
	OCXO_UNLOCK,
	OCXO_TRACE,
	OCXO_LOCK,
	OCXO_HOLD,
}ocxo_state_enum;

//��Ҫ�����ľ������
typedef enum
{
	OCXO_1 = 0,
	//OCXO_2,
	OCXO_NUM,
}ocxo_type_enum;

//������PID����
typedef enum
{
	PID_NORMAL = 0,
	PID_FAST,
	PID_TYPE_NUM,
}pid_type_enum;

//ѱ��PID���ݽṹ
typedef struct
{
	F32	err_sum;		//�м���̵��ۼӺ�
	F32	pid_kp;			//��������
	F32	pid_ki;			//��������
	F32	pid_kd;			//΢������
	F32	last_err;	    // k-1 ʱ���������
	F32	pre_err;	    // k-2 ʱ���������

	F32	c_scale;		//��������

	F32	pid_delta;		//����ֵ
	S32	voltage;		//���ص�ѹ(DACֵ)

	U32 pid_cnt;		//PID�������ڼ���
	U32 period;			//����
}pid_para_t;


//OCXO״̬���ݽṹ
typedef struct
{
	U8 sync_times;					//
	ocxo_start_enum start_type;	//��������
	ocxo_state_enum status;			//����״̬
	U32 preheat_cnt;					//Ԥ��ʱ��
	U32 trace_cnt;						//����ʱ��
	U32 trace2lock_cnt;					//���ٵ���������ʱ��
	U32 lock_cnt;					//����ʱ��
	U32 hold_cnt;					//����ʱ��
	U32 after_sync_cnt;				//ͬ������
	U32 lock2trace_cnt;				//��������������ʱ��
	pid_type_enum	pid_flag;		//PIDѱ��ģʽ
	F32	vco;						//vco��ѹֵ
	S32 voltage;					//DACѹ��ֵ
	F32 time_acc;					//ʱ�侫��
	F32 freq_acc;					//Ƶ��׼ȷ��
	S32 init_voltage;				//��ʼDACֵ
	pid_para_t* p_tame_pid;
}ocxo_handle_t;

extern U8 get_ocxo_status(U8 type);
extern void ocxo_tame_process(void);
extern void ocxo_fml_init(U8 ocxo_type, ocxo_handle_t** p_handle);
#endif
/*eof*/

