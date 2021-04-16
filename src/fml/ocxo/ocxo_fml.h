
#ifndef _OCXO_FML_H_
#define _OCXO_FML_H_

#include "pubdef.h"

#define OCXO_DFT_VOLTAGE	(2.1/5.0*(1<<20))//(1 << 19)
#define OCXO_MAX_VOLTAGE	(4.5/5.0*(1<<20))	// 3.5V
#define OCXO_MIN_VOLTAGE 	(1.0/5.0*(1<<20))	// 1.2V
#define OCXO_FULL_VOLTAGE	(1<<20)

typedef enum
{
	OCXO_START_COLD = 0x0,	//冷启动
	OCXO_START_HEAT,		//热启动
}ocxo_start_enum;

typedef enum
{
	OCXO_PREHEAT = 0,
	OCXO_UNLOCK,
	OCXO_TRACE,
	OCXO_LOCK,
	OCXO_HOLD,
}ocxo_state_enum;

//需要调整的晶振个数
typedef enum
{
	OCXO_1 = 0,
	//OCXO_2,
	OCXO_NUM,
}ocxo_type_enum;

//调整的PID类型
typedef enum
{
	PID_NORMAL = 0,
	PID_FAST,
	PID_TYPE_NUM,
}pid_type_enum;

//驯服PID数据结构
typedef struct
{
	F32	err_sum;		//中间过程的累加和
	F32	pid_kp;			//比例因子
	F32	pid_ki;			//积分因子
	F32	pid_kd;			//微分因子
	F32	last_err;	    // k-1 时刻误差增量
	F32	pre_err;	    // k-2 时刻误差增量

	F32	c_scale;		//缩放因子

	F32	pid_delta;		//修正值
	S32	voltage;		//调控电压(DAC值)

	U32 pid_cnt;		//PID调整周期计数
	U32 period;			//周期
}pid_para_t;


//OCXO状态数据结构
typedef struct
{
	U8 sync_times;					//
	ocxo_start_enum start_type;	//启动类型
	ocxo_state_enum status;			//晶振状态
	U32 preheat_cnt;					//预热时间
	U32 trace_cnt;						//跟踪时长
	U32 trace2lock_cnt;					//跟踪到锁定连判时间
	U32 lock_cnt;					//锁定时长
	U32 hold_cnt;					//保持时长
	U32 after_sync_cnt;				//同步后间隔
	U32 lock2trace_cnt;				//锁定到跟踪连判时间
	pid_type_enum	pid_flag;		//PID驯服模式
	F32	vco;						//vco电压值
	S32 voltage;					//DAC压控值
	F32 time_acc;					//时间精度
	F32 freq_acc;					//频率准确度
	S32 init_voltage;				//初始DAC值
	pid_para_t* p_tame_pid;
}ocxo_handle_t;

extern U8 get_ocxo_status(U8 type);
extern void ocxo_tame_process(void);
extern void ocxo_fml_init(U8 ocxo_type, ocxo_handle_t** p_handle);
#endif
/*eof*/

