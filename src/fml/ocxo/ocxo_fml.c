/******************************************************************************

                  版权所有 (C), 2018-2025, GridSatellite

 ******************************************************************************
  文 件 名   : ocxo_fml.c
  版 本 号   : 初稿
  作    者   : wzq
  生成日期   : 2019年6月17日
  最近修改   :
  功能描述   : OCXO功能模块
  函数列表   :
              _ocxo_calc_freq_acc
              ocxo_fml_init
              _ocxo_get_err
              _ocxo_get_coarse_phase_err
              _ocxo_pid_para_init
              _ocxo_pid_tame
              _ocxo_status_change
              _ocxo_sync_1pps_enable
              _ocxo_sync_is_enable
              _ocxo_tame_is_enable
              ocxo_tame_process
              _ocxo_write_voltage
  修改历史   :
  1.日    期   : 2019年6月17日
    作    者   : wzq
    修改内容   : 创建文件

******************************************************************************/


/*------------------------------头文件------------------------------*/
#include "ocxo/ocxo_fml.h"
#include "project_def.h"
#include "dac/dac_drv.h"
#include "gnss/ubx.h"
/*------------------------------头文件------------------------------*/


/*------------------------------文件宏------------------------------*/
#define PID_FAST_PERIOD	3
#define PID_NORMAL_PERIOD	5



#define FPGA_CLK_UNIT                   4           //FPGA 鉴相精度ns
#define OCXO_PID_MAX_PHASE_ERR          1000		//满足PID驯服的最大相差1000ns
#define OCXO_MAX_PREHEAT_COLD_START     300         //冷启动
#define OCXO_MAX_PREHEAT_HEAT_START     5           //热启动
#define OCXO_PHASE_ERR_IN_LOCK          30			//判定锁定的相差
#define OCXO_PHASE_ERR_UN_LOCK          100			//判定非锁定的相差
#define OCXO_ADGUGE_LOCK_TIMES          100			//判断进入锁定的次数
#define OCXO_ADGUGE_HOLD_TIMES          7200		//判断进入保持的最少锁定时间
#define OCXO_MAX_HOLD_TIMES             3600		//保持最长时间
#define OCXO_LOCK2TRACE_MAX_TIMES       3           //锁定到跟踪的判定次数
#define OCXO_HEAT_STAR_TEMPER           (30)
/*------------------------------文件宏------------------------------*/


/*------------------------------文件变量------------------------------*/
static pid_para_t ocxo_pid_para[OCXO_NUM][PID_TYPE_NUM];
static F32 err_window[OCXO_NUM][60];
static ocxo_handle_t ocxo_handle[OCXO_NUM];
static F32 fpga_diff_err = 0;	//FPGA鉴相值
/*------------------------------文件变量------------------------------*/


/*------------------------------函数声明------------------------------*/

/*------------------------------函数声明------------------------------*/


/*****************************************************************************
 函 数 名  : _pid_para_init
 功能描述  : PID参数初始化
 输入参数  : S32 voltage  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月17日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static void _ocxo_pid_para_init(U8 ocxo_type, S32 voltage)
{
	GLOBAL_MEMSET(ocxo_pid_para[ocxo_type], 0x0, sizeof(ocxo_pid_para[ocxo_type]));
	ocxo_pid_para[ocxo_type][PID_NORMAL].pid_kp = 100;
	ocxo_pid_para[ocxo_type][PID_NORMAL].pid_ki = 10;
	ocxo_pid_para[ocxo_type][PID_NORMAL].pid_kd = 0;
	ocxo_pid_para[ocxo_type][PID_NORMAL].c_scale = 0.3;
	ocxo_pid_para[ocxo_type][PID_NORMAL].voltage = voltage;
	ocxo_pid_para[ocxo_type][PID_NORMAL].period = PID_NORMAL_PERIOD;

	ocxo_pid_para[ocxo_type][PID_FAST].pid_kp = 100;
	ocxo_pid_para[ocxo_type][PID_FAST].pid_ki = 10;
	ocxo_pid_para[ocxo_type][PID_FAST].pid_kd = 0;
	ocxo_pid_para[ocxo_type][PID_FAST].c_scale = 0.5;
	ocxo_pid_para[ocxo_type][PID_FAST].voltage = voltage;
	ocxo_pid_para[ocxo_type][PID_FAST].period = PID_FAST_PERIOD;
}

//写入DA值
static void _ocxo_write_voltage(U8 ocxo_type, S32 voltage)
{
	U8 dac_ch = DAC_CHANNEL_1;
	
	ocxo_handle[ocxo_type].voltage = voltage;
	ocxo_handle[ocxo_type].vco = ((F32)voltage)/OCXO_FULL_VOLTAGE*5;
	/* 设置DA输出 */
	dac_ch = (ocxo_type == OCXO_1) ? DAC_CHANNEL_2 : DAC_CHANNEL_1;
	dac_write_val(dac_ch, voltage);
}


/*****************************************************************************
 函 数 名  : _ocxo_calc_freq_acc
 功能描述  : 计算频率准确度
 输入参数  : U8 ocxo_type  
             F32 diff_err  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月19日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static void _ocxo_calc_freq_acc(U8 ocxo_type, F32 diff_err)
{
	#define FREQ_ACC_BUF_LEN	100
	#define FREQ_ACC_CALC_LEN	10
	static F32 freq_acc_buf[OCXO_NUM][FREQ_ACC_BUF_LEN];
	static U8 calc_count[OCXO_NUM] = {0}, freq_acc_valid[OCXO_NUM] = {0};	//计算计数值
	U8 i = 0;
	F32 freq_acc_start = 0, freq_acc_stop = 0;
	
	if(get_cur_ref() != REF_NONE)
	{
		if(freq_acc_valid[ocxo_type])
		{
			for(i = 0; i < (FREQ_ACC_BUF_LEN-1); i++)
			{
				freq_acc_buf[ocxo_type][i] = freq_acc_buf[ocxo_type][i+1];
			}
			freq_acc_buf[ocxo_type][FREQ_ACC_BUF_LEN-1] = diff_err;
			for(i = 0; i < FREQ_ACC_CALC_LEN; i++)
			{
				freq_acc_start += freq_acc_buf[ocxo_type][i];
			}
			freq_acc_start = freq_acc_start/FREQ_ACC_CALC_LEN;
			for(i = (FREQ_ACC_BUF_LEN-FREQ_ACC_CALC_LEN); i < FREQ_ACC_BUF_LEN; i++)
			{
				freq_acc_stop += freq_acc_buf[ocxo_type][i];
			}
			freq_acc_stop = freq_acc_stop/FREQ_ACC_CALC_LEN;

			ocxo_handle[ocxo_type].freq_acc = (freq_acc_stop - freq_acc_start)/(FREQ_ACC_BUF_LEN-FREQ_ACC_CALC_LEN)*(F32)1e-9;
			if(ABS_DATA(ocxo_handle[ocxo_type].freq_acc) < (F32)1e-13)
			{
				ocxo_handle[ocxo_type].freq_acc = 1e-13;
			}
		}
		else
		{
			freq_acc_buf[ocxo_type][calc_count[ocxo_type]] = ocxo_handle[ocxo_type].time_acc;
		}
		calc_count[ocxo_type] = (calc_count[ocxo_type]+1)%FREQ_ACC_BUF_LEN;
		if(calc_count[ocxo_type] == 0)
		{
			freq_acc_valid[ocxo_type] = TRUE;
			
		}
	}
}

/*****************************************************************************
 函 数 名  : _ocxo_get_fpga_phase_err
 功能描述  : 获取FPGA相差
 输入参数  : U8 ocxo_type  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月9日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static F32 _ocxo_get_fpga_phase_err(U8 ocxo_type)
{
	U16 fpga_val = 0;
	F32 err = 0;

	fpga_val = fmc_read_reg(FMC_BASE_TIME_CTRL, BASE_TC_PHASE_ERR_VAL);
	err = (fpga_val & 0x7FFF) * FPGA_CLK_UNIT;
	if(APP_IS_BIT_SET(fpga_val, 15))
	{
		err *= (-1);
	}
	fmc_write_reg(FMC_BASE_TIME_CTRL, BASE_TC_PHASE_ERR_CLR, 1);	//清除鉴相值标记
	return err;
}

/*****************************************************************************
 函 数 名  : _ocxo_get_coarse_phase_err
 功能描述  : 获取FPGA相位差
 输入参数  : U8 ocxo_type  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月19日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static F32 _ocxo_get_coarse_phase_err(U8 ocxo_type)
{
	F32 ret = 0;

	switch(ocxo_type)
	{
		case OCXO_1:
		{
			if(get_cur_ref() != REF_NONE)
			{
				ret = _ocxo_get_fpga_phase_err(ocxo_type);
			}
			break;
		}
		default:
			break;
	}
	return ret;
}

/*****************************************************************************
 函 数 名  : _ocxo_get_err
 功能描述  : 获取PPS相位差
 输入参数  : U8 ocxo_type  
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月18日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static F32 _ocxo_get_err(U8 ocxo_type)
{
	F32 err = 0;

	if(get_cur_ref() != REF_NONE)
	{
		/*获取相位差*/
		fpga_diff_err = _ocxo_get_coarse_phase_err(ocxo_type);
		err = fpga_diff_err;
		if(abs(fpga_diff_err) < TDC_MEASURE_RANGE)
		{
			if(tdc_get_measure(&err) == TRUE)
			{
				#ifdef UBLOX_USED
				if(get_cur_ref() == REF_GNSS)
				{
					err = err - gnss_ublox_get_1pps_err();
				}
				#endif
			}
		}
		if(get_cur_ref() == REF_BDC)
		{
			err += 80;
		}
		_ocxo_calc_freq_acc(ocxo_type, err);
	}
	return err;
}

/*****************************************************************************
 函 数 名  : get_ocxo_status
 功能描述  : 获取OCXO状态
 输入参数  : U8 type  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年8月15日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
U8 get_ocxo_status(U8 type)
{
	return ocxo_handle[type].status;
}

// 1PPS同步操作使能
static void _ocxo_sync_1pps_enable(U8 ocxo_type)
{
	/* 相差过大时同步操作 */
	fmc_write_reg(FMC_BASE_TIME_CTRL, BASE_TC_SRCUP_EN, 1);
}

//获取同步操作使能
static U8 _ocxo_sync_is_enable(U8 ocxo_type)
{
	U8 ret = FALSE;

    //当晶振状态为跟踪状态
	if(ocxo_handle[ocxo_type].status == OCXO_TRACE)
	{
		//if((ABS_DATA(ocxo_handle.time_acc) > TAME_PID_MAX_PHASE_ERR) || (pps_diff_ready_flag == 0))
		if(ABS_DATA(ocxo_handle[ocxo_type].time_acc) > OCXO_PID_MAX_PHASE_ERR)
		{
			ocxo_handle[ocxo_type].sync_times++;
		}
		else
		{
			ocxo_handle[ocxo_type].sync_times = 0;
		}

		if(ocxo_handle[ocxo_type].sync_times > 3)
		{
			ocxo_handle[ocxo_type].sync_times = 0;
			ret = TRUE;
		}
		/*if(!first_sync_flag)
		{
			first_sync_flag = TRUE;
			ret = TRUE;
		}*/
	}
	else
	{
		ocxo_handle[ocxo_type].sync_times = 0;
	}

	return ret;
}

//判断是否满足驯服条件
/* 预热完成 参考有效 */
static U8 _ocxo_tame_is_enable(U8 ocxo_type)
{
	U8 ret = FALSE;

	if(((ocxo_handle[ocxo_type].status == OCXO_TRACE) || (ocxo_handle[ocxo_type].status == OCXO_LOCK)) && (ABS_DATA(ocxo_handle[ocxo_type].time_acc) < OCXO_PID_MAX_PHASE_ERR) 
		&& (get_cur_ref() != REF_NONE)
		)
	{
		ret = TRUE;
	}
	return ret;
}

//判断并切换驯服状态
static void _ocxo_status_change(U8 ocxo_type)
{

	switch(ocxo_handle[ocxo_type].status)
	{
		case OCXO_PREHEAT:	//预热
		{
			ocxo_handle[ocxo_type].preheat_cnt++;
			if(get_cur_ref() != REF_NONE)
			{
				if((ocxo_handle[ocxo_type].start_type == OCXO_START_COLD) && (ocxo_handle[ocxo_type].preheat_cnt > OCXO_MAX_PREHEAT_COLD_START))
				{
					ocxo_handle[ocxo_type].status = OCXO_TRACE;
				}
				else if(ocxo_handle[ocxo_type].preheat_cnt > OCXO_MAX_PREHEAT_HEAT_START)
				{
					ocxo_handle[ocxo_type].status = OCXO_TRACE;
				}
			}
			break;
		}

		case OCXO_TRACE:	//跟踪
		{
			if(get_cur_ref() != REF_NONE)
			{
				ocxo_handle[ocxo_type].trace_cnt++;
				if(ABS_DATA(ocxo_handle[ocxo_type].time_acc) < OCXO_PHASE_ERR_IN_LOCK)
				{
					ocxo_handle[ocxo_type].trace2lock_cnt++;
				}
				else
				{
					ocxo_handle[ocxo_type].trace2lock_cnt = 0;
				}
				if(ocxo_handle[ocxo_type].trace2lock_cnt > OCXO_ADGUGE_LOCK_TIMES)
				{
					ocxo_handle[ocxo_type].trace2lock_cnt = 0;
					ocxo_handle[ocxo_type].status = OCXO_LOCK;
					ocxo_handle[ocxo_type].trace_cnt = 0;
				}
			}
			else
			{
				ocxo_handle[ocxo_type].trace2lock_cnt = 0;
				ocxo_handle[ocxo_type].trace_cnt = 0;
				ocxo_handle[ocxo_type].status = OCXO_UNLOCK;
			}
			break;
		}

		case OCXO_LOCK:	//锁定
		{
			ocxo_handle[ocxo_type].lock_cnt++;
			if(get_cur_ref() == REF_NONE)
			{
				if(ocxo_handle[ocxo_type].lock_cnt > OCXO_ADGUGE_HOLD_TIMES)
				{
					ocxo_handle[ocxo_type].status = OCXO_HOLD;
				}
				else
				{
					ocxo_handle[ocxo_type].status = OCXO_UNLOCK;
				}

				ocxo_handle[ocxo_type].lock_cnt = 0;
				ocxo_handle[ocxo_type].lock2trace_cnt = 0;
			}
			else if(ABS_DATA(ocxo_handle[ocxo_type].time_acc) > OCXO_PHASE_ERR_UN_LOCK)
			{
				ocxo_handle[ocxo_type].lock2trace_cnt++;
				if(ocxo_handle[ocxo_type].lock2trace_cnt > OCXO_LOCK2TRACE_MAX_TIMES)
				{
					ocxo_handle[ocxo_type].status = OCXO_TRACE;
					ocxo_handle[ocxo_type].lock_cnt = 0;
					ocxo_handle[ocxo_type].lock2trace_cnt = 0;
				}
			}
			else
			{
				ocxo_handle[ocxo_type].lock2trace_cnt = 0;
			}
			break;
		}

		case OCXO_HOLD:	//保持
		{
			ocxo_handle[ocxo_type].hold_cnt++;
			if(get_cur_ref() != REF_NONE)
			{
				if(ocxo_handle[ocxo_type].hold_cnt > OCXO_MAX_HOLD_TIMES)
				{
					ocxo_handle[ocxo_type].hold_cnt = 0;
					ocxo_handle[ocxo_type].status = OCXO_UNLOCK;
				}
			}
			else
			{
				ocxo_handle[ocxo_type].hold_cnt = 0;
				ocxo_handle[ocxo_type].status = OCXO_TRACE;
			}
			break;
		}

		case OCXO_UNLOCK:	//失锁
		{
			if(get_cur_ref() != REF_NONE)
			{
				ocxo_handle[ocxo_type].status = OCXO_TRACE;
			}
			break;
		}

		default:
			break;
	}

}

/*****************************************************************************
 函 数 名  : _ocxo_pid_tame
 功能描述  : PID驯服
 输入参数  : pid_para_t* pid  
             S32 err          
 输出参数  : 无
 返 回 值  : static
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月17日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
static void _ocxo_pid_tame(U8 ocxo_type, pid_para_t* pid, F32 err)
{
	F32 tmp_kp = 0, tmp_ki = 0;
	F32 p_err = 0, i_err = 0, err_sum = 0;
	S32 voltage_delta = 0;
	U8 i = 0;

	if(!pid) return;
	
	err_window[ocxo_type][pid->pid_cnt%pid->period] = err;
	pid->pid_cnt++;
	if(pid->pid_cnt >= pid->period)
	{
		bubble_sort_float(err_window[ocxo_type], pid->period);
		for(i = 1, err_sum = 0; i < (pid->period-1); i++)
		{
			err_sum += err_window[ocxo_type][i];
		}
		err = err_sum / (pid->period-2);
		pid->pid_cnt = 0;
		DBG_Ocxo_Print("pid err = %.2f ns\r\n", err);

		p_err = err - pid->last_err;       //  e(k) - e(k-1)
		i_err = err;
		tmp_kp = pid->pid_kp * p_err;      // 增量误差 Delta_pk
		tmp_ki = pid->pid_ki * i_err;    	// 积分误差 Delta_ik
		pid->pid_delta = tmp_kp + tmp_ki;
		pid->pre_err = pid->last_err;
		pid->last_err = err;
		pid->pid_delta = pid->pid_delta * pid->c_scale;		//调节缩放因子
		voltage_delta = ABS_DATA(pid->pid_delta);
		if(pid->pid_delta > 0)	// PID补偿值大于0
		{
			if ( (pid->voltage + voltage_delta) < OCXO_MAX_VOLTAGE)	//没到满偏值
			{
				pid->voltage += voltage_delta;
			}
			else
			{
				ERR_PRINT(("PID超上限(%d)!!!!\r\n", voltage_delta));
				GLOBAL_PRINT(("参考电压:%.6f, PID参数:%.3f, %.3f, %.3f; pid->c_scale = %.3f, pid->pid_delta = %.3f, pid->PrevError = %.3f, pid->LastError = %.3f\r\n",
									pid->voltage,pid->pid_kp, pid->pid_ki, pid->pid_kd, pid->c_scale, pid->pid_delta, pid->pre_err, pid->last_err));
				pid->voltage = OCXO_DFT_VOLTAGE;//ocxo_handle[ocxo_type].init_voltage;	//异常情况跳出,让压控恢复到上电状态的起控电压
			}
		}
		else
		{
			if (pid->voltage > voltage_delta)
			{
				pid->voltage  -= voltage_delta;
			}
			else
			{
				ERR_PRINT(("PID超下限(%d)!!!!\r\n", voltage_delta));
				GLOBAL_PRINT(("参考电压:%.6f, PID参数:%.3f, %.3f, %.3f; pid->c_scale = %.3f, pid->pid_delta = %.3f, pid->PrevError = %.3f, pid->LastError = %.3f\r\n",
									pid->voltage,pid->pid_kp, pid->pid_ki, pid->pid_kd, pid->c_scale, pid->pid_delta, pid->pre_err, pid->last_err));
				pid->voltage = OCXO_DFT_VOLTAGE;//ocxo_handle[ocxo_type].init_voltage;	//异常情况跳出,让压控恢复到上电状态的起控电压
			}
		}
		if((pid->voltage >= OCXO_MAX_VOLTAGE) || (pid->voltage < OCXO_MIN_VOLTAGE))
		{
			GLOBAL_PRINT(("参考电压:%.6f, PID参数:%.3f, %.3f, %.3f; pid->c_scale = %.3f, pid->pid_delta = %.3f, pid->PrevError = %.3f, pid->LastError = %.3f\r\n",
									pid->voltage,pid->pid_kp, pid->pid_ki, pid->pid_kd, pid->c_scale, pid->pid_delta, pid->pre_err, pid->last_err));
			pid->voltage = OCXO_DFT_VOLTAGE;//ocxo_handle[ocxo_type].init_voltage;
		}
		_ocxo_write_voltage(ocxo_type, pid->voltage);
	}
	
}

/*****************************************************************************
 函 数 名  : ocxo_tame_process
 功能描述  : OCXO操作
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月17日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void ocxo_tame_process(void)
{
	U8 i = 0;

	for(i = 0; i < OCXO_NUM; i++)
	{
		ocxo_handle[i].time_acc = _ocxo_get_err(i);
		if(_ocxo_sync_is_enable(i))	//同步使能
		{
			WARN_PRINT(("1PPS同步!!!!!\r\n"));
			_ocxo_sync_1pps_enable(i);
			ocxo_handle[i].p_tame_pid->last_err = 0;
			ocxo_handle[i].after_sync_cnt++;
		}
		if((ocxo_handle[i].after_sync_cnt) && (ocxo_handle[i].after_sync_cnt++ < 3))
		{
			return;
		}
		ocxo_handle[i].after_sync_cnt = 0;
		_ocxo_status_change(i);	//驯服状态切换
		if(_ocxo_tame_is_enable(i))
		{
			if(ocxo_handle[i].lock_cnt > 300)
			{
				ocxo_handle[i].p_tame_pid = &(ocxo_pid_para[i][PID_NORMAL]);
				ocxo_handle[i].pid_flag = PID_NORMAL;
			}
			else
			{
				ocxo_handle[i].p_tame_pid = &(ocxo_pid_para[i][PID_FAST]);
				ocxo_handle[i].pid_flag = PID_FAST;
			}
			_ocxo_pid_tame(i, ocxo_handle[i].p_tame_pid, ocxo_handle[i].time_acc);
			ocxo_handle[i].p_tame_pid->voltage = ocxo_handle[i].voltage;
			if((ocxo_handle[i].status == OCXO_LOCK) && (ocxo_handle[i].lock_cnt == 3600))		//锁定1小时后
			{
				//sys_para_save();
			}
		}
		DBG_Ocxo_Print("OCXO_INFO[%u]: status:%u[%u], vco:%.4fV(%u), time_acc:%.1f[%.0f, %.3f]ns, freq_acc:%.2E, preheat_cnt: %us, trace_cnt:%us, lock_cnt:%us, hold_cnt:%us.\r\n",
						i, ocxo_handle[i].status, ocxo_handle[i].pid_flag, ocxo_handle[i].vco, ocxo_handle[i].voltage,
						ocxo_handle[i].time_acc, fpga_diff_err, gnss_ublox_get_1pps_err(),  ocxo_handle[i].freq_acc,
						ocxo_handle[i].preheat_cnt, ocxo_handle[i].trace_cnt, ocxo_handle[i].lock_cnt, ocxo_handle[i].hold_cnt);
		DBG_Ocxo_Print("参考电压:%d, PID参数:%.3f, %.3f, %.3f; period = %u, c_scale = %.3f, pid_delta = %.3f, PrevError = %.3f, LastError = %.3f\r\n",
									ocxo_handle[i].p_tame_pid->voltage,ocxo_handle[i].p_tame_pid->pid_kp, 
									ocxo_handle[i].p_tame_pid->pid_ki, ocxo_handle[i].p_tame_pid->pid_kd, 
									ocxo_handle[i].p_tame_pid->period, ocxo_handle[i].p_tame_pid->c_scale, 
									ocxo_handle[i].p_tame_pid->pid_delta, ocxo_handle[i].p_tame_pid->pre_err, ocxo_handle[i].p_tame_pid->last_err);
	}
}


/*****************************************************************************
 函 数 名  : ocxo_fml_init
 功能描述  : OCXO初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2019年6月17日
    作    者   : wzq
    修改内容   : 新生成函数

*****************************************************************************/
void ocxo_fml_init(IN U8 ocxo_type, INOUT ocxo_handle_t** p_handle)
{
	GLOBAL_MEMSET(&ocxo_handle[ocxo_type], 0x0, sizeof(ocxo_handle[ocxo_type]));
	*p_handle = &ocxo_handle[ocxo_type];
	(*p_handle)->init_voltage = main_handle_g.cfg.ocxo_voltage[ocxo_type];
	_ocxo_pid_para_init(ocxo_type, (*p_handle)->init_voltage);
	_ocxo_write_voltage(ocxo_type, (*p_handle)->init_voltage);
	(*p_handle)->start_type = OCXO_START_HEAT;
	ocxo_handle[ocxo_type].p_tame_pid = &(ocxo_pid_para[ocxo_type][PID_FAST]);
	ocxo_handle[ocxo_type].pid_flag = PID_FAST;
	GLOBAL_TRACE(("OCXO[%u]初始电压，vco = %fV\r\n", ocxo_type, ocxo_handle[ocxo_type].vco));
	NOTE_PRINT(("OCXO[%u]模块初始化完成!!!!!\r\n", ocxo_type));
}

/*eof*/

