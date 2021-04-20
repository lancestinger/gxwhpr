/******************************************************************************

                  ��Ȩ���� (C), 2018-2025, GridSatellite

 ******************************************************************************
  �� �� ��   : ocxo_fml.c
  �� �� ��   : ����
  ��    ��   : wzq
  ��������   : 2019��6��17��
  ����޸�   :
  ��������   : OCXO����ģ��
  �����б�   :
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
  �޸���ʷ   :
  1.��    ��   : 2019��6��17��
    ��    ��   : wzq
    �޸�����   : �����ļ�

******************************************************************************/


/*------------------------------ͷ�ļ�------------------------------*/
#include "ocxo/ocxo_fml.h"
#include "project_def.h"
#include "dac/dac_drv.h"
#include "gnss/ubx.h"
/*------------------------------ͷ�ļ�------------------------------*/


/*------------------------------�ļ���------------------------------*/
#define PID_FAST_PERIOD	3
#define PID_NORMAL_PERIOD	5



#define FPGA_CLK_UNIT                   4           //FPGA ���ྫ��ns
#define OCXO_PID_MAX_PHASE_ERR          1000		//����PIDѱ����������1000ns
#define OCXO_MAX_PREHEAT_COLD_START     300         //������
#define OCXO_MAX_PREHEAT_HEAT_START     5           //������
#define OCXO_PHASE_ERR_IN_LOCK          30			//�ж����������
#define OCXO_PHASE_ERR_UN_LOCK          100			//�ж������������
#define OCXO_ADGUGE_LOCK_TIMES          100			//�жϽ��������Ĵ���
#define OCXO_ADGUGE_HOLD_TIMES          7200		//�жϽ��뱣�ֵ���������ʱ��
#define OCXO_MAX_HOLD_TIMES             3600		//�����ʱ��
#define OCXO_LOCK2TRACE_MAX_TIMES       3           //���������ٵ��ж�����
#define OCXO_HEAT_STAR_TEMPER           (30)
/*------------------------------�ļ���------------------------------*/


/*------------------------------�ļ�����------------------------------*/
static pid_para_t ocxo_pid_para[OCXO_NUM][PID_TYPE_NUM];
static F32 err_window[OCXO_NUM][60];
static ocxo_handle_t ocxo_handle[OCXO_NUM];
static F32 fpga_diff_err = 0;	//FPGA����ֵ
/*------------------------------�ļ�����------------------------------*/


/*------------------------------��������------------------------------*/

/*------------------------------��������------------------------------*/


/*****************************************************************************
 �� �� ��  : _pid_para_init
 ��������  : PID������ʼ��
 �������  : S32 voltage  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��17��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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

//д��DAֵ
static void _ocxo_write_voltage(U8 ocxo_type, S32 voltage)
{
	U8 dac_ch = DAC_CHANNEL_1;
	
	ocxo_handle[ocxo_type].voltage = voltage;
	ocxo_handle[ocxo_type].vco = ((F32)voltage)/OCXO_FULL_VOLTAGE*5;
	/* ����DA��� */
	dac_ch = (ocxo_type == OCXO_1) ? DAC_CHANNEL_2 : DAC_CHANNEL_1;
	dac_write_val(dac_ch, voltage);
}


/*****************************************************************************
 �� �� ��  : _ocxo_calc_freq_acc
 ��������  : ����Ƶ��׼ȷ��
 �������  : U8 ocxo_type  
             F32 diff_err  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��19��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
static void _ocxo_calc_freq_acc(U8 ocxo_type, F32 diff_err)
{
	#define FREQ_ACC_BUF_LEN	100
	#define FREQ_ACC_CALC_LEN	10
	static F32 freq_acc_buf[OCXO_NUM][FREQ_ACC_BUF_LEN];
	static U8 calc_count[OCXO_NUM] = {0}, freq_acc_valid[OCXO_NUM] = {0};	//�������ֵ
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
 �� �� ��  : _ocxo_get_fpga_phase_err
 ��������  : ��ȡFPGA���
 �������  : U8 ocxo_type  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��8��9��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
	fmc_write_reg(FMC_BASE_TIME_CTRL, BASE_TC_PHASE_ERR_CLR, 1);	//�������ֵ���
	return err;
}

/*****************************************************************************
 �� �� ��  : _ocxo_get_coarse_phase_err
 ��������  : ��ȡFPGA��λ��
 �������  : U8 ocxo_type  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��19��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
 �� �� ��  : _ocxo_get_err
 ��������  : ��ȡPPS��λ��
 �������  : U8 ocxo_type  
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��18��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
static F32 _ocxo_get_err(U8 ocxo_type)
{
	F32 err = 0;

	if(get_cur_ref() != REF_NONE)
	{
		/*��ȡ��λ��*/
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
 �� �� ��  : get_ocxo_status
 ��������  : ��ȡOCXO״̬
 �������  : U8 type  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��8��15��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
U8 get_ocxo_status(U8 type)
{
	return ocxo_handle[type].status;
}

// 1PPSͬ������ʹ��
static void _ocxo_sync_1pps_enable(U8 ocxo_type)
{
	/* ������ʱͬ������ */
	fmc_write_reg(FMC_BASE_TIME_CTRL, BASE_TC_SRCUP_EN, 1);
}

//��ȡͬ������ʹ��
static U8 _ocxo_sync_is_enable(U8 ocxo_type)
{
	U8 ret = FALSE;

    //������״̬Ϊ����״̬
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

//�ж��Ƿ�����ѱ������
/* Ԥ����� �ο���Ч */
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

//�жϲ��л�ѱ��״̬
static void _ocxo_status_change(U8 ocxo_type)
{

	switch(ocxo_handle[ocxo_type].status)
	{
		case OCXO_PREHEAT:	//Ԥ��
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

		case OCXO_TRACE:	//����
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

		case OCXO_LOCK:	//����
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

		case OCXO_HOLD:	//����
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

		case OCXO_UNLOCK:	//ʧ��
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
 �� �� ��  : _ocxo_pid_tame
 ��������  : PIDѱ��
 �������  : pid_para_t* pid  
             S32 err          
 �������  : ��
 �� �� ֵ  : static
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��17��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
		tmp_kp = pid->pid_kp * p_err;      // ������� Delta_pk
		tmp_ki = pid->pid_ki * i_err;    	// ������� Delta_ik
		pid->pid_delta = tmp_kp + tmp_ki;
		pid->pre_err = pid->last_err;
		pid->last_err = err;
		pid->pid_delta = pid->pid_delta * pid->c_scale;		//������������
		voltage_delta = ABS_DATA(pid->pid_delta);
		if(pid->pid_delta > 0)	// PID����ֵ����0
		{
			if ( (pid->voltage + voltage_delta) < OCXO_MAX_VOLTAGE)	//û����ƫֵ
			{
				pid->voltage += voltage_delta;
			}
			else
			{
				ERR_PRINT(("PID������(%d)!!!!\r\n", voltage_delta));
				GLOBAL_PRINT(("�ο���ѹ:%.6f, PID����:%.3f, %.3f, %.3f; pid->c_scale = %.3f, pid->pid_delta = %.3f, pid->PrevError = %.3f, pid->LastError = %.3f\r\n",
									pid->voltage,pid->pid_kp, pid->pid_ki, pid->pid_kd, pid->c_scale, pid->pid_delta, pid->pre_err, pid->last_err));
				pid->voltage = OCXO_DFT_VOLTAGE;//ocxo_handle[ocxo_type].init_voltage;	//�쳣�������,��ѹ�ػָ����ϵ�״̬����ص�ѹ
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
				ERR_PRINT(("PID������(%d)!!!!\r\n", voltage_delta));
				GLOBAL_PRINT(("�ο���ѹ:%.6f, PID����:%.3f, %.3f, %.3f; pid->c_scale = %.3f, pid->pid_delta = %.3f, pid->PrevError = %.3f, pid->LastError = %.3f\r\n",
									pid->voltage,pid->pid_kp, pid->pid_ki, pid->pid_kd, pid->c_scale, pid->pid_delta, pid->pre_err, pid->last_err));
				pid->voltage = OCXO_DFT_VOLTAGE;//ocxo_handle[ocxo_type].init_voltage;	//�쳣�������,��ѹ�ػָ����ϵ�״̬����ص�ѹ
			}
		}
		if((pid->voltage >= OCXO_MAX_VOLTAGE) || (pid->voltage < OCXO_MIN_VOLTAGE))
		{
			GLOBAL_PRINT(("�ο���ѹ:%.6f, PID����:%.3f, %.3f, %.3f; pid->c_scale = %.3f, pid->pid_delta = %.3f, pid->PrevError = %.3f, pid->LastError = %.3f\r\n",
									pid->voltage,pid->pid_kp, pid->pid_ki, pid->pid_kd, pid->c_scale, pid->pid_delta, pid->pre_err, pid->last_err));
			pid->voltage = OCXO_DFT_VOLTAGE;//ocxo_handle[ocxo_type].init_voltage;
		}
		_ocxo_write_voltage(ocxo_type, pid->voltage);
	}
	
}

/*****************************************************************************
 �� �� ��  : ocxo_tame_process
 ��������  : OCXO����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��17��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

*****************************************************************************/
void ocxo_tame_process(void)
{
	U8 i = 0;

	for(i = 0; i < OCXO_NUM; i++)
	{
		ocxo_handle[i].time_acc = _ocxo_get_err(i);
		if(_ocxo_sync_is_enable(i))	//ͬ��ʹ��
		{
			WARN_PRINT(("1PPSͬ��!!!!!\r\n"));
			_ocxo_sync_1pps_enable(i);
			ocxo_handle[i].p_tame_pid->last_err = 0;
			ocxo_handle[i].after_sync_cnt++;
		}
		if((ocxo_handle[i].after_sync_cnt) && (ocxo_handle[i].after_sync_cnt++ < 3))
		{
			return;
		}
		ocxo_handle[i].after_sync_cnt = 0;
		_ocxo_status_change(i);	//ѱ��״̬�л�
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
			if((ocxo_handle[i].status == OCXO_LOCK) && (ocxo_handle[i].lock_cnt == 3600))		//����1Сʱ��
			{
				//sys_para_save();
			}
		}
		DBG_Ocxo_Print("OCXO_INFO[%u]: status:%u[%u], vco:%.4fV(%u), time_acc:%.1f[%.0f, %.3f]ns, freq_acc:%.2E, preheat_cnt: %us, trace_cnt:%us, lock_cnt:%us, hold_cnt:%us.\r\n",
						i, ocxo_handle[i].status, ocxo_handle[i].pid_flag, ocxo_handle[i].vco, ocxo_handle[i].voltage,
						ocxo_handle[i].time_acc, fpga_diff_err, gnss_ublox_get_1pps_err(),  ocxo_handle[i].freq_acc,
						ocxo_handle[i].preheat_cnt, ocxo_handle[i].trace_cnt, ocxo_handle[i].lock_cnt, ocxo_handle[i].hold_cnt);
		DBG_Ocxo_Print("�ο���ѹ:%d, PID����:%.3f, %.3f, %.3f; period = %u, c_scale = %.3f, pid_delta = %.3f, PrevError = %.3f, LastError = %.3f\r\n",
									ocxo_handle[i].p_tame_pid->voltage,ocxo_handle[i].p_tame_pid->pid_kp, 
									ocxo_handle[i].p_tame_pid->pid_ki, ocxo_handle[i].p_tame_pid->pid_kd, 
									ocxo_handle[i].p_tame_pid->period, ocxo_handle[i].p_tame_pid->c_scale, 
									ocxo_handle[i].p_tame_pid->pid_delta, ocxo_handle[i].p_tame_pid->pre_err, ocxo_handle[i].p_tame_pid->last_err);
	}
}


/*****************************************************************************
 �� �� ��  : ocxo_fml_init
 ��������  : OCXO��ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2019��6��17��
    ��    ��   : wzq
    �޸�����   : �����ɺ���

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
	GLOBAL_TRACE(("OCXO[%u]��ʼ��ѹ��vco = %fV\r\n", ocxo_type, ocxo_handle[ocxo_type].vco));
	NOTE_PRINT(("OCXO[%u]ģ���ʼ�����!!!!!\r\n", ocxo_type));
}

/*eof*/

