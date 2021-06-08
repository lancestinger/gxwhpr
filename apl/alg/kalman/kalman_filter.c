/*
 * FileName : kalman_filter.c
 * Author   : xiahouzuoxin @163.com
 * Version  : v1.0
 * Date     : 2014/9/24 20:36:51
 * Brief    : 
 * 
 * Copyright (C) MICL,USTB
 */
 
#include "kalman_filter.h"

/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = 1;
 *     H = 1; 
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 *   state - Klaman filter structure
 *   init_x - initial x state value   
 *   init_p - initial estimated error convariance
 * @outputs 
 * @retval  
 */
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;//初始�?    state->p = init_p;
    state->A = 1;//状�?�转移矩�?    state->H = 1;//观测矩阵
    state->q = 2e2;//10e-6;  /* predict noise convariance */
    state->r = 5e2;//10e-5;  /* measure error convariance */
}

/*
 * @brief   
 *   1 Dimension Kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 * @retval  
 *   Estimated result
 */
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict 预测�?*/
    state->x = state->A * state->x + state->B * state->u;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement 修正�?*/
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);//卡尔曼增�?    state->x = state->x + state->gain * (z_measure - state->H * state->x);//输出�?    state->p = (1 - state->gain * state->H) * state->p;//方差

    return state->x;
}

/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0}; 
 *   and @q,@r are valued after prior tests. 
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 * @outputs 
 * @retval  
 */
void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2])
{
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7;  /* estimated error convariance */
}

/*
 * @brief   
 *   2 Dimension kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 *   state->x[0] - Updated state value, Such as angle,velocity
 *   state->x[1] - Updated state value, Such as diffrence angle, acceleration
 *   state->p    - Updated estimated error convatiance matrix
 * @retval  
 *   Return value is equals to state->x[0], so maybe angle or velocity.
 */
float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;

    /* Step1: Predict */
	 //预测当前角�?�度状�?��?? 自变�?互变�?由于state->A[0][1]�?.1，所以角度的变化是由角�?�度的测�?0.1得到角度，现有的角度，此式子即是角�?�度积分累加成角�?
	state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
	//预测当前角度状�?��?? 自变�?互变�? 由于state->A[1][0]�?，所以角速度的变化是由角速度的测量自己决�?    
	state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
		//预测当前角度状�?��?? 自变�?互变�? 由于state->A[1][0]�?，所以角速度的变化是由角速度的测量自己决�?    
	state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
	//预测当前角度状�?��?? 自变�?互变�? 由于state->A[1][0]�?，所以角速度的变化是由角速度的测量自己决�?
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: Measurement */ 
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
	// 角度中间变量p * H^T
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
	//卡尔曼增益计算公式的分母，测量噪声在此融合r + H * p * H^T]^(-1)，因为角度是计算出来的，角�?�度是测量出来的，唯�?测量噪声只有�?�?    
	temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;//计算角度的卡尔曼增益
    state->gain[1] = temp1 / temp;//计算角�?�度的卡尔曼增益
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
	//更新当前状�?�，卡尔曼滤波器也在此输�?
	state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}

void kalman_doppler_init(kalman_dop_state *state, float *init_x, float (*init_p)[2], float delta_t)
{
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = delta_t;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = delta_t;
    /* measure noise convariance */
    state->q[0][0] = 1;
    state->q[1][1] = 1;
    state->r       = 10;  /* estimated error convariance */
}


float kalman_filter_doppler(kalman_dop_state *state, float z_measure)
{
    float temp0 = 0.0f; 
    float temp1 = 0.0f;
    float temp = 0.0f;
    kalman2_state *cache_state;
    kalman2_state *pred_state;

    memset(&cache_state,0,sizeof(cache_state));

    /* Step1: Predict */
	cache_state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];   
	cache_state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
	state->x[0] = cache_state->x[0];
	state->x[1] = cache_state->x[1];
    /* p(n|n-1)=A*p(n-1|n-1)*A'+q */
    //A*P
	cache_state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    cache_state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[0][1] * state->p[1][1];
    cache_state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[1][1] * state->p[1][0];
    cache_state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
	//Cache_p*A'+q
    pred_state->p[0][0] = cache_state->p[0][0] * state->A[0][0] + cache_state->p[0][1] * state->A[0][1] + state->q[0][0];
    pred_state->p[0][1] = cache_state->p[0][0] * state->A[1][0] + cache_state->p[0][1] * state->A[1][1] + state->q[0][1];
    pred_state->p[1][0] = cache_state->p[1][0] * state->A[0][0] + cache_state->p[1][1] * state->A[0][1] + state->q[1][0];
    pred_state->p[1][1] = cache_state->p[1][0] * state->A[1][0] + cache_state->p[1][1] * state->A[1][1] + state->q[1][1];
    

    /* Step2: Measurement */  
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = pred_state->p[0][0] * state->H[0] + pred_state->p[0][1] * state->H[1];
    temp1 = pred_state->p[1][0] * state->H[0] + pred_state->p[1][1] * state->H[1];
	temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];

	state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}




//2. 以高度为�?定义卡尔曼结构体并初始化参数
KFP KFP_height={0,0,0,0.001,0.543};

//1. 结构体类型定�?//typedef struct 
//{
//    float LastP;//上次估算协方�?初始化�?�为0.02
//    float out;//卡尔曼滤波器输出 初始化�?�为0
//    float Kg;//卡尔曼增�?初始化�?�为0
//    float Q;//过程噪声协方�?初始化�?�为0.001
//    float R;//观测噪声协方�?初始化�?�为0.543
//} KFP;//Kalman Filter parameter
//

/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input �?要滤波的参数的测量�?�（即传感器的采集�?�）
 *@return 滤波后的参数（最优�?�）
 */
 float kalmanFilter(KFP *kfp,float input)
 {
     //预测协方差方程：k时刻系统估算协方�?= k-1时刻的系统协方差 + 过程噪声协方�?     
     kfp->LastP = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增�?= k时刻系统估算协方�?/ （k时刻系统估算协方�?+ 观测噪声协方差）
     kfp->Kg = kfp->LastP / (kfp->LastP + kfp->R);
     //更新�?优�?�方程：k时刻状�?�变量的�?优�??= 状�?�变量的预测�?+ 卡尔曼增�?* （测量�??- 状�?�变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上�?次的输出�?     //更新协方差方�? 本次的系统协方差付给 kfp->LastP 威下�?次运算准备�??     
     kfp->LastP = (1-kfp->Kg) * kfp->LastP;
     return kfp->out;
 }

/**
 *调用卡尔曼滤波器 实践
 */
//int height;
//int kalman_height=0;
//kalman_height = kalmanFilter(&KFP_height,(float)height);
//
