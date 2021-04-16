/*
 * FileName : kalman_filter.h
 * Author   : xiahouzuoxin @163.com
 * Version  : v1.0
 * Date     : 2014/9/24 20:37:01
 * Brief    : 
 * 
 * Copyright (C) MICL,USTB
 */
#ifndef  _KALMAN_FILTER_H
#define  _KALMAN_FILTER_H

/* 
 * NOTES: n Dimension means the state is n dimension, 
 * measurement always 1 dimension 
 */

/* 1 Dimension */
typedef struct {
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) *///״̬ת���������󣩣�ʱ��仯��
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   *///�۲����
    float q;  /* process(predict) noise convariance  ����Ԥ�⣩��������*/
    float r;  /* measure noise convariance ������������ */
    float p;  /* estimated error convariance ��������*/
    float gain;
		float B;
		float u;
} kalman1_state;

//1. �ṹ�����Ͷ���
typedef struct 
{
    float LastP;//�ϴι���Э���� ��ʼ��ֵΪ0.02
    float out;//�������˲������ ��ʼ��ֵΪ0
    float Kg;//���������� ��ʼ��ֵΪ0
    float Q;//��������Э���� ��ʼ��ֵΪ0.001
    float R;//�۲�����Э���� ��ʼ��ֵΪ0.543
} KFP;//Kalman Filter parameter


/* 2 Dimension */
typedef struct {
    float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    float r;        /* measure noise convariance */
    float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    float gain[2];  /* 2x1 */
} kalman2_state;


typedef struct {
    float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    float q[2][2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    float r;        /* measure noise convariance */
    float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    float gain[2];  /* 2x1 */
} kalman_dop_state; 


extern void kalman1_init(kalman1_state *state, float init_x, float init_p);
extern float kalman1_filter(kalman1_state *state, float z_measure);
extern void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2]);
extern float kalman2_filter(kalman2_state *state, float z_measure);
extern float kalman_filter_doppler(kalman_dop_state *state, float z_measure);

float kalmanFilter(KFP *kfp,float input);

#endif  /*_KALMAN_FILTER_H*/

