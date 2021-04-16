// **************************************************************/
// ********************** Matrix_functions.h *******************/
// *
// *                      �������㺯���Ķ���                   */
// ***********************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define DNUM 3		//ά���������
 
 
//һ������ת��:right:�������result:�������hight:�������ĸߣ�width:�������Ŀ�
void Multi_transpose(float * right, float * result, int hight, int width);
 
//����������:right:�������result:�������hight:�������ĸߣ�width:�������Ŀ�
void Multi_negation(float * right, float * result, int hight, int width);
 
//�����������:left:���������right:�����Ҿ���result:�������
//hight1:�����ĸߣ�width1:�����Ŀ�hight2:�Ҿ���ĸߣ�width2:�Ҿ���Ŀ�
void Multi(float * left, float * right, float * result, int hight1, int width1, int hight2, int width2);
 
//�ģ��������:left:���������right:����Ҿ���result:�������
//hight:����ĸߣ�width������Ŀ�
void Matrix_addition(float * left, float * right, float * result, int hight, int width);
 
//�壬�����˾���:number:������right:�������result:�������
//hight:����ĸߣ�width:����Ŀ�
void Multi_constant(float number, float * right, float * result, int hight, int width);
 
//�����������棨���Ż���
//arcs:�������n:����Ľ�����ans:�����
void Matrix_inversion(float arcs[DNUM][DNUM], int n, float ans[DNUM][DNUM]);
 
//�ߣ���ӡ��������
//right:�������hight:����ߣ�width:�����
void Matrix_print(float * right,int hight, int width);

//���꺯������ת��Ϊ���Բ���
//(x-a)^2 + (y-b)^2 + (x-c)^2 = R^2 <=> dx+ey+fz = G;
//coordinate : ����, left_parameter �� distance ���� ; ��߲���,right_parameter ���ұ߲���, dimension �� ά��, num ���������к�������
int arguments_transformer(float * coordinate,float * distance,float * left_parameter,float * right_parameter, int dimension, int num) ;
int least_square(float *left_parameter,float *right_parameter, float *result,const int dimension1, const int num1);

int main11(void);
void  Allarrange(float *str,int k,int len,float *src,int *type);
int factorial(int x);
int arguments_transformer1(float * coordinate,float * distance,float * left_parameter,float * right_parameter, int dimension, int num);
void swap(float *a,float *b);

