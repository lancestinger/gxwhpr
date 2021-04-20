// **************************************************************/
// ********************** Matrix_functions.cpp *****************/
// *
// *                      �������㺯����ʵ��                   */
// ***********************************************************/
#include "Matrix_functions.h"
#include "EH_uwb.h"

 
//һ������ת���㷨��number:������right:�˾���result:�������hight:����ߣ�width:�����
void Multi_transpose(float * right, float * result, int hight, int width) {
 
	for (int i = 0; i < hight; i++) {//i��ʾ��i��
		for (int j = 0; j < width; j++) {//j��ʾ��j��
			result[j*hight + i] = right[i*width + j];//������ result[i][j] = result[i*f2+j];
		}
	}
}
 
//�����������㷨��number:������right:�˾���result:�������hight:����ߣ�width:�����
void Multi_negation(float * right, float * result, int hight, int width) {
 
	for (int i = 0; i < hight; i++) {//i��ʾ��i��
		for (int j = 0; j < width; j++) {//j��ʾ��j��
			result[i*width + j] = -right[i*width + j];//������ result[i][j] = result[i*f2+j];
		}
	}
}
 
//������������㷨������ĸ�������������˵ľ��������������
void Multi(float * left, float * right, float * result, int hight1, int width1, int hight2, int width2) {
	for (int i = 0; i < hight1; i++) {//i��ʾ��i��
		for (int j = 0; j < width2; j++) {//j��ʾ��j��
			result[i*width2 + j] = 0;//������ result[i][j] = result[i*f2+j];
			for (int p = 0; p < width1; p++) {
				result[i*width2 + j] += left[i*width1 + p] * right[p*width2 + j];
			}
		}
	}
}
 
 
//�ģ�������ӣ�left:�������,right:�������result:�������hight:����ߣ�width:�����
void Matrix_addition(float * left, float * right, float * result, int hight, int width)
{
 
	for (int i = 0; i < hight; i++) {//i��ʾ��i��
		for (int j = 0; j < width; j++) {//j��ʾ��j��
			result[i*width + j] = left[i*width + j] + right[i*width + j];//������ result[i][j] = result[i*f2+j];
		}
	}
}
 
//�壬������������㷨��number:������right:�˾���result:�������hight:����ߣ�width:�����
void Multi_constant(float number, float * right, float * result, int hight, int width) {
 
	for (int i = 0; i < hight; i++) {//i��ʾ��i��
		for (int j = 0; j < width; j++) {//j��ʾ��j��
			result[i*width + j] = number*right[i*width + j];//������ result[i][j] = result[i*f2+j];
		}
	}
}
 
 
//�������������㷨
//������ʽ
int getA(float arcs[DNUM][DNUM], int n)//����һ��չ������|A|
{
	if (n == 1)
	{
		return arcs[0][0];
	}
	int ans = 0;
	float temp[DNUM][DNUM];
	int i, j, k;
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n - 1; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];
 
			}
		}
		int t = getA(temp, n - 1);
		if (i % 2 == 0)
		{
			ans += arcs[0][i] * t;
		}
		else
		{
			ans -= arcs[0][i] * t;
		}
	}
	return ans;
}
//���������ʽ
void getAStart(float arcs[DNUM][DNUM], int n, float ans[DNUM][DNUM])//����ÿһ��ÿһ�е�ÿ��Ԫ������Ӧ������ʽ�����A*
{
	if (n == 1)
	{
		ans[0][0] = 1;
		return;
	}
	int i, j, k, t;
	float temp[DNUM][DNUM];
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				for (t = 0; t<n - 1; t++)
				{
					temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
				}
			}
 
 
			ans[i][j] = getA(temp, n - 1);
			if ((i + j) % 2 == 1)
			{
				ans[i][j] = -ans[i][j];
			}
		}
	}
}
//�������棬����ʽ���������ʽ
void Matrix_inversion(float arcs[DNUM][DNUM], int n, float ans[DNUM][DNUM]) {

	int a = getA(arcs, n);
	if (a == 0)
	{
		printf("Matrix_inversion can not transform!\r\n");
		Least_fail = 1;
	}
	else
	{
		float astar[DNUM][DNUM];
		getAStart(arcs, n, astar);
		for (int i = 0; i<n; i++)
		{
			for (int j = 0; j<n; j++)
			{
				ans[i][j] = (double)astar[i][j] / a;
			}
		}
	}
 
}
 
//�ߣ���ӡ��������
//right:�������hight:����ߣ�width:�����
void Matrix_print(float * right, int hight, int width) {
	printf("result: ");
	for (int i = 0; i <= hight; i++) {//i��ʾ��i��
		for (int j = 0; j < width; j++) {//j��ʾ��j��
			printf("%f  ",right[i*width + j]);//������ result[i][j] = result[i*f2+j];
		}
	}
	printf("\r\n");
}


//����ϵ��������ת��Ϊ���Բ���
//(x-a)^2 + (y-b)^2 + (x-c)^2 = R^2 <=> dx+ey+fz = G;
//coordinate : ����, left_parameter �� distance ���� ; ��߲���,right_parameter ���ұ߲���, dimension �� ά��, num ���������к�������
int arguments_transformer(float * coordinate,float * distance,float * left_parameter,float * right_parameter, int dimension, int num) 
{
	int i,j;


	for(i = 0; i < num-1; i++){
		for(j = 0; j < dimension; j++) {
			left_parameter[i*dimension+j] = 2*(coordinate[i*dimension+j] - coordinate[(i+1)*dimension+j]);
		}
	}
	for(i = 0; i < num-1; i++) {
		right_parameter[i] = pow(distance[i+1],2) - pow(distance[i],2);
		for(j = 0; j < dimension; j++){
			right_parameter[i] += pow(coordinate[i*dimension+j],2) -  pow(coordinate[(i+1)*dimension+j],2);
		}
	}
	return 0;
}
int arguments_transformer1(float * coordinate,float * distance,float * left_parameter,float * right_parameter, int dimension, int num) 
{
	int i,j;


	for(i = 0; i < num-1; i++){
		for(j = 0; j < dimension; j++) {
			left_parameter[i*dimension+j] = 2*(coordinate[j] - coordinate[(i+1)*dimension+j]);
		}
	}
	for(i = 0; i < num-1; i++) {
		right_parameter[i] = pow(distance[i+1],2) - pow(distance[0],2);
		for(j = 0; j < dimension; j++){
			right_parameter[i] += pow(coordinate[j],2) -  pow(coordinate[(i+1)*dimension+j],2);
		}
	}
	return 0;
}

void swap(float *a,float *b)
{
	float temp;
	temp = *a;
	*a = *b;
	*b = temp;	
}

void  Allarrange(float *str,int k,int len,float *src,int *type)
{
	int i;
	if(k==len)
	{
		//static int s_i=1;
		//printf("��%d������Ϊ:\t\n",s_i++);
		for(int j =0;j< len;j++)
			src[(*type)*len+j] = str[j];
		*type = (*type)+1;
		return ;
	}
	else
	{
		for(i=k;i<len;i++)
		{
			swap(str+i,str+k);
			Allarrange(str,k+1,len,src,type);
			swap(str+i,str+k);
		}
	}
}
int factorial(int x)
{
	
	if (x == 0 )
	{
		return 1;
	}
	else 				
		return x*factorial(x-1);
}

int main11()
{
	float str[3] = {1,2,3};
	float src[18] = {0};
	int i = 0;
	Allarrange(str,0,3,src,&i);
	for(int j =0;j< 18;j++)
			printf("%0.1f\t",src[j]);
	printf("i = %d\r\n",i);
	return 0;
}



