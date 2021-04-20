// **************************************************************/
// ********************** Matrix_functions.cpp *****************/
// *
// *                      矩阵运算函数的实现                   */
// ***********************************************************/
#include "Matrix_functions.h"
#include "EH_uwb.h"

 
//一，矩阵转置算法，number:常数，right:乘矩阵，result:结果矩阵，hight:矩阵高，width:矩阵宽
void Multi_transpose(float * right, float * result, int hight, int width) {
 
	for (int i = 0; i < hight; i++) {//i表示第i行
		for (int j = 0; j < width; j++) {//j表示第j列
			result[j*hight + i] = right[i*width + j];//在这里 result[i][j] = result[i*f2+j];
		}
	}
}
 
//二，矩阵求反算法，number:常数，right:乘矩阵，result:结果矩阵，hight:矩阵高，width:矩阵宽
void Multi_negation(float * right, float * result, int hight, int width) {
 
	for (int i = 0; i < hight; i++) {//i表示第i行
		for (int j = 0; j < width; j++) {//j表示第j列
			result[i*width + j] = -right[i*width + j];//在这里 result[i][j] = result[i*f2+j];
		}
	}
}
 
//三，矩阵相乘算法，最后四个参数是两个相乘的矩阵的行数和列数
void Multi(float * left, float * right, float * result, int hight1, int width1, int hight2, int width2) {
	for (int i = 0; i < hight1; i++) {//i表示第i行
		for (int j = 0; j < width2; j++) {//j表示第j列
			result[i*width2 + j] = 0;//在这里 result[i][j] = result[i*f2+j];
			for (int p = 0; p < width1; p++) {
				result[i*width2 + j] += left[i*width1 + p] * right[p*width2 + j];
			}
		}
	}
}
 
 
//四，矩阵相加，left:输入矩阵,right:输入矩阵，result:结果矩阵，hight:矩阵高，width:矩阵宽
void Matrix_addition(float * left, float * right, float * result, int hight, int width)
{
 
	for (int i = 0; i < hight; i++) {//i表示第i行
		for (int j = 0; j < width; j++) {//j表示第j列
			result[i*width + j] = left[i*width + j] + right[i*width + j];//在这里 result[i][j] = result[i*f2+j];
		}
	}
}
 
//五，常数矩阵相乘算法，number:常数，right:乘矩阵，result:结果矩阵，hight:矩阵高，width:矩阵宽
void Multi_constant(float number, float * right, float * result, int hight, int width) {
 
	for (int i = 0; i < hight; i++) {//i表示第i行
		for (int j = 0; j < width; j++) {//j表示第j列
			result[i*width + j] = number*right[i*width + j];//在这里 result[i][j] = result[i*f2+j];
		}
	}
}
 
 
//六，矩阵求逆算法
//求行列式
int getA(float arcs[DNUM][DNUM], int n)//按第一行展开计算|A|
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
//求代数余子式
void getAStart(float arcs[DNUM][DNUM], int n, float ans[DNUM][DNUM])//计算每一行每一列的每个元素所对应的余子式，组成A*
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
//矩阵求逆，行列式与代数余子式
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
 
//七，打印矩阵内容
//right:输入矩阵，hight:矩阵高，width:矩阵宽
void Matrix_print(float * right, int hight, int width) {
	printf("result: ");
	for (int i = 0; i <= hight; i++) {//i表示第i行
		for (int j = 0; j < width; j++) {//j表示第j列
			printf("%f  ",right[i*width + j]);//在这里 result[i][j] = result[i*f2+j];
		}
	}
	printf("\r\n");
}


//坐标系函数参数转换为线性参数
//(x-a)^2 + (y-b)^2 + (x-c)^2 = R^2 <=> dx+ey+fz = G;
//coordinate : 坐标, left_parameter ； distance 距离 ; 左边参数,right_parameter ：右边参数, dimension ： 维度, num ：矩阵组中函数数量
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
		//printf("第%d种排列为:\t\n",s_i++);
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



