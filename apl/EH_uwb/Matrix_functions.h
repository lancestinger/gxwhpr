// **************************************************************/
// ********************** Matrix_functions.h *******************/
// *
// *                      矩阵运算函数的定义                   */
// ***********************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define DNUM 3		//维度最大数量
 
 
//一，矩阵转置:right:输入矩阵，result:输出矩阵，hight:输入矩阵的高，width:输入矩阵的宽
void Multi_transpose(float * right, float * result, int hight, int width);
 
//二，矩阵求反:right:输入矩阵，result:输出矩阵，hight:输入矩阵的高，width:输入矩阵的宽
void Multi_negation(float * right, float * result, int hight, int width);
 
//三，矩阵相乘:left:输入左矩阵，right:输入右矩阵，result:输出矩阵
//hight1:左矩阵的高，width1:左矩阵的宽，hight2:右矩阵的高，width2:右矩阵的宽
void Multi(float * left, float * right, float * result, int hight1, int width1, int hight2, int width2);
 
//四，矩阵相加:left:输入左矩阵，right:输出右矩阵，result:输出矩阵
//hight:矩阵的高，width：矩阵的宽
void Matrix_addition(float * left, float * right, float * result, int hight, int width);
 
//五，常数乘矩阵:number:常数，right:输入矩阵，result:输出矩阵
//hight:矩阵的高，width:矩阵的宽
void Multi_constant(float number, float * right, float * result, int hight, int width);
 
//六，矩阵求逆（待优化）
//arcs:输入矩阵，n:矩阵的阶数，ans:逆矩阵
void Matrix_inversion(float arcs[DNUM][DNUM], int n, float ans[DNUM][DNUM]);
 
//七，打印矩阵内容
//right:输入矩阵，hight:矩阵高，width:矩阵宽
void Matrix_print(float * right,int hight, int width);

//坐标函数参数转换为线性参数
//(x-a)^2 + (y-b)^2 + (x-c)^2 = R^2 <=> dx+ey+fz = G;
//coordinate : 坐标, left_parameter ； distance 距离 ; 左边参数,right_parameter ：右边参数, dimension ： 维度, num ：矩阵组中函数数量
int arguments_transformer(float * coordinate,float * distance,float * left_parameter,float * right_parameter, int dimension, int num) ;
int least_square(float *left_parameter,float *right_parameter, float *result,const int dimension1, const int num1);

int main11(void);
void  Allarrange(float *str,int k,int len,float *src,int *type);
int factorial(int x);
int arguments_transformer1(float * coordinate,float * distance,float * left_parameter,float * right_parameter, int dimension, int num);
void swap(float *a,float *b);

