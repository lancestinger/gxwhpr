#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "platform/deca/deca_sleep.h"
#include "platform/port/port.h"
#include "math.h"
//#include "usart.h"
#include "loc.h"
#include "log.h"
#include "drv/platform/delay/delay.h"


void Filter(const double x,const double y, const double z, double *out_xyz, u32 time, u8 rest_flag)
{
	static u32 cnt = 0;
	static u32 last_first_time = 0;
	static u32 last_second_time = 0;
	static double last_first_x, last_first_y, last_first_z;
	static double last_second_x, last_second_y, last_second_z;
	double now_x, now_y, now_z;
	double speed_x, speed_y, speed_z;
	double ns0, ns1;
	
	if(rest_flag == 1)
	{
		cnt = 0;
	}
	
	if(cnt == 0)
	{
		cnt++;
		last_first_x = x;
		last_first_y = y;
		last_first_z = z;
		last_first_time = time;
		out_xyz[0] = x;
		out_xyz[1] = y;
		out_xyz[2] = z;
	}
	else if(cnt == 1)
	{
		cnt++;
		last_second_x = x;
		last_second_y = y;
		last_second_z = z;	
		last_second_time = time;
		out_xyz[0] = x;
		out_xyz[1] = y;
		out_xyz[2] = z;	
	}
	else
	{		
		ns0 = get_count_time(last_first_time, last_second_time) / 1000000.0;
		ns1 = get_count_time(last_second_time, time) / 1000000.0;
		speed_x = (last_second_x - last_first_x) / ns0;
		speed_y = (last_second_y - last_first_y) / ns0;
		speed_z = (last_second_z - last_first_z) / ns0;
		now_x = (last_second_x + speed_x * ns1)*0.5 + x*0.5;
		now_y = (last_second_y + speed_y * ns1)*0.5 + y*0.5;
		now_z = (last_second_z + speed_z * ns1)*0.5 + z*0.5;
		
		last_first_x = last_second_x;
		last_first_y = last_second_y;
		last_first_z = last_second_z;
		last_second_x = now_x;
		last_second_y = now_y;
		last_second_z = now_z;
		
		out_xyz[0] = now_x;
		out_xyz[1] = now_y;
		out_xyz[2] = now_z;	
	}
}


void Rotate2(double x1, double y1, double alpha, double* x2, double* y2)
{
	*x2 = x1 * cos(alpha) - y1 * sin(alpha);
	*y2 = x1 * sin(alpha) + y1 * cos(alpha);
}



void Rotate3(double x1, double y1, double z1, 
             double alphaX,double alphaY,double alphaZ, 
             double* x2, double* y2, double* z2)
{
	//Z Axis Rotation
	double x3 = x1 * cos(alphaZ) - y1 * sin(alphaZ);
	double y3 = x1 * sin(alphaZ) + y1 * cos(alphaZ);
	double z3 = z1;

	//Y Axis Rotation
	double z4 = z3 * cos(alphaY) - x3 * sin(alphaY);
	double x4 = z3 * sin(alphaY) + x3 * cos(alphaY);
	double y4 = y3;

	//X Axis Rotation
	*y2 = y4 * cos(alphaX) - z4 * sin(alphaX);
	*z2 = y4 * sin(alphaX) + z4 * cos(alphaX);
	*x2 = x4;
}


#if 0

//顺时针转					 
void positive_rotate3(double x1, double y1, double z1, double x2, double y2, double z2,double* x, double* y, double* z)
{
	double cosz,sinz,cosy,siny,cosx,sinx;	
	double x3,y3,z3,x4,y4,z4,x5,y5,z5;
	double dst,dst2;


	dst = sqrt(pow(x1,2) + pow(y1,2));
	cosz = fabs(x1 / dst);
	sinz = fabs(y1 / dst);
	dst = sqrt(pow(x1,2) + pow(y1,2) + pow(z1,2));
	dst2 = sqrt(pow(x1,2) + pow(y1,2));
	cosy = fabs(dst2 / dst);
	siny = fabs(z1 / dst);

	

	//Z Axis Rotation
	if(y1 > 0)
	{
		sinz = -sinz;
	}
	if(x1 < 0)
	{
		cosz = -cosz;
	}	
	
	x3 = x2 * cosz - y2 * sinz;
	y3 = x2 * sinz + y2 * cosz;
	z3 = z2;	


	//Y Axis Rotation	
	if(z1 > 0)
	{
		siny = -siny;
	}	

	
	if(x1 < 0)
	{
		cosy = -cosy;
	}	
	
	x4 = x3 * cosy - z3 * siny;
	z4 = x3 * siny + z3 * cosy;
	y4 = y3;


	*y = y4;	
	*z = z4;	
	*x = x4;	
}


//逆时针转
void negative_rotate3(double x1, double y1, double z1, double x2, double y2, double z2,double* x, double* y, double* z)
{
	double cosz,sinz,cosy,siny,cosx,sinx;	
	double x3,y3,z3,x4,y4,z4;
	double dst,dst2;


	dst = sqrt(pow(x1,2) + pow(y1,2));
	cosz = fabs(x1 / dst);
	sinz = fabs(y1 / dst);
	dst = sqrt(pow(x1,2) + pow(y1,2) + pow(z1,2));
	dst2 = sqrt(pow(x1,2) + pow(y1,2));
	cosy = fabs(dst2 / dst);
	siny = fabs(z1 / dst);


	//Y Axis Rotation
	if(z1 > 0)
	{
		siny = -siny;
	}	
	
	if(x1 < 0)
	{
		cosy = -cosy;
	}	

	
	x3 = x2 * cosy + z2 * siny;
	z3 = -x2 * siny + z2 * cosy;
	y3 = y2;

	
	//Z Axis Rotation
	if(y1 > 0)
	{
		sinz = -sinz;
	}
	if(x1 < 0)
	{
		cosz = -cosz;
	}	

	
	x4 = x3 * cosz + y3 * sinz;
	y4 = -x3 * sinz + y3 * cosz;
	z4 = z3;	




	*y = y4;	
	*z = z4;	
	*x = x4;	
	
}

#endif


//顺时针转					 
void positive_rotate3(double x1, double y1, double z1, double x2, double y2, double z2,double* x, double* y, double* z)
{
	double cosz,sinz,cosy,siny,cosx,sinx;	
	double x3,y3,z3,x4,y4,z4,x5,y5,z5;
	double dst,dst2;


	dst = sqrt(pow(x1,2) + pow(y1,2));
	cosz = fabs(x1 / dst);
	sinz = fabs(y1 / dst);
	dst = sqrt(pow(x1,2) + pow(y1,2) + pow(z1,2));
	dst2 = sqrt(pow(x1,2) + pow(y1,2));
	cosy = fabs(dst2 / dst);
	siny = fabs(z1 / dst);

	

	//Z Axis Rotation
	if(y1 > 0)
	{
		sinz = -sinz;
	}
	if(x1 < 0)
	{
		cosz = -cosz;
	}	
	
	x3 = x2 * cosz - y2 * sinz;
	y3 = x2 * sinz + y2 * cosz;
	z3 = z2;	


	//Y Axis Rotation	
	if(z1 > 0)
	{
		siny = -siny;
	}	

	
	//if(x1 < 0)
//	{
//		cosy = -cosy;
//	}	
	
	x4 = x3 * cosy - z3 * siny;
	z4 = x3 * siny + z3 * cosy;
	y4 = y3;


	*y = y4;	
	*z = z4;	
	*x = x4;	
}


//逆时针转
void negative_rotate3(double x1, double y1, double z1, double x2, double y2, double z2,double* x, double* y, double* z)
{
	double cosz,sinz,cosy,siny,cosx,sinx;	
	double x3,y3,z3,x4,y4,z4;
	double dst,dst2;


	dst = sqrt(pow(x1,2) + pow(y1,2));
	cosz = fabs(x1 / dst);
	sinz = fabs(y1 / dst);
	dst = sqrt(pow(x1,2) + pow(y1,2) + pow(z1,2));
	dst2 = sqrt(pow(x1,2) + pow(y1,2));
	cosy = fabs(dst2 / dst);
	siny = fabs(z1 / dst);


	//Y Axis Rotation
	if(z1 > 0)
	{
		siny = -siny;
	}	
	
	//if(x1 < 0)
	//{
	//	cosy = -cosy;
	//}	

	
	x3 = x2 * cosy + z2 * siny;
	z3 = -x2 * siny + z2 * cosy;
	y3 = y2;

	
	//Z Axis Rotation
	if(y1 > 0)
	{
		sinz = -sinz;
	}
	if(x1 < 0)
	{
		cosz = -cosz;
	}	

	
	x4 = x3 * cosz + y3 * sinz;
	y4 = -x3 * sinz + y3 * cosz;
	z4 = z3;	




	*y = y4;	
	*z = z4;	
	*x = x4;	
	
}



int CalcPosition_enu(double A_r,double B_x,double B_y,double B_z,double B_r,double tag_h, u8 anchor_on_left, double *point_out)
{
	double Dist_12;
	double x, y, z, x0, y0, x1, y1, x2, y2 , r1, r2;
	double sin, cos;
	double tmp;
	double tmp_x,tmp_y,tmp_z;
	double t2wall_dist; //接受机到隧道墙壁的距离

	

	/* 0.以两基站的连线为横坐标轴，A基站作为原点 */	
	Dist_12 = sqrt(pow(B_x,2) + pow(B_y,2) + pow(B_z,2));
	DBG_WARNING_PRINT("triangle dist: %lf, %lf, %lf m\r\n", Dist_12, A_r, B_r);
	
	x1 = 0;
	y1 = 0;
	r1 = A_r;
	x2 = Dist_12;
	y2 = 0;	
	r2 = B_r;	

	if(A_r > (Dist_12 + 50) || B_r > (Dist_12 + 50))
	{
		DBG_WARNING_PRINT("wrong distance, %lf, %lf m\r\n", A_r, B_r);
		return -1;
	}	

	if((A_r + B_r) <= Dist_12 || (A_r + Dist_12) <= B_r || (B_r + Dist_12) <= A_r) 
	{
		DBG_WARNING_PRINT("Invalid triangle\r\n");
		return -2;
	}

	//1.求在x轴上的映射x0
	y0 = 0;
	x0 = (pow(x2,2) - pow(r2,2) + pow(r1,2) ) / (2 * x2);

	//2.求x,y坐标
	x = x0;
	z = tag_h;
	tmp = pow(r1,2) - pow(x,2) - pow(z,2);
	if(tmp < 0)
	{
		DBG_WARNING_PRINT("Invalid calc value: %lf\r\n", tmp);
		return -4;
	}
			
	
	y = sqrt(tmp);
	t2wall_dist = y;
	if(anchor_on_left == 1)
	{
		y = 0 - y;
	}

	

end:
	//3.计算基于东北天enu的坐标
	negative_rotate3(B_x,B_y,B_z,x, y, z,&tmp_x, &tmp_y, &tmp_z);
	x = tmp_x;
	y = tmp_y;
	z = tmp_z;

	if(isnan(x) || isnan(y) || isnan(z))
	{
		DBG_WARNING_PRINT("Invalid calc value\r\n");
		return -5;
	}

	//实际坐标
	point_out[0] = x;
	point_out[1] = y;
	point_out[2] = z;
	point_out[2] = t2wall_dist;

	return 0;

}




/******************************************************************************
												    二维基站组合筛选
*******************************************************************************/					
u8 Judge_2D (double x1, double y1,double x2, double y2,double x3, double y3)//判断是不是有效三角形，筛选排除无效三角形，以便保证良好的数据 
				 {
				   double Dist_12,Dist_13,Dist_23;					 
					 Dist_12=sqrt(pow((x1-x2),2)+pow((y1-y2),2));//1、2基站距离	
					 Dist_13=sqrt(pow((x1-x3),2)+pow((y1-y3),2));//1、3基站距离
					 Dist_23=sqrt(pow((x2-x3),2)+pow((y2-y3),2));//2、3基站距离
					if((Dist_12+Dist_13)>(Dist_23*Triangle_scale) && (Dist_12+Dist_23)>(Dist_13*Triangle_scale)  &&  (Dist_13+Dist_23)>(Dist_12*Triangle_scale))
						return 1;
					else return 0;
				 
				 }					
/******************************************************************************
												    三维基站组合筛选
*******************************************************************************/					
u8 Judge_3D (double x1, double y1,double z1,double x2, double y2, double z2,double x3, double y3, double z3)//判断是不是有效三角形，筛选排除无效三角形，以便保证良好的数据 
				 {
					 
				   double Dist_12,Dist_13,Dist_23;
					 Dist_12=sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2));//1、2基站距离	
					 Dist_13=sqrt(pow((x1-x3),2)+pow((y1-y3),2)+pow((z1-z3),2));//1、3基站距离
					 Dist_23=sqrt(pow((x2-x3),2)+pow((y2-y3),2)+pow((z2-z3),2));//2、3基站距离
					if((Dist_12+Dist_13)>(Dist_23*Triangle_scale) && (Dist_12+Dist_23)>(Dist_13*Triangle_scale)  &&  (Dist_13+Dist_23)>(Dist_12*Triangle_scale))
						return 1;
					else return 0;
					
					 /*
					double Dist_12,Dist_13,Dist_23;									 
					 Dist_12=sqrt(pow((x1-x2),2)+pow((y1-y2),2));//1、2基站距离
					 Dist_13=sqrt(pow((x1-x3),2)+pow((y1-y3),2));//1、3基站距离
					 Dist_23=sqrt(pow((x2-x3),2)+pow((y2-y3),2));//2、3基站距离
					if((Dist_12+Dist_13)>(Dist_23*Triangle_scale) && (Dist_12+Dist_23)>(Dist_13*Triangle_scale)  &&  (Dist_13+Dist_23)>(Dist_12*Triangle_scale))
						return 1;
					else return 0;
				 */
				 }					
/******************************************************************************
												    二维坐标计算算法
*******************************************************************************/												
				void Get_three_BS_Out_XY(double x1, double y1, double r1,
                                 double x2, double y2, double r2,
                                 double x3, double y3, double r3,double *PP_point_out)
				{
						double A[2][2];
            double B[2][2];
            double C1[2];
            double det = 0;    //determinant
            A[0][0] = 2 * (x1 - x2); 
					  A[0][1] = 2 * (y1 - y2); 
            A[1][0] = 2 * (x1 - x3);
    				A[1][1] = 2 * (y1 - y3); 
             
            det =A[0][0] * A[1][1] - A[1][0] * A[0][1];

            if (det != 0)
             {
                 B[0][0] = A[1][1] / det;
                 B[0][1] = -A[0][1] / det;


                 B[1][0] = -A[1][0] / det;
                 B[1][1] = A[0][0] / det;

                 C1[0] = r2 * r2 - r1 * r1 - x2 * x2 + x1 * x1 - y2 * y2 + y1 * y1;
                 C1[1] = r3 * r3 - r1 * r1 - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1;
                 
                 PP_point_out[0] = B[0][0] * C1[0] + B[0][1] * C1[1] ;
                 PP_point_out[1] = B[1][0] * C1[0] + B[1][1] * C1[1] ;	
                 
             }
             else
             {
                 PP_point_out[0] = 0;
                 PP_point_out[1] = 0;
                 
             }					
				}
/// <summary>
/******************************************************************************
												    二维坐标计算预先处理筛选
*******************************************************************************/
						
				u8 PersonPosition_xy(double A_x,double A_y,double A_r,u16 A_en,
													double B_x,double B_y,double B_r,u16 B_en,
													double C_x,double C_y,double C_r,u16 C_en,
													double D_x,double D_y,double D_r,u16 D_en,
													double E_x,double E_y,double E_r,u16 E_en,
													double F_x,double F_y,double F_r,u16 F_en,
													double G_x,double G_y,double G_r,u16 G_en,
													double H_x,double H_y,double H_r,u16 H_en,double *point_out)
				{					
		  double point_buf[80][2];  //每三个基站循环定位得到的坐标数据缓存，8取3的组合数量为56
      double BS_buf_EN [8][3];  //实际使能且测距成功的基站的数据

      u8 BS_EN_num = 0;   //使能且测距成功的基站数量计数
	
      u8 i = 0, num = 0;
      u8 E = 0, R = 0, T = 0;
	
      if (A_en==1)  //A基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = A_x;
             BS_buf_EN[BS_EN_num][1] = A_y;
             BS_buf_EN[BS_EN_num][2] = A_r;
             BS_EN_num++;
        }
				if (B_en==1)  //B基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = B_x;
             BS_buf_EN[BS_EN_num][1] = B_y;
             BS_buf_EN[BS_EN_num][2] = B_r;
             BS_EN_num++;
        }
				if (C_en==1)  //C基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = C_x;
             BS_buf_EN[BS_EN_num][1] = C_y;
             BS_buf_EN[BS_EN_num][2] = C_r;
             BS_EN_num++;
        }
				if (D_en==1)  //D基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = D_x;
             BS_buf_EN[BS_EN_num][1] = D_y;
             BS_buf_EN[BS_EN_num][2] = D_r;
             BS_EN_num++;
        }			
				if (E_en==1)  //E基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = E_x;
             BS_buf_EN[BS_EN_num][1] = E_y;
             BS_buf_EN[BS_EN_num][2] = E_r;
             BS_EN_num++;
        }
       	if (F_en==1)  //F基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = F_x;
             BS_buf_EN[BS_EN_num][1] = F_y;
             BS_buf_EN[BS_EN_num][2] = F_r;
             BS_EN_num++;
        }
				if (G_en==1)  //G基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = G_x;
             BS_buf_EN[BS_EN_num][1] = G_y;
             BS_buf_EN[BS_EN_num][2] = G_r;
             BS_EN_num++;
        }
				if (H_en==1)  //H基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = H_x;
             BS_buf_EN[BS_EN_num][1] = H_y;
             BS_buf_EN[BS_EN_num][2] = H_r;
             BS_EN_num++;
        }
				
				//if (BS_EN_num < 3)    //少于3个基站，无法定位
				if (BS_EN_num < 2)    //少于3个基站，无法定位
             {
                
                 return 1;
             }

             //for (E = 0; E < (BS_EN_num - 2); E++)  //将所有使能的基站每三个分组进行循环定位
						 for (E = 0; E <= (BS_EN_num - 2); E++)  //将所有使能的基站每三个分组进行循环定位
             {
                 for (R = E + 1; R < (BS_EN_num - 1); R++)
                 {
                     for (T = R + 1; T < BS_EN_num; T++)
                     {
											 u8 flag=0;		
    									 ERROR_FLAG=0;                     //计算耗时，需要归零一下错误标志位，相当于喂狗	 									
											 flag=Judge_2D(BS_buf_EN[E][0], BS_buf_EN[E][1],BS_buf_EN[R][0], BS_buf_EN[R][1],BS_buf_EN[T][0], BS_buf_EN[T][1]);
												if(flag)
												{
														Get_three_BS_Out_XY(BS_buf_EN[E][0], BS_buf_EN[E][1], BS_buf_EN[E][2],
																							BS_buf_EN[R][0], BS_buf_EN[R][1], BS_buf_EN[R][2],
																					    BS_buf_EN[T][0], BS_buf_EN[T][1], BS_buf_EN[T][2],point_buf[num]);
														num++;    //计算得到的坐标数据存入point_buf数组
												}
                     }
                 }
             }
				
			     	 point_out[0] = 0.0;
             point_out[1] = 0.0;
						 
             for (i = 0; i < num; i++)  //将所有计算得到的数据相加存入point_out
             {
                 point_out[0] += point_buf[i][0];
                 point_out[1] += point_buf[i][1];
             }
             if (num != 0)       //取平均值输出坐标数据
             {
                 point_out[0] = point_out[0] / num;
                 point_out[1] = point_out[1] / num;
             }
						 return 0;
					 

				}
/******************************************************************************
												    三维坐标计算预先处理筛选
*******************************************************************************/				
				
				u8 PersonPosition_xyz(double A_x,double A_y,double A_z,double A_r,u16 A_en,
														double B_x,double B_y,double B_z,double B_r,u16 B_en,
														double C_x,double C_y,double C_z,double C_r,u16 C_en,
														double D_x,double D_y,double D_z,double D_r,u16 D_en,
														double E_x,double E_y,double E_z,double E_r,u16 E_en,
														double F_x,double F_y,double F_z,double F_r,u16 F_en,
														double G_x,double G_y,double G_z,double G_r,u16 G_en,
														double H_x,double H_y,double H_z,double H_r,u16 H_en,double *point_out_xyz)
{
		  double point_buf[130][3];  //每四个基站循环定位得到的坐标数据缓存，8取4的组合数量为70,速度大小于89会出问题
      double BS_buf_EN [8][4];  //实际使能且测距成的基站的数据
      u8 BS_EN_num = 0;   //使能且测距成功的基站数量计数
	
      int i = 0, num = 0;
      int E = 0, R = 0, T = 0, K = 0;
	     
      if (A_en==1)  //A基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = A_x;
             BS_buf_EN[BS_EN_num][1] = A_y;
             BS_buf_EN[BS_EN_num][2] = A_z;
             BS_buf_EN[BS_EN_num][3] = A_r;
             BS_EN_num++;
        }
				if (B_en==1)  //B基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = B_x;
             BS_buf_EN[BS_EN_num][1] = B_y;
             BS_buf_EN[BS_EN_num][2] = B_z;
             BS_buf_EN[BS_EN_num][3] = B_r;
             BS_EN_num++;
        }
				if (C_en==1)  //C基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = C_x;
             BS_buf_EN[BS_EN_num][1] = C_y;
             BS_buf_EN[BS_EN_num][2] = C_z;
             BS_buf_EN[BS_EN_num][3] = C_r;
             BS_EN_num++;
        }
				if (D_en==1)  //D基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = D_x;
             BS_buf_EN[BS_EN_num][1] = D_y;
             BS_buf_EN[BS_EN_num][2] = D_z;
             BS_buf_EN[BS_EN_num][3] = D_r;
             BS_EN_num++;
        }			
				if (E_en==1)  //E基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = E_x;
             BS_buf_EN[BS_EN_num][1] = E_y;
             BS_buf_EN[BS_EN_num][2] = E_z;
             BS_buf_EN[BS_EN_num][3] = E_r;
             BS_EN_num++;
        }
       	if (F_en==1)  //F基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = F_x;
             BS_buf_EN[BS_EN_num][1] = F_y;
             BS_buf_EN[BS_EN_num][2] = F_z;
             BS_buf_EN[BS_EN_num][3] = F_r;
             BS_EN_num++;
        }
				if (G_en==1)  //G基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = G_x;
             BS_buf_EN[BS_EN_num][1] = G_y;
             BS_buf_EN[BS_EN_num][2] = G_z;
             BS_buf_EN[BS_EN_num][3] = G_r;
             BS_EN_num++;
        }
				if (H_en==1)  //H基站使能且测距成功就将其数据存入BS_buf_EN数组
        {
             BS_buf_EN[BS_EN_num][0] = H_x;
             BS_buf_EN[BS_EN_num][1] = H_y;
             BS_buf_EN[BS_EN_num][2] = H_z;
             BS_buf_EN[BS_EN_num][3] = H_r;
             BS_EN_num++;
        }
				
				if (BS_EN_num < 4)    //少于4个基站，无法定位
             {
                
                 return 1;
             }

             for (E = 0; E < (BS_EN_num - 3); E++)  //将所有使能的基站每四个分组进行循环定位
             {
                 for (R = E + 1; R < (BS_EN_num - 2); R++)
                 {
                     for (T = R + 1; T < (BS_EN_num - 1); T++)
                     {
                         for (K = T + 1; K < BS_EN_num; K++)
                         {

											     u8 flag=0;		
               					   ERROR_FLAG=0;                     //计算耗时，需要归零一下错误标志位，相当于喂狗													 
                           flag+=Judge_3D(BS_buf_EN[E][0], BS_buf_EN[E][1], BS_buf_EN[E][2], BS_buf_EN[R][0], BS_buf_EN[R][1],BS_buf_EN[R][2], BS_buf_EN[T][0], BS_buf_EN[T][1], BS_buf_EN[T][2]);													 
													 flag+=Judge_3D(BS_buf_EN[E][0], BS_buf_EN[E][1], BS_buf_EN[E][2], BS_buf_EN[R][0], BS_buf_EN[R][1],BS_buf_EN[R][2], BS_buf_EN[K][0], BS_buf_EN[K][1], BS_buf_EN[K][2]);
													 flag+=Judge_3D(BS_buf_EN[E][0], BS_buf_EN[E][1], BS_buf_EN[E][2], BS_buf_EN[T][0], BS_buf_EN[T][1],BS_buf_EN[T][2], BS_buf_EN[K][0], BS_buf_EN[K][1], BS_buf_EN[K][2]);
													 flag+=Judge_3D(BS_buf_EN[R][0], BS_buf_EN[R][1], BS_buf_EN[R][2], BS_buf_EN[T][0], BS_buf_EN[T][1],BS_buf_EN[T][2], BS_buf_EN[K][0], BS_buf_EN[K][1], BS_buf_EN[K][2]);											 
                           if(flag>2)//至少两个模型有效
															{											
																	Get_three_BS_Out_XYZ(BS_buf_EN[E][0], BS_buf_EN[E][1], BS_buf_EN[E][2], BS_buf_EN[E][3],
																											 BS_buf_EN[R][0], BS_buf_EN[R][1], BS_buf_EN[R][2], BS_buf_EN[R][3],
																											 BS_buf_EN[T][0], BS_buf_EN[T][1], BS_buf_EN[T][2], BS_buf_EN[T][3],
																											 BS_buf_EN[K][0], BS_buf_EN[K][1], BS_buf_EN[K][2], BS_buf_EN[K][3],point_buf[num]);
                                  num++;    //计算得到的坐标数据存入point_buf数组
											        }
                         }
                     }
                 }
             }
		
						 point_out_xyz[0] = 0.0;
             point_out_xyz[1] = 0.0;
             point_out_xyz[2] = 0.0;
             for (i = 0; i < num; i++)  //将所有计算得到的数据相加存入point_out
             {
                 point_out_xyz[0] += point_buf[i][0];
                 point_out_xyz[1] += point_buf[i][1];
                 point_out_xyz[2] += point_buf[i][2];
             }
             if (num != 0)       //取平均值输出坐标数据
             {
                 point_out_xyz[0] = point_out_xyz[0] / num;
                 point_out_xyz[1] = point_out_xyz[1] / num;
                 point_out_xyz[2] = point_out_xyz[2] / num;
             }
						 return 0;
						
}

 /******************************************************************************
												    三维坐标计算算法
*******************************************************************************/	
void  Get_three_BS_Out_XYZ(double x1, double y1, double z1, double r1,
                           double x2, double y2, double z2, double r2,
                           double x3, double y3, double z3, double r3,
                           double x4, double y4, double z4, double r4,double *Point_xyz)//三维坐标求解
         {
             double A[3][3];
             double B[3][3];
             double C1[3];
             double det = 0;    //矩阵A的行列式
					 //以3*3的二维数组A存储矩阵A的数据
             A[0][0] = 2 * (x1 - x2); A[0][1] = 2 * (y1 - y2); A[0][2] = 2 * (z1 - z2);
             A[1][0] = 2 * (x1 - x3); A[1][1] = 2 * (y1 - y3); A[1][2] = 2 * (z1 - z3);
             A[2][0] = 2 * (x1 - x4); A[2][1] = 2 * (y1 - y4); A[2][2] = 2 * (z1 - z4);  

           //求矩阵A的行列式的值
             det = A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*A[2][1]
					        -A[2][0]*A[1][1]*A[0][2]-A[1][0]*A[0][1]*A[2][2]-A[0][0]*A[2][1]*A[1][2];

             if (det != 0)  //只有在矩阵A的行列式不为0时，矩阵A才存在逆矩阵，3*3的二维数组B即为A的逆矩阵
             {
                 B[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
                 B[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) / det;
                 B[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;

                 B[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) / det;
                 B[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
                 B[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) / det;

                 B[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
                 B[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) / det;
                 B[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;
                 
                 //数组C为公式A*X=C中的矩阵C
                 C1[0] = r2 * r2 - r1 * r1 - x2 * x2 + x1 * x1 - y2 * y2 + y1 * y1 - z2 * z2 + z1 * z1;
                 C1[1] = r3 * r3 - r1 * r1 - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1 - z3 * z3 + z1 * z1;
                 C1[2] = r4 * r4 - r1 * r1 - x4 * x4 + x1 * x1 - y4 * y4 + y1 * y1 - z4 * z4 + z1 * z1;
							 
							 //将矩阵A的逆矩阵左乘矩阵C得到标签x,y,z的值
                 Point_xyz[0] = B[0][0] * C1[0] + B[0][1] * C1[1] + B[0][2] * C1[2];
                 Point_xyz[1] = B[1][0] * C1[0] + B[1][1] * C1[1] + B[1][2] * C1[2];
                 Point_xyz[2] = B[2][0] * C1[0] + B[2][1] * C1[1] + B[2][2] * C1[2];
                 
             }
             else
             {
                 Point_xyz[0] = 0;
                 Point_xyz[1] = 0;
                 Point_xyz[2] = 0;
                 
             }
         }
				
