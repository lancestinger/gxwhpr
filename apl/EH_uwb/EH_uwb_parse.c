#include "EH_uwb_parse.h"
#include <math.h>
#include <stdio.h>



#define PI   (3.1415926535898)  //圆周率
#define x_PI (180.0/PI)  //弧度
#define f  (1.0/298.257223563) //wsg-84 扁率
#define earth_a  (6378137.0)  //长半轴,单位m
#define earth_b  ( earth_a * (1-f)) //短半轴
#define ec  (( earth_a * earth_a - earth_b * earth_b )/( earth_a * earth_a )) //第一偏心率
#define ecc  (( earth_a * earth_a - earth_b * earth_b )/( earth_b * earth_b )) //第二偏心率


//高斯投影正算
Vector2 *BLToxy(PointBL BL,Vector2 *xy,double L0)
{
    double B = BL.B/x_PI;
    double L = BL.L/x_PI;
    //辅助计算公式
    double W = sqrt(1 - ec * sin(B) * sin(B));
    double n2 = ecc * cos(B) * cos(B);
    double t = tan(B);
    //曲率半径
    double N = earth_a / W;
    double M = earth_a * (1 - ec) / pow(W, 3);
    double M0 = earth_a * (1 - ec);
    //子午线弧长计算公式
    double Ac = 1 + 3 / 4.0000000000 * ec + 45 / 64.0000000000 * pow(ec, 2) + 175 / 256.0000000000 * pow(ec, 3) + 11025 / 16384.0000000000 * pow(ec, 4) + 43659 / 65536.0000000000 * pow(ec, 5);
    double Bc = 3 / 4.0000000000 * ec + 15 / 16.0000000000 * pow(ec, 2) + 525 / 512.0000000000 * pow(ec, 3) + 2205 / 2048.0000000000 * pow(ec, 4) + 72765 / 65536.0000000000 * pow(ec, 5);
    double Cc = 15 / 64.0000000000 * pow(ec, 2) + 105 / 256.0000000000 * pow(ec, 3) + 2205 / 4096.0000000000* pow(ec, 4) + 10395 / 16384.0000000000 * pow(ec, 5);
    double Dc = 35 / 512.0000000000 * pow(ec, 3) + 315 / 2048.0000000000 * pow(ec, 4) + 31185 / 131072.0000000000 * pow(ec, 5);
    double Ec = 315 / 16384.0000000000 * pow(ec, 4) + 3465 / 65536.0000000000 * pow(ec, 5);
    double Fc = 693 / 131072.0000000000 * pow(ec, 5);
    double Alpha = Ac * M0;
    double Beta = -1 / 2.0000000000 * Bc * M0;
    double Gamma = 1 / 4.0000000000 * Cc * M0;
    double Delte = -1 / 6.0000000000 * Dc * M0;
    double Epsilon = 1 / 8.0000000000 * Ec * M0;
    double Zeta = -1 / 10.0000000000 * Fc * M0;
    //子午弧长
    double X = Alpha * B + Beta * sin(2 * B) + Gamma * sin(4 * B) + Delte * sin(6 * B) + Epsilon * sin(8 * B) + Zeta * sin(10 * B);
    //经差
    double l = L - L0 / 180 * PI;
    //辅助量
    double a0 = X;
    double a1 = N * cos(B);
    double a2 = 1 / 2.0000000000 * N * pow(cos(B), 2) * t;
    double a3 = 1 / 6.0000000000 * N * pow(cos(B), 3) * (1 - pow(t, 2) + n2);
    double a4 = 1 / 24.0000000000 * N * pow(cos(B), 4) * (5 - pow(t, 2) + 9 * n2 + 4 * pow(n2, 2));
    double a5 = 1 / 120.0000000000 * N * pow(cos(B), 5) * (5 - 18 * pow(t, 2) + pow(t, 4) + 14 * n2 - 58 * n2 * pow(t, 2));
    double a6 = 1 / 720.0000000000 * N * pow(cos(B), 6) * (61 - 58 * pow(t, 2) + pow(t, 4) + 270 * n2 - 330 * n2 * pow(t, 2)) * t;
    xy->x = a0 + a2 * pow(l, 2) + a4 * pow(l, 4) + a6 * pow(l, 6);
    xy->y = a1 * l + a3 * pow(l, 3) + a5 * pow(l, 5);
    return xy;
}
/// <summary>
/// 高斯投影反算
/// </summary>
/// <param name="xy"><"高斯平面直角坐标系坐标">
/// <param name="L0"><"中央子午线经度">
/// <returns></returns>
PointBL *xyToBL(Vector2 xy,PointBL *BL,double L0)
{
    double x = xy.x;
    double y = xy.y;
    double Ac = 1 + 3 / 4.0000000000 * ec + 45 / 64.0000000000 * pow(ec, 2) + 175 / 256.0000000000 * pow(ec, 3) + 11025 / 16384.0000000000 * pow(ec, 4) + 43659 / 65536.0000000000 * pow(ec, 5);
    double Bc = 3 / 4.0000000000 * ec + 15 / 16.0000000000 * pow(ec, 2) + 525 / 512.0000000000* pow(ec, 3) + 2205 / 2048.0000000000 * pow(ec, 4) + 72765 / 65536.0000000000 * pow(ec, 5);
    double Cc = 15 / 64.0000000000 * pow(ec, 2) + 105 / 256.0000000000 * pow(ec, 3) + 2205 / 4096.0000000000 * pow(ec, 4) + 10395 / 16384.0000000000 * pow(ec, 5);
    double Dc = 35 / 512.0000000000 * pow(ec, 3) + 315 / 2048.0000000000 * pow(ec, 4) + 31185 / 131072.0000000000 * pow(ec, 5);
    double Ec = 315 / 16384.0000000000 * pow(ec, 4) + 3465 / 65536.0000000000 * pow(ec, 5);
    double Fc = 693 / 131072.0000000000 * pow(ec, 5);
    double M0 = earth_a * (1 - ec);
    double Alpha = Ac * M0;
    double Beta = -1 / 2.0000000000 * Bc * M0;
    double Gamma = 1 / 4.0000000000 * Cc * M0;
    double Delte = -1 / 6.0000000000 * Dc * M0;
    double Epsilon = 1 / 8.0000000000 * Ec * M0;
    double Zeta = -1 / 10.0000000000 * Fc * M0;
    double X = x;
    double B0 = X / Alpha;
    double Bf = 0;
    while (1)
    {
        double dert = Beta * sin(2 * B0) + Gamma * sin(4 * B0) + Delte * sin(6 * B0) + Epsilon * sin(8 * B0) + Zeta * sin(10 * B0);
        Bf = (X - dert) / Alpha;
        if (fabs(Bf - B0) < 1e-13)
            break;
        else
            B0 = Bf;
    }
    //辅助公式
    double Wf = sqrt(1 - ec * pow(sin(Bf), 2));
    double n2 = ecc * pow(cos(Bf), 2);
    double tf = tan(Bf);
    double Nf = earth_a / Wf;
    double Mf = earth_a * (1 - ec) / pow(Wf, 3);
    double b0 = Bf;
    double b1 = 1 / (Nf * cos(Bf));
    double b2 = -tf / (2 * Nf * Mf);
    double b3 = -(1 + 2 * tf * tf + n2) * b1 / (6 * Nf * Nf);
    double b4 = -(5 + 3 * tf * tf + n2 - 9 * n2 * tf * tf) * b2 / (12 * Nf * Nf);
    double b5 = -(5 + 28 * tf * tf + 24 * pow(tf, 4) + 6 * n2 + 8 * n2 * tf * tf) * b1 / (120 * pow(Nf, 4));
    double b6 = (61 + 90 * tf * tf + 45 * pow(tf, 4)) * b2 / (360 * pow(Nf, 4));
    BL->B = (b0 + b2 * pow(y, 2) + b4 * pow(y, 4) + b6 * pow(y, 6))*x_PI;
    BL->L = (b1 * pow(y, 1) + b3 * pow(y, 3) + b5 * pow(y, 5) + L0 * PI / 180)*x_PI;
    return BL;
}
//大地坐标到空间坐标
void BLHToXYZ(Vector3 *des,const PointBLH src)
{
		printf("BLHToXYZ \r\n");
		double B = src.B/x_PI;
    double L = src.L/x_PI;
    double H = src.H;
    //辅助计算公式
    double W = sqrt(1 - ec* sin(B)*sin(B));
		
    double N = earth_a / W;
    des->x = (N + H) * cos(B) * cos(L);
    des->y = (N + H) * cos(B) * sin(L);
    des->z = (N * (1 - ec) + H) * sin(B);
    
	return ;
}
//空间坐标到大地坐标
void  XYZToBLH(PointBLH *des , const Vector3 src)
{
		printf("XYZToBLH \r\n");
    double X = src.x;
    double Y = src.y;
    double Z = src.z;
    des->L = atan(Y / X);
    if (X < 0)
        des->L += PI;
    double r = sqrt(X * X + Y * Y);
    double B1 = atan(Z / r);
    double B2;
    while (1) //迭代
    {
        double W1 = sqrt(1 - ec  * (sin(B1) * sin(B1)));
        double N1 = earth_a / W1;
        B2 = atan((Z + N1 * ec * sin(B1)) / r);
        if (fabs(B2 - B1) <= 1e-13)
            break;
        B1 = B2;
    }
    des->B = B2;
    double W = sqrt(1 - ec  * (sin(B2) * sin(B2)));
    double N = earth_a / W;
    des->H = r / cos(B2) - N;
    return ;
}
//两点距离
double Distance(Vector2 pnt1,Vector2 pnt2)
{
		double  a1 = sqrt(pow((pnt1.x-pnt2.x),2)+pow((pnt1.y-pnt2.y),2));
		return a1;
}
//第三点位置
int CalculateTriangleThirdPoint(const Vector2 vA, const Vector2 vB, Vector2 *vC, double  a, double  b, double  c )
{
	double  AB = acos((a*a+b*b-c*c)/(2.0*a*b));
	double  AC = acos((a*a+c*c-b*b)/(2.0*a*c));
	double  BC = acos((b*b+c*c-a*a)/(2.0*b*c));
	double  ABx = atan((vB.y-vA.y)/(vB.x-vA.x));
	double 	ACx=0.0;

	if( (PI-1e-13) < AB && AB < (PI+1e-13)){
			vC->x = vA.x + b*cos(ABx);
			vC->y = vA.y + b*sin(ABx);
			return 0;
	}//三点在一线上
	if((PI+1e-13)<(AB+AC+BC)||(AB+AC+BC)<(PI-1e-13)){
		printf("DELTA ERROR %f \r\n",(AB+AC+BC));
		return -1;
	}//三点没有构不成三角形

	if(1){ //两个结果取一个
	ACx = ABx + BC;
	vC->x = vA.x + b*cos(ACx);
	vC->y = vA.y + b*sin(ACx);
	}else{
	ACx = ABx - BC;
	vC->x = vA.x + b*cos(ACx);
	vC->y = vA.y + b*sin(ACx);
	}
	return 0;
}
//点到直线的垂足点
Vector2* getFootPoint(Vector2 point, Vector2 pnt1, Vector2 pnt2, Vector2 *fpoint)
{
    double A=pnt2.y-pnt1.y;     //y2-y1
    double B=pnt1.x-pnt2.x;     //x1-x2;
    double C=pnt2.x*pnt1.y-pnt1.x*pnt2.y;     //x2*y1-x1*y2
    if (A * A + B * B < 1e-13) {
		fpoint->x = pnt1.x;
        fpoint->y = pnt1.y;
    }//pnt1与pnt2重叠
    else if (fabs(A * point.x + B * point.y + C) < 1e-13) {
		fpoint->x = point.x;
        fpoint->y = point.y;
    }//point在直线上(pnt1_pnt2)
    else {
        double x = (B * B * point.x - A * B * point.y - A * C) / (A * A + B * B);
        double y = (-A * B * point.x + A * A * point.y - B * C) / (A * A + B * B);
        fpoint->x = x;
        fpoint->y = y;
    }
	return fpoint;
}
//角度弧度转换
void nmea_BL2pos(const PointBL *info, POS *pos)
{
    pos->lat = (info->B)/x_PI;//纬度
    pos->lon = (info->L)/x_PI; //经度
}
//中央经线经度
double Center_longitude(double longitude)
{
			return 3*round(longitude/3.0);
}

/*
static uin16_t distances[100] = {0};

void Uwb_Dist_VARC(uint16_t dist[], uint8_t statn_id)
{	
	uint8_t i=0,j=0;
	uint16_t avr =0,Varc =0;
	static uint16_t sum =0, num =0;
	
	num++;
	sum += dist; 
	if (num == 100)
	{
		avr = sum/num;
		sum = 0;
		for (j=0;j<num;j++)
		{
			sum += (distances[j]-avr)*(distances[j]-avr);
			Varc = sum/(num-1);
		}
		memcpy(distances,0x0,sizeof distances);
		sum = 0;
		num = 0;
	}
	else if(num > 100)
	{
		printf("Variance error!!");
	}
	
	printf("Variance = %.2f",Varc);


}
*/

uint8_t Drop_Sign = 0;
uint16_t Drop_Num = 0;

//串口解析
int uwb_uart_parse(UWBagree * uwbagree,const char *data)
{
		int i = 0;
		static uint8_t Confid_Sign = 0;
		
		uwbagree->head = (uint32_t)((data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0));
		if(uwbagree->head != 0x02119102){
			//printf("uwbagree->head is error 0x%x \r\n",uwbagree->head);
			return -1;
		}
		uwbagree->cmd = (uint16_t)((data[5] << 8) + (data[4] << 0));
		if(uwbagree->cmd != 0x3001){
			//printf("uwbagree->cmd is error 0x%x \r\n",uwbagree->cmd);
			return -1;
		}
		uwbagree->reserved = (uint16_t)((data[7] << 8) + (data[6] << 0));
		uwbagree->length = (uint32_t)((data[11] << 24) + (data[10] << 16) + (data[9] << 8) + (data[8] << 0));
		uwbagree->data.id = (uint32_t)((data[15] << 24) + (data[14] << 16) + (data[13] << 8) + (data[12] << 0)) ;
		uwbagree->data.top = (uint32_t)((data[18] << 16) + (data[17] << 8) + (data[16] << 0));
		uwbagree->data.num = (uint8_t)(data[19]<<0); 
		for(i=0;i<uwbagree->data.num;i++)
		{
			uwbagree->data.info[i].addr = (uint16_t)((data[21+i*6] << 8) + (data[20+i*6] << 0));
			uwbagree->data.info[i].dist = (uint16_t)((data[23+i*6] << 8) + (data[22+i*6] << 0));
			uwbagree->data.info[i].deg = (uint8_t)(data[24+i*6]);
			uwbagree->data.info[i].RSSI = (uint8_t)(data[25+i*6]);
			//UWB singnal Confidence check
			//if(uwbagree->data.info[i].deg <= 5){
			//	Confid_Sign = 1;
			//}
		}
		printf("Station NUM = %d\r\n",uwbagree->data.num);
		//Get Number of UWB singnal should be Dropped
		if(Confid_Sign == 0){
			Drop_Sign = 0;
			Drop_Num = 0;
		}else{
			Drop_Sign = 1;
			Drop_Num++;
			Confid_Sign = 0;
		}
		uwbagree->proof = (uint8_t)data[26+i*6];
		if(Drop_Sign != 0)
		{
			printf("UWB confidence too low!!!\r\n");
			Drop_Sign = 0;
			return -1;
		}
		return 0;
}



