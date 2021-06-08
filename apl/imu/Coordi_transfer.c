#include "json/Cjson_run.h"
#include "Coordi_transfer.h"
#include <math.h>
#include "ins_baseparams.h"
#include "fml/gnss/nmea.h"
#include "Imu.h"

//--------------------------------------------------------------------

static Nav_std_sum Lac_E;
static Nav_std_sum Lac_N;
static Nav_std_sum Lac_H;
static Nav_std_sum Vel_E;
static Nav_std_sum Vel_N;
static Nav_std_sum Vel_H;

static Pre_locat Pre_location;
static Pre_veloc Pre_vel;

static WGS LLA_loc;
static WGS ORIGN_pot;
static ECEF ECEF_loc;
static ENU ENU_loc;

#define M_PI 3.1415926
	
const double dSemiMajorAxis = 6378137.0;//椭球长半轴
const double e2 = 0.0066943799013;//第一偏心率的平方

int FIRST_ANGLE_GET = 0;
int VELOC_VALID = 0;

//------------------分割线--------------------------------------------

//GPS采集到的经纬度数据无法直接使用，需要先转换
double transfer(double val)
{
	double deg = ((int)(val / 100));
	val = deg + (val - deg * 100) / 60;
	return val;
}

//WGS84 ---> ECEF
//pcg为WGS-84坐标系结构体指针，pcc为ECEF坐标系结构体指针
void WGSToECEF(PWGS pcg, PECEF pcc)
{
	double B = pcg->latitude; //纬度
	double L = pcg->longitude; //经度
	double H = pcg->height;  //高度
	double N; //卯酉圈曲率半径
	double B_, L_;
	
	//经纬度转换成弧度 通过GPS采集的经纬度需要先经过转换
	
	//测试正常经纬度数据使用
	B_ = B*M_PI/180;
	L_ = L*M_PI/180;
	
	N = dSemiMajorAxis / sqrt(1.0 - e2 * sin(B_) * sin(B_));
	pcc->x = (N + H) * cos(B_) * cos(L_);
	pcc->y = (N + H) * cos(B_) * sin(L_);
	pcc->z = (N * (1.0 - e2) + H) * sin(B_);
}

//ECEF ---> WGS84
//pcg为WGS-84坐标系结构体指针，pcc为ECEF坐标系结构体指针
void ECEFToWGS(PWGS pcg, PECEF pcc)
{
	double B0, R, N;
	double B_, L_;
	double X = pcc->x;
	double Y = pcc->y;
	double Z = pcc->z;
	
	R = sqrt(X * X + Y * Y);
	B0 = atan2(Z, R);
	
	while (1)
	{
		N = dSemiMajorAxis / sqrt(1.0 - e2 * sin(B0) * sin(B0));
		B_ = atan2(Z + N * e2 * sin(B0), R);
		if (fabs(B_ - B0) < 1.0e-10)
		break;
		B0 = B_;
	}
	
	L_ = atan2(Y, X);
	pcg->height = R / cos(B_) - N;
	
	//弧度转换成经纬度
	pcg->latitude = B_ * 180 / M_PI;
	pcg->longitude = L_ * 180 / M_PI;
}

//ECEF ---> ENU
//pcc为ECEF坐标系结构体指针，center为东北天坐标原点的指针，pct为东北天坐标系结构体指针
//坐标原点center要用GPS采到的第一个点的数据
void ECEFToENU(PECEF pcc, PWGS center, PENU pct)
{
	double dX, dY, dZ;
	PECEF Geodetic;
	Geodetic = (PECEF)GLOBAL_MALLOC(sizeof(ECEF));
	
	WGSToECEF(center, Geodetic);
	
	dX = pcc->x - Geodetic->x;
	dY = pcc->y - Geodetic->y;
	dZ = pcc->z - Geodetic->z;
	
	double B, L, H;
	B = (center->latitude)*M_PI/180;
	L = center->longitude*M_PI/180;
	H = center->height;
	
	pct->easting = -sin(L) * dX + cos(L) * dY; //X轴
	
	pct->northing = -sin(B) * cos(L) * dX - sin(B) * sin(L) * dY + cos(B) * dZ; //Y轴
	pct->upping = cos(B) * cos(L) * dX + cos(B) * sin(L) * dY + sin(B) * dZ;  //Z轴
	
	GLOBAL_FREE(Geodetic);
}

//ENU ---> ECEF
//pcc为ECEF坐标系结构体指针，center为东北天坐标原点的指针，pct为东北天坐标系结构体指针
//坐标原点center要用GPS采到的第一个点的数据
void ENUToECEF(PECEF pcc, PWGS center, PENU pct)
{
	PECEF Geodetic;
	Geodetic = (PECEF)GLOBAL_MALLOC(sizeof(ECEF));
	WGSToECEF(center, Geodetic);
	
	double B, L, H;
	B = center->latitude*M_PI/180;
	L = center->longitude*M_PI/180;
	H = center->height;
	
	pcc->x = -sin(B) * cos(L) * pct->northing - sin(L) * pct->easting + cos(B) * cos(L) * pct->upping + Geodetic->x;
	pcc->y = -sin(B) * sin(L) * pct->northing + cos(L) * pct->easting + cos(B) * sin(L) * pct->upping + Geodetic->y;
	pcc->z = cos(B) * pct->northing + sin(B) * pct->upping + Geodetic->z;
	
	GLOBAL_FREE(Geodetic);
}


void Coordi_initial(void)
{
	GLOBAL_MEMSET(&Lac_E,0x0,sizeof(Nav_std_sum));
	GLOBAL_MEMSET(&Lac_N,0x0,sizeof(Nav_std_sum));
	GLOBAL_MEMSET(&Lac_H,0x0,sizeof(Nav_std_sum));
	GLOBAL_MEMSET(&Vel_E,0x0,sizeof(Nav_std_sum));
	GLOBAL_MEMSET(&Vel_N,0x0,sizeof(Nav_std_sum));
	GLOBAL_MEMSET(&Vel_H,0x0,sizeof(Nav_std_sum));
	GLOBAL_MEMSET(&LLA_loc,0x0,sizeof(WGS));
	GLOBAL_MEMSET(&ORIGN_pot,0x0,sizeof(WGS));
	GLOBAL_MEMSET(&ECEF_loc,0x0,sizeof(ECEF));
	GLOBAL_MEMSET(&ENU_loc,0x0,sizeof(ENU));

	ORIGN_pot.latitude = 30.574869;
	ORIGN_pot.longitude = 104.058434;
	ORIGN_pot.height = 512.599976;

}

int Std_Devia( Nav_std_sum *Val)
{
	double Sum = Val->Sum;
	double Mean = Val->Mean;
	double Var = Val->Var;
	double Std = Val->Std;
	int    Num = Val->NUM;
	double Data = Val->Data;

	Num++;
	
	Sum = Sum+(Num-1)*(1.0)/Num*(Data-Mean)*(Data-Mean);
	Mean = Mean+(Data-Mean)/Num;

	//GLOBAL_PRINT(("Std_Devia data = %f,%f,%f,%f,%d,%f,\r\n",Sum,Mean,Var,Std,Num,Data));

	if(Num<=1)
	{
		Var = 0;
		Std = 0;
	}
	else
	{
		Var = Sum/(Num-1);
		if(Var<0)
		{
			ERR_PRINT(("Var < 0 ERROR!!\r\n"));
			ERR_PRINT(("Std_Devia data = %f,%f,%f,%f,%d,%f,\r\n",Sum,Mean,Var,Std,Num,Data));
			return -1;
		}
		Std = sqrt(Var);
	}

	Val->Sum = Sum;
	Val->Mean = Mean;
	Val->Var = Var;
	Val->Std = Std;
	Val->NUM = Num;
	Val->Data = Data;

	return 0;
}

int Std_Manage(ins_geopos_t *Nav_locat, ins_vel_t *Nav_vel)
{
	//int ret = 0;
	/*
	分别对位置与速度的东、北、天方向
	进行标准差递推计算
	*/
	double REAL_locE,REAL_locN,PRE_locE,PRE_locN,Miners_locN,Miners_locE;
	double REAL_velE,REAL_velN,PRE_velE,PRE_velN,Miners_velN,Miners_velE;
	
	if(!First_flag)
	{
		//位置部分
		if(Lac_N.Data > Pre_location.loc_N)
			REAL_locN = Lac_N.Data - Pre_location.loc_N;
		else
			REAL_locN = Pre_location.loc_N - Lac_N.Data;

		if(Lac_E.Data > Pre_location.loc_E)
			REAL_locE = Lac_E.Data - Pre_location.loc_E;
		else
			REAL_locE = Pre_location.loc_E - Lac_E.Data;

		PRE_locN = Vel_N.Data>0?Vel_N.Data:(-1.0*Vel_N.Data);
		PRE_locE = Vel_E.Data>0?Vel_E.Data:(-1.0*Vel_E.Data);

		Miners_locN = PRE_locN > REAL_locN ? (PRE_locN - REAL_locN):(REAL_locN - PRE_locN);
		Miners_locE = PRE_locE > REAL_locE ? (PRE_locE - REAL_locE):(REAL_locE - PRE_locE);

		//速度部分
		REAL_velN = Vel_N.Data>0?Vel_N.Data:(-1.0*Vel_N.Data);
		REAL_velE = Vel_E.Data>0?Vel_E.Data:(-1.0*Vel_E.Data);

		if(2*Pre_vel.vel_N[0] > Pre_vel.vel_N[1])
			PRE_velN = 2*Pre_vel.vel_N[0] - Pre_vel.vel_N[1];
		else
			PRE_velN = Pre_vel.vel_N[1] - 2*Pre_vel.vel_N[0];
		
		if(2*Pre_vel.vel_E[0] > Pre_vel.vel_E[1])
			PRE_velE = 2*Pre_vel.vel_E[0] - Pre_vel.vel_E[1];
		else
			PRE_velE = Pre_vel.vel_E[1] - 2*Pre_vel.vel_E[0];

		Miners_velN = PRE_velN > REAL_velN ? (PRE_velN - REAL_velN):(REAL_velN - PRE_velN);
		Miners_velE = PRE_velE > REAL_velE ? (PRE_velE - REAL_velE):(REAL_velE - PRE_velE);

		Nav_locat->std_e = Miners_locE;
		Nav_locat->std_n = Miners_locN;
		Nav_vel->std_ve = Miners_velE;
		Nav_vel->std_vn = Miners_velN;

	}
	Pre_location.loc_E = Lac_E.Data;
	Pre_location.loc_N = Lac_N.Data;
	Pre_vel.vel_E[1] = Pre_vel.vel_E[0];
	Pre_vel.vel_N[1] = Pre_vel.vel_N[0];
	Pre_vel.vel_E[0] = Vel_E.Data>0?Vel_E.Data:(-1.0*Vel_E.Data);
	Pre_vel.vel_N[0] = Vel_N.Data>0?Vel_N.Data:(-1.0*Vel_N.Data);

	
#if 0
	if(Std_Devia(&Lac_E)!=0){return -1;}
	Nav_locat->std_e = Lac_E.Std;
	if(Std_Devia(&Lac_N)!=0){return -1;}
	Nav_locat->std_n = Lac_N.Std;
	if(Std_Devia(&Lac_H)!=0){return -1;}
	Nav_locat->std_h = Lac_H.Std;
	if(Std_Devia(&Vel_E)!=0){return -1;}
	Nav_vel->std_ve = Vel_E.Std;
	if(Std_Devia(&Vel_N)!=0){return -1;}
	Nav_vel->std_vn = Vel_N.Std;
	if(Std_Devia(&Vel_H)!=0){return -1;}
	Nav_vel->std_vu = Vel_H.Std;
#endif
	return 0;
}

int Lat_Lon_ext(uint32_t time, ins_geopos_t *Nav_locat, ins_vel_t *Nav_vel)
{
	U32 delta_t=0;
	F64 delta_Sn = 0;
	F64 delta_Se = 0;
	F64 V = 0;
	F32 H = 0, angle = 0;
	F64 lat = 0,lon = 0,Vn = 0,Ve = 0;

	if(time - UBX_1PPS_time <= 0)
	{
		delta_t = 0;
		DBG_IMU_Print("1PPS ERROR!!\r\n");
		return -1;
	}
	else
	{
		//GLOBAL_PRINT(("UBX_1PPS_time = %d\r\n",UBX_1PPS_time));
		delta_t = time - UBX_1PPS_time;
	}

	lat = NMEA_Data_ptr.lat;
	lon = NMEA_Data_ptr.lon;
	angle = NMEA_Data_ptr.angle; //NMEA DATA
	V = NMEA_Data_ptr.veloc;     //NMEA DATA
	H = NMEA_Data_ptr.alt;

	//For test
	if(!FIRST_ANGLE_GET)
	{
		angle = 0;
	}

	if(angle<0||angle>=360||delta_t<=0||lat>90||lat<-90||lon>180||lon<-180||V<0)
	{
		DBG_IMU_Print("NMEA Data ERROR!!\r\n");
		return -1;
	}
	else if(angle>=0 && angle<360)
	{
		//先将延迟前的经纬度坐标写入坐标转换缓存结构体
		LLA_loc.latitude = lat;
		LLA_loc.longitude = lon;
		LLA_loc.height = H;

		//经纬度坐标转ECEF坐标
		WGSToECEF(&LLA_loc, &ECEF_loc);
		//ECEF坐标转东北天坐标
		ECEFToENU(&ECEF_loc, &ORIGN_pot, &ENU_loc);//得到延迟前的东北天坐标

		angle = angle*M_PI/180;  //角度转弧度
		
		//延迟距离计算与速度分解
		Vn = cos(angle)*V;
		Ve = sin(angle)*V;
		delta_Sn = Vn*delta_t/1000;//延迟北向距离
		delta_Se = Ve*delta_t/1000;//延迟东向距离

		//东北向分别增加延迟距离
		ENU_loc.northing += delta_Sn;
		ENU_loc.easting  += delta_Se;

		//转换后的ENU东北天坐标与分解后的速度，写入标准差递推结构体
		Lac_N.Data = ENU_loc.northing;
		Lac_E.Data = ENU_loc.easting;
		Lac_H.Data = ENU_loc.upping;
		Vel_N.Data = Vn;
		Vel_E.Data = Ve;
		Vel_H.Data = 0;

		//反向转换成经纬度，输入给融合算法使用
		ENUToECEF(&ECEF_loc, &ORIGN_pot, &ENU_loc);
		ECEFToWGS(&LLA_loc, &ECEF_loc);

		//将推算后的经纬度坐标和分解后的速度，写入融合算法的入参结构体
		Nav_locat->lat = LLA_loc.latitude;
		Nav_locat->lng = LLA_loc.longitude;
		Nav_locat->h = LLA_loc.height;
		Nav_vel->vn = Vn;
		Nav_vel->ve = Ve;
		Nav_vel->vu = 0;

		//GLOBAL_PRINT(("ENUData = %f,%f,%f\r\n",ENU_loc.northing,ENU_loc.easting,ENU_loc.upping));
		//GLOBAL_PRINT(("Vel_N&Vel_E = %f,%f\r\n",Vel_N.Data,Vel_E.Data));
	}

	return 0;
}

void ECEF_ENU_WGS_TEST(void)
{

		ORIGN_pot.latitude = 30.574869;
		ORIGN_pot.longitude = 104.058434;
		ORIGN_pot.height = 512.6;

		//先将延迟前的经纬度坐标写入坐标转换缓存结构体
		LLA_loc.latitude = 30.001;
		LLA_loc.longitude = 103.05341;
		LLA_loc.height = 600.599976;

		//经纬度坐标转ECEF坐标
		WGSToECEF(&LLA_loc, &ECEF_loc);

		GLOBAL_PRINT(("ECEF_loc_Before = %f,%f,%f\r\n",ECEF_loc.x,ECEF_loc.y,ECEF_loc.z));
		//ECEF坐标转东北天坐标
		ECEFToENU(&ECEF_loc, &ORIGN_pot, &ENU_loc);//得到延迟前的东北天坐标

		GLOBAL_PRINT(("ENU_loc = %f,%f,%f\r\n",ENU_loc.northing,ENU_loc.easting,ENU_loc.upping));

		//反向转换成经纬度，输入给融合算法使用
		ENUToECEF(&ECEF_loc, &ORIGN_pot, &ENU_loc);
		
		GLOBAL_PRINT(("ECEF_loc_After = %f,%f,%f\r\n",ECEF_loc.x,ECEF_loc.y,ECEF_loc.z));

		ECEFToWGS(&LLA_loc, &ECEF_loc);
		
		GLOBAL_PRINT(("WGS = %f,%f,%f\r\n",LLA_loc.latitude,LLA_loc.longitude,LLA_loc.height));

		delay_ms(1000);
}

#if 0	
int main()
{
	//定义所有坐标变量
	PWGS pcg; //GPS坐标
	pcg = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	PECEF pcc; //空间直角坐标
	pcc = (PECEF)GLOBAL_MALLOC(sizeof(ECEF));
	PWGS center; //东北天坐标原点对应的空间直角坐标
	center = (PWGS)GLOBAL_MALLOC(sizeof(WGS));
	PENU pct; //东北天坐标
	pct = (PENU)GLOBAL_MALLOC(sizeof(ENU));
	//经纬度数据自行设置
	center->latitude = ;
	center->longitude = ;
	center->height = ;
	
	pct->easting = ;
	pct->northing = ;
	pct->upping = ;
	
	//函数请自行调用
	
	GLOBAL_FREE(pcc);
	GLOBAL_FREE(pcg);
	GLOBAL_FREE(center);
	GLOBAL_FREE(pct);
	//free(pcb);
	
	return 0;
}
#endif

