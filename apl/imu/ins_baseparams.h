#ifndef __INS_BASE_PARAMS_H__
#define __INS_BASE_PARAMS_H__

#ifdef __cplusplus
extern "C" {
#endif


//Data Type definition
typedef unsigned char       uint1; //Symbol as u1
typedef unsigned short      uint2; //Symbol as u2
typedef unsigned long       uint4; //Symbol as u4
typedef char		         int1;  //Symbol as i1
typedef short		         int2;  //Symbol as i2
typedef int		             int4;  //Symbol as i4
typedef long long int        int8;
typedef float		           f4;  //Symbol as f4
typedef double		           f8;  //Symbol as f8
//typedef int                    bool;


///////////////////////////////////////////
//Standard constants

#ifndef false
#define false  0
#endif

#ifndef true
#define true   1
#endif


/*
* macro definition
*/
#define PI               (3.141592653589793)
#define DEG2SEMI         (1.0/180.0)			 //the factor of degree to semicircle
#define SEMI2RAD         (PI)				     //the factor of semicircle to radian, i.e. PI
#define DEG2RAD          (PI/180.0)			     //the factor of degree to radian
#define SEC2RAD          (PI/180.0/3600.0)		 //the factor of second to radian
#define RAD2DEG          (180.0/PI)			     //the factor of radian to degree
#define RAD2SEC          (3600.0*180.0/PI)       //the factor of radian to second
#define EPSILON          (1e-15)				 //Minimum limitation
#define INFINITY         (1e15)					//Maximum limitation, infinity


///////////////////////////////////////////
//WGS84 constants
#define	WGS84_a			(6378137.0		)	//equatorial (semimajor) radius of the WGS84 ellipsoid: m
#define	WGS84_f			(1.0/298.257223563)	//flattening of WGS84 ellipsoid
#define	WGS84_GM		(3.986005e14	)	//Earth gravitation constant, it's the product of gravitation constant and WGS84 earth mass: m^3/s^2
#define	WGS84_AngleRate	(7.292115e-5	)	//the angle rate of WGS84 earth rotation: radian/s	
#define	WGS84_e2		((2.0-WGS84_f)*WGS84_f)	//the square of first eccentricity of WGS84 ellipsoid, e2 = f * (2 - f)

#define SENSOR_SAMPLE_RATE          (50)
 

/*
* struct definition
*/
typedef struct _imu_rawdata
{
	int8 t;
	float x;
	float y;
	float z;

} imu_rawdata_t;

typedef struct _imu_data
{
	int8 t;
	float x;
	float y;
	float z;

} imu_data_t;

typedef struct _StepInfo
{
	int tTag;
	float acceNorm;
	float stepIntrvl;

} StepInfo_t;

typedef struct
{
	double q0;
	double q1;
	double q2;
	double q3;

} cQuaternion;

enum SENSOR_TYPE
{
	SENSOR_TYPE_ACCE,
	SENSOR_TYPE_MAG,
	SENSOR_TYPE_GYRO
};

enum PDR_DEVICE_TAKE_MODE 
{
	PDR_DEVICE_TAKE_MODE_UNKNOWN,
	PDR_DEVICE_TAKE_MODE_TEXTURING,
	PDR_DEVICE_TAKE_MODE_DANGLING,
	PDR_DEVICE_TAKE_MODE_PHONE,
	PDR_DEVICE_TAKE_MODE_POCKET
};

typedef enum pdr_sensor_data_type
{
	SENSOR_DATA_TYPE_ACCE,
	SENSOR_DATA_TYPE_MAG,
	SENSOR_DATA_TYPE_GYRO

} pdr_sensor_data_type_t;

typedef enum pdr_motion_mode_type
{ 
	PDR_MOTION_MODE_UNKNOWN,
	PDR_MOTION_MODE_WALKING,
	PDR_MOTION_MODE_DRIVING,
	PDR_MOTION_MODE_STATIC_STRICT,
	PDR_MOTION_MODE_STATIC_LOOSE

} pdr_motion_mode_type_t;


typedef struct _ins_geoPos
{
	// latitude, longitude, geodetic height;
	double lat;
	double lng;
	float h;

	// standard deviation of position in e, n, h
	double std_e;
	double std_n;
	double std_h;
    
	/*
	_ins_geoPos()
	{
		std_e = 1.0;
		std_n = 1.0;
		std_h = 1.0;
	}
	*/

} ins_geopos_t;

typedef struct _ins_vel
{
	// velocity in e, n, h
	double ve;
	double vn;
	double vu;

	// standard deviation of velocity in e, n, h
	double std_ve;
	double std_vn;
	double std_vu;

	/*
	_ins_vel()
	{
		std_ve = 0.3;
		std_vn = 0.3;
		std_vu = 0.3;

	}
	*/

} ins_vel_t;

typedef struct _ins_atti
{
	/* definition of pitch, roll, yaw :
	* from the view of positive direction of rotation axis, b-frame related to L-frame in counter-clockwise is positive.
	*/    
	float p;
	float r; 
	float y; //Orientation, in rad

	// standard deviation of pitch, roll, yaw
	float std_p;
	float std_r;
	float std_y;

} ins_atti_t;


#ifdef __cplusplus
}
#endif

#endif