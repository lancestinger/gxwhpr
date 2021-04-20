#include "ins_gravity.h"
#include "comm/project_def.h"

#include <math.h>

#define J2 (0.00108263)
#define J4 (-2.37091222e-6)
#define J6 (6.08347e-9)


/*
 * From a solid coordinates space coordinates (X, Y, Z), access to the gravity vector component in e
*/
int4 ins_GetGravityIne(double dX,double dY,double dZ, double* dRx, double* dRy, double* dRz)
{
	double dD = sqrt(dX*dX + dY*dY + dZ*dZ);

	double dK = WGS84_a/dD; //Ellipsoid axis  divided by geocentric vector	
	double dK2 = dK * dK;   //K square
	double dK4 = dK2 * dK2; //K quartic 
	double dK6 = dK4 * dK2;

	double a1 = -WGS84_GM/(dD*dD);
	double a2 = 1.0 + 1.5*J2*dK2 - 15.0/8.0*J4*dK4 + 35.0/16.0*J6*dK6;
	double a3 = -4.5*J2*dK2 + 75.0/4.0*J4*dK4 - 735.0/16.0*J6*dK6;
	double a4 = -175.0/8.0*J4*dK4 +2205.0/16.0*J6*dK6;
	double a5 = -1617.0/16.0*J6*dK6;

	double b1 = 3.0*J2*dK2 - 15.0/2.0*J4*dK4 + 105.0/8.0*J6*dK6;
	double b2 = 35.0/2.0*J4*dK4 - 945.0/12.0*J6*dK6;
	double b3 = 693.0/8.0*J6*dK6;

	double c1 = a2;
	double c2 = a3 - b1;
	double c3 = a4 - b2;
	double c4 = a5 - b3;

	double d1 = a2 + b1;
	double d2 = c2 + b2;
	double d3 = c3 + b3;
	double d4 = c4;

	double t = dZ/dD; //Ellipsoid Z-axis vector coordinates divided by the geocentric
	double t2 = t  *  t;
	double t4 = t2 * t2;
	double t6 = t4 * t2;

	*dRx = a1/dD * (c1 + c2*t2 + c3*t4 + c4*t6) * dX + WGS84_AngleRate * WGS84_AngleRate * dX;
	*dRy = a1/dD * (c1 + c2*t2 + c3*t4 + c4*t6) * dY + WGS84_AngleRate * WGS84_AngleRate * dY;
	*dRz = a1/dD * (d1 + d2*t2 + d3*t4 + d4*t6) * dZ + 0;

	return 1;
}


// latitude: in deg
int4 ins_GetGravityInL(double latitude, double dH, double veGravity[])
{
	double dB = latitude * DEG2RAD;

	//Calculate gravity vector in L
	veGravity[0] = veGravity[1] = veGravity[2] = 0;

	veGravity[2] = 9.7803267714 * ( 1.0 + 0.00527094*sin(dB)*sin(dB) + 0.0000232718*pow(sin(dB), 4) ) - 0.3086e-5 * dH;
	veGravity[2] *= -1.0;

	return 1;
}
