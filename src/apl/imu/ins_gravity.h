#ifndef __INS_GRAVITY_H__
#define __INS_GRAVITY_H__


#ifdef __cplusplus
extern "C" {
#endif


#include "ins_baseparams.h"


int4 ins_GetGravityIne(double dX,double dY,double dZ, double* dRx, double* dRy, double* dRz);

int4 ins_GetGravityInL(double latitude, double dH, double veGravity[]);


#ifdef __cplusplus
}
#endif

#endif