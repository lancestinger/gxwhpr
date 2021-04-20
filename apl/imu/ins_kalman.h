#ifndef __INS_KALMAN_H__
#define __INS_KALMAN_H__


#ifdef __cplusplus
extern "C" {
#endif


#include "ins_baseparams.h"
#include "ins_matrix.h"


void ins_kalman_init();

void ins_kalman_TimeUpdate(float dt);

void ins_kalman_MeasureUpdate(const ins_geopos_t* pPos, const ins_vel_t* pVel, ins_geopos_t* pPosFus, ins_vel_t* pVelFus);


#ifdef __cplusplus
}
#endif


#endif