#ifndef COMM_UTIL_H
#define COMM_UTIL_H

#include <stdint.h>
#include <math.h>

#include "mav_common/comm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void normalizeSIAnglePi (float * angle);
void normalizeSIAngle2Pi(float * angle);

/*
// ****************** accel ***********************

int32_t SItoCommAcc(float acc);
float commToSIAcc(int32_t acc);

// ****************** angle ***********************

float commToSIAngle(uint16_t angle);
uint16_t SIToCommAngle(float angle);

// ****************** position ***********************

float commToSIPosition(int32_t pos);
int32_t SIToCommPosition(float pos);

float commToSIKFCov(uint32_t cov);
uint32_t SIToCommKFCov(float cov);
float commToSIVelocity(int16_t vel);
int16_t SIToCommVelocity(float vel);

// **** for direct motor control

int16_t SItoCommCmdRoll    (float roll);
int16_t SItoCommCmdPitch   (float pitch);
int32_t SItoCommCmdYawRate (float yaw_rate);
int16_t SItoCommCmdThrust  (float thrust);

float commToSICmdRoll    (int16_t roll);
float commToSICmdPitch   (int16_t pitch);
float commToSICmdYawRate (int32_t yaw_rate);
float commToSICmdThrust  (int16_t thrust);
*/
#ifdef __cplusplus
};
#endif

#endif
