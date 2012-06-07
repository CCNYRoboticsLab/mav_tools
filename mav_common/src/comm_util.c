#include "mav_common/comm_util.h"

void normalizeSIAngle2Pi(Angle * angle)
{
  if      (*angle <         0.0) *angle += 2.0 * M_PI;
  else if (*angle >= 2.0 * M_PI) *angle -= 2.0 * M_PI;
}

void normalizeSIAnglePi(Angle * angle)
{
  if      (*angle <  -M_PI) *angle += 2.0 * M_PI;
  else if (*angle >=  M_PI) *angle -= 2.0 * M_PI;
}

/*
// ****************** covariance ***********************

float commToSIKFCov(uint32_t cov)
{
  return (float)(cov) / 1000000.0;
}

uint32_t SIToCommKFCov(float cov)
{
  return (uint32_t)(cov * 1000000.0);
}

// ****************** accel ***********************

int32_t SItoCommAcc(float acc)
{
  return (int16_t)(acc * 1000.0);
}

float commToSIAcc(int32_t acc)
{
  return ((float)(acc)) / 1000.0;
}

// ****************** angle ***********************

float commToSIAngle(uint16_t angle)
{
  return ((float)(angle)) / 65535 * (2.0 * M_PI);
}

uint16_t SIToCommAngle(float angle)
{
  return (int)(angle / (2.0 * M_PI) * 65535.0);
}

// ****************** position ***********************

float commToSIPosition(int32_t pos)
{
  return ((float)(pos) ) / 1000.0;
}

int32_t SIToCommPosition(float pos)
{
  return (int32_t)(pos * 1000.0);
}

// ****************** velocity ***********************

float commToSIVelocity(int16_t vel)
{
  return ((float)(vel) ) / 1000.0;
}

int16_t SIToCommVelocity(float vel)
{
  return (int16_t)(vel * 1000.0);
}

// ****************** Motor commands ****************

int16_t SItoCommCmdRoll(float roll)
{
  return (int16_t)(roll / (2.0 * M_PI) * 65535.0);
  //return (int16_t)(roll * 1000.0);
}

int16_t SItoCommCmdPitch(float pitch)
{
  return (int16_t)(pitch / (2.0 * M_PI) * 65535.0);
  //return (int16_t)(pitch * 1000.0);
}

int32_t SItoCommCmdYawRate(float yaw_rate)
{
  return (int32_t)(yaw_rate / (2.0 * M_PI) * 65535.0);
  //return (int16_t)(yaw_rate * 1000.0);
}

int16_t SItoCommCmdThrust (float thrust)
{
  return (int16_t)(thrust * 100.0);
}

float commToSICmdRoll(int16_t roll)
{
  return ((float)(roll)) / 65535 * (2.0 * M_PI);
  //return ((float)(roll) / 1000.0);
}

float commToSICmdPitch(int16_t pitch)
{
  return ((float)(pitch)) / 65535 * (2.0 * M_PI);
  //return ((float)(pitch) / 1000.0);
}

float commToSICmdYawRate (int32_t yaw_rate)
{
  return ((float)(yaw_rate)) / 65535 * (2.0 * M_PI);
  //return ((float)(yaw_rate) / 1000.0);
}

float commToSICmdThrust  (int16_t thrust)
{
  return ((float)(thrust) / 100.0);
}

*/
