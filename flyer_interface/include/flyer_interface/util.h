/*
 *  AscTec Autopilot Interface
 *  Copyright (C) 2011, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FLYER_INTERFACE_UTIL_H
#define FLYER_INTERFACE_UTIL_H

#include <stdint.h>
#include <math.h>

namespace asctec
{

#define GRAVITY 9.810665

inline double commToSIAngle(uint16_t angle);
inline uint16_t SIToCommAngle(double angle);

inline double commToSIPosition(int32_t position);
inline int32_t SIToCommPosition(double position);

inline double commToSILinearVelocity(int16_t velocity);
inline int16_t SIToCommLinearVelocity(double velocity);

inline double degToRad(double angle);
inline double radToDeg(double angle);

inline float commToSIAcc(int16_t acc);

inline float commToSIKFCov(uint32_t cov);
inline uint32_t SIToCommKFCov(float cov);

inline float commToSIVoltage(int16_t voltage);

// **** for direct motor control

inline int16_t SItoCommCmdRoll    (float roll);
inline int16_t SItoCommCmdPitch   (float pitch);
inline int16_t SItoCommCmdYawRate (float yaw_rate);
inline int16_t SItoCommCmdThrust  (float thrust);

inline float commToSICmdRoll    (int16_t roll);
inline float commToSICmdPitch   (int16_t pitch);
inline float commToSICmdYawRate (int16_t yaw_rate);
inline float commToSICmdThrust  (int16_t thrust);

// ******************************************

inline double commToSIAngle(uint16_t angle)
{
  return degToRad(static_cast<double>(angle) / 100.0);
}

inline uint16_t SIToCommAngle(double angle)
{
  return static_cast<uint16_t>(radToDeg(angle * 100.0));
}

inline double commToSIPosition(int32_t position)
{
  return static_cast<double>(position) / 1000.0;
}

inline int32_t SIToCommPosition(double position)
{
  return static_cast<int32_t>(position * 1000.0);
}

inline double commToSILinearVelocity(int16_t velocity)
{
  return static_cast<double>(velocity) / 1000.0;
}

inline int16_t SIToCommLinearVelocity(double velocity)
{
  return static_cast<int32_t>(velocity * 1000.0);
}

inline double degToRad(double angle)
{
  return angle * M_PI / 180.0;
}

inline double radToDeg(double angle)
{
  return angle * 180.0 / M_PI;
}

// normalizes to [0, 2pi)
inline void normalizeSIAngle(double& angle)
{
  while (angle  <        0.0) angle += 2.0 * M_PI;
  while (angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;
}

float commToSIAcc(int16_t acc)
{
   return ( (float)(acc) ) * GRAVITY / 1000.0;
}

inline float commToSIKFCov(uint32_t cov)
{
  return (float)(cov) / 1000000.0;
}

inline uint32_t SIToCommKFCov(float cov)
{
  return (uint32_t)(cov * 1000000.0);
}

inline float commToSIVoltage(int16_t voltage)
{
  return (float)(voltage) / 1000.0;  
}

// **** for direct motor control

inline int16_t SItoCommCmdRoll(float roll)
{
  return (int16_t)(radToDeg(roll) * 100.0);
}

inline int16_t SItoCommCmdPitch(float pitch)
{
  return (int16_t)(radToDeg(pitch) * 100.0);
}

inline int16_t SItoCommCmdYawRate(float yaw_rate)
{
  return (int16_t)(radToDeg(yaw_rate) * 100.0);
}

inline int16_t SItoCommCmdThrust (float thrust)
{
  return (int16_t)(thrust * 100.0);
}

inline float commToSICmdRoll(int16_t roll)
{
  return degToRad((float)(roll) / 100.0);
}

inline float commToSICmdPitch(int16_t pitch)
{
  return degToRad((float)(pitch) / 100.0);
}

inline float commToSICmdYawRate (int16_t yaw_rate)
{
  return degToRad((float)(yaw_rate) / 100.0);
}

inline float commToSICmdThrust  (int16_t thrust)
{
  return (float)(thrust) / 100.0;
}

} // end namespace asctec

#endif // FLYER_INTERFACE_UTIL_H 
