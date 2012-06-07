#ifndef COMM_PACKETS_H
#define COMM_PACKETS_H

#include <inttypes.h>

#include "mav_common/comm_types.h"
  
#define MAV_ACK_PKT_ID                          0x00
#define MAV_CTRL_CFG_PKT_ID                     0x01
#define MAV_PID_CFG_PKT_ID                      0x02
#define MAV_KF_CFG_PKT_ID                       0x03
#define MAV_DUMMY_PKT_ID                        0x04
#define MAV_POSE2D_PKT_ID                       0x05
#define MAV_HEIGHT_PKT_ID                       0x06
#define MAV_POSE_PKT_ID                         0x07
#define MAV_FLIGHT_ACTION_PKT_ID                0x08
#define MAV_FLIGHT_STATE_PKT_ID                 0x09
#define MAV_IMU_PKT_ID                          0x0A
#define MAV_RCDATA_PKT_ID                       0x0B
#define MAV_TIMESYNC_PKT_ID                     0x0C
#define MAV_STATUS_PKT_ID                       0x0D
#define MAV_TX_FREQ_CFG_PKT_ID                  0x0E
#define MAV_CTRL_INPUT_PKT_ID                   0x0F
#define MAV_DES_POSE_PKT_ID                     0x10
#define MAV_DES_VEL_PKT_ID                      0x11
#define MAV_CTRL_DEBUG_PKT_ID                   0x12

// for testing the connection
typedef struct
__attribute__((packed))
{
  uint8_t dummy;
} MAV_DUMMY_PKT;

/// define at which frequencies data should be sent
typedef struct
__attribute__((packed))
{
  uint16_t imu_period;
  uint16_t rcdata_period;
  uint16_t flight_state_period;
  uint16_t pose_period;
  uint16_t status_period;
  uint16_t ctrl_debug_period;

  uint16_t imu_phase;
  uint16_t rcdata_phase;
  uint16_t flight_state_phase;
  uint16_t pose_phase;
  uint16_t status_phase;
  uint16_t ctrl_debug_phase;

} MAV_TX_FREQ_CFG_PKT;

/// acknowledge packet with message type that has been acknowledged
typedef struct
__attribute__((packed))
{
  uint8_t ack_packet;
} MAV_ACK_PKT;

// config packet for ctrl params
typedef struct
__attribute__((packed))
{
 // p, i, d, bias for x-position control
 Param k_p_x;
 Param k_i_x;
 Param k_d_x;
 Param k_d2_x;
 Param d_base_x;
 Param bias_x;
 Param max_i_x;
 Param max_err_x;

 // p, i, d, bias for y-position control
 Param k_p_y;
 Param k_i_y;
 Param k_d_y;
 Param k_d2_y;
 Param d_base_y;
 Param bias_y;
 Param max_i_y;
 Param max_err_y;

 // p, i, d, bias for x-velocity control
 Param k_p_vx;
 Param k_i_vx;
 Param k_d_vx;
 Param bias_vx;
 Param max_i_vx;
 Param max_err_vx;

 // p, i, d, bias for y-velocity control
 Param k_p_vy;
 Param k_i_vy;
 Param k_d_vy;
 Param bias_vy;
 Param max_i_vy;
 Param max_err_vy;

 // p, i, d, bias for y-velocity control
  Param k_p_vz;
  Param k_i_vz;
  Param k_d_vz;
  Param bias_vz;
  Param max_i_vz;
  Param max_err_vz;

 // p, i, d, bias for height control
 Param k_p_z;
 Param k_i_z;
 Param k_d_z;
 Param k_d2_z;
 Param bias_z;
 Param max_i_z;
 Param max_err_z;

 // p, i, d, bias for yaw control
 Param k_p_yaw;
 Param k_i_yaw;
 Param k_d_yaw;
 Param bias_yaw;
 Param max_i_yaw;
 Param max_err_yaw;

} MAV_PID_CFG_PKT;

// config packet for ctrl params
typedef struct
__attribute__((packed))
{
  uint8_t ctrl_mode_roll;
  uint8_t ctrl_mode_pitch;
  uint8_t ctrl_mode_yaw_rate;
  uint8_t ctrl_mode_thrust;

  Angle       cmd_roll_limit;       // roll limit,     SI
  Angle       cmd_pitch_limit;      // pitch limit,    SI
  AngVelocity cmd_yaw_rate_limit;   // yaw rate limit, SI
  Thrust      cmd_thrust_limit;     // thrust limit,   %

  Param cmd_roll_delta_limit;       // [rad / iteration]
  Param cmd_pitch_delta_limit;      // [rad / iteration]
  Param cmd_yaw_rate_delta_limit;   // [rad/s / iteration]
  Param cmd_thrust_delta_limit;     // [% / iteration]

} MAV_CTRL_CFG_PKT;

// debug packet for the controllers
typedef struct
__attribute__((packed))
{
  int16_t cmd_roll_LL;         // in LL units
  int16_t cmd_pitch_LL;        // in LL units
  int16_t cmd_yaw_rate_LL;     // in LL units
  int16_t cmd_thrust_LL;       // in LL units

  Angle cmd_roll;         // in SI units
  Angle cmd_pitch;        // in SI units
  Angle cmd_yaw_rate;     // in SI units
  Angle cmd_thrust;       // in SI units

  int16_t roll_limit;     // in LL units
  int16_t pitch_limit;    // in LL units
  int16_t yaw_rate_limit; // in LL units
  int16_t thrust_limit;   // in LL units

  int32_t error_x;
  int32_t error_y;
  int32_t error_z;
  int16_t error_yaw;

  float vel_x_bf; //velocity in the horizontal plan
  float vel_y_bf;
  float pid_error_vx_bf;
  float pid_error_vy_bf;
  float ax_bf; //acceleration in the horizontal plan
  float ay_bf; //acceleration in the horizontal plan
  float az;
  uint8_t ctrl_mode_roll;
  uint8_t ctrl_mode_pitch;
  uint8_t ctrl_mode_yaw_rate;
  uint8_t ctrl_mode_thrust;

  float acc_x_wf;
  float acc_y_wf;
  float acc_z_wf;

  float pid_x_i_term;    // in SI (%)
  float pid_y_i_term;    // in SI
  float pid_yaw_i_term;  // in SI
  float pid_z_i_term;    // in SI

  float pid_error_x_bf; // body frame x error in SI
  float pid_error_y_bf; // body frame y error in SI

} MAV_CTRL_DEBUG_PKT;

// 2d pose from laser scan matcher
typedef struct
__attribute__((packed))
{
  Position  x;     // x position, SI
  Position  y;     // y position, SI
  Velocity  vx;    // x velocity, SI
  Velocity  vy;    // y velocity, SI
  Angle     yaw;   // yaw position, SI
} MAV_POSE2D_PKT;

// height from laser altimeter
typedef struct
__attribute__((packed))
{
  Position z;      // z position, SI
  Velocity vz;     // z velocity, SI
} MAV_HEIGHT_PKT;

typedef struct
__attribute__((packed))
{
  Position x;     // x position, SI
  Position y;     // y position, SI
  Position z;     // z position, SI

  Velocity vx;    // x velocity, SI
  Velocity vy;    // y velocity, SI
  Velocity vz;    // y velocity, SI

  Angle roll;     // roll  orientation, SI
  Angle pitch;    // pitch orientation, SI
  Angle yaw;      // yaw   orientation, SI
} MAV_POSE_PKT;

// state action
typedef struct
__attribute__((packed))
{
  uint8_t action;
} MAV_FLIGHT_ACTION_PKT;

// flight state
typedef struct
__attribute__((packed))
{
  uint8_t state;
} MAV_FLIGHT_STATE_PKT;

// state action
typedef struct
__attribute__((packed))
{
  int64_t timestamp;           // timestamp in us; synchronized to host pc
  int64_t timesync_offset;

  float battery_voltage;     // battery voltage [mv]

  //uint16_t flight_time;        // Flight time [s]

  float cpu_load;           // cpu load in % of the time of one sdk loop

  //uint32_t rx_packets;         // total received packets
  //uint32_t rx_packets_good;    // total received good packets
} MAV_STATUS_PKT;

typedef struct
__attribute__((packed))
{
 Angle roll;  // roll  orientation, SI
 Angle pitch; // pitch orientation, SI
 Angle yaw;   // yaw   orientation, SI

 AngVelocity roll_rate;
 AngVelocity pitch_rate;
 AngVelocity yaw_rate;

 Acceleration acc_x;
 Acceleration acc_y;
 Acceleration acc_z;

} MAV_IMU_PKT;

typedef struct
__attribute__((packed))
{
 uint8_t enable_mask;

 Param Q_x;
 Param Q_y;
 Param Q_z;
 Param Q_roll;
 Param Q_ptch;
 Param Q_yaw;

 Param R_x; //laser x position covariace
 Param R_vx;//laser x velocity covariace
 Param R_y;
 Param R_vy;
 Param R_z;
 Param R_vz;
 Param R_vz_p; //for pressure sensor
 Param R_roll;
 Param R_ptch;
 Param R_yaw;

} MAV_KF_CFG_PKT;

/// contains RC data that was transmitted to the helicopter
typedef struct
__attribute__((packed))
{
  /// timestamp in us; synchronized to host pc
  int64_t timestamp;

  uint16_t channel[8];

  // channel[0]: Pitch
  // channel[1]: Roll
  // channel[2]: Thrust
  // channel[3]: Yaw
  // channel[4]: Serial interface enable/disable
  // channel[5]: manual / height control / GPS + height control
  //
  // range of each channel: 0..4095

} MAV_RCDATA_PKT;

/// packet for time synchronization between HL processor and PC
/**
 *   Client |        | Server
 *          |        |
 *      tc1 |--------| ts1
 *          |        |
 *      tc2 |--------| ts2
 *          |        |
 *
 *  ts1=ts2; tc2 is measured on the arrival of the packet
 */
typedef struct
__attribute__((packed))
{
  int64_t tc1;
  int64_t ts1;
} MAV_TIMESYNC_PKT;

// Used in direct control mode
typedef struct
__attribute__((packed))
{
  Angle       cmd_roll;      // roll  ,    in SI
  Angle       cmd_pitch;     // pitch ,    in SI
  AngVelocity cmd_yaw_rate;  // yaw_rate,  in SI
  Thrust      cmd_thrust;    // thrust,    in %
} MAV_CTRL_INPUT_PKT;

typedef struct
__attribute__((packed))
{
  Position x;     // desired x position, in SI
  Position y;     // desired y position, in SI
  Position z;     // desired z position, in SI

  Angle yaw;          // desired yaw orientation, in SI
} MAV_DES_POSE_PKT;

typedef struct
__attribute__((packed))
{
  Velocity vx;    // desired x velocity, in SI
  Velocity vy;    // desired y velocity, in SI
  Velocity vz;

  AngVelocity yaw_rate;  // desired yaw rate, in SI
} MAV_DES_VEL_PKT;

#endif // COMM_PACKETS_H
