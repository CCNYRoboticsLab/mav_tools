#ifndef COMM_H
#define COMM_H

// KF packet bits

#define MAV_KF_X_BIT           0
#define MAV_KF_Y_BIT           1
#define MAV_KF_Z_BIT           2
#define MAV_KF_ROLL_BIT        3
#define MAV_KF_PITCH_BIT       4
#define MAV_KF_YAW_BIT         5
#define MAV_KF_RESET_BIT       7

// LL_CMD packet bits

#define MAV_LL_CMD_PITCH_BIT        0
#define MAV_LL_CMD_ROLL_BIT         1
#define MAV_LL_CMD_YAW_RATE_BIT     2
#define MAV_LL_CMD_THRUST_BIT       3
#define MAV_LL_CMD_HEIGHT_BIT       4
#define MAV_LL_CMD_GPS_BIT          5

#define MAV_LL_CMD_ROLL_MASK         (1<<MAV_LL_CMD_ROLL_BIT)
#define MAV_LL_CMD_PITCH_MASK        (1<<MAV_LL_CMD_PITCH_BIT)
#define MAV_LL_CMD_YAW_RATE_MASK     (1<<MAV_LL_CMD_YAW_RATE_BIT)
#define MAV_LL_CMD_THRUST_MASK       (1<<MAV_LL_CMD_THRUST_BIT)
#define MAV_LL_CMD_HEIGHT_MASK       (1<<MAV_LL_CMD_HEIGHT_BIT)
#define MAV_LL_CMD_GPS_MASK          (1<<MAV_LL_CMD_GPS_BIT)

#define MAV_LL_CMD_RPYT_MASK (MAV_LL_CMD_ROLL_MASK || MAV_LL_CMD_PITCH_MASK || MAV_LL_CMD_YAW_RATE_MASK || MAV_LL_CMD_THRUST_MASK)

// defaukt serial baudrate speed

#define MAV_DEFAULT_BAUDRATE                    460800

// packet transmit periods (in ms)

/*
#define MAV_DEFAULT_TX_PERIOD_POSE                   5
#define MAV_DEFAULT_TX_PERIOD_IMU                   20
#define MAV_DEFAULT_TX_PERIOD_FLIGHT_STATE          20
#define MAV_DEFAULT_TX_PERIOD_RCDATA               100
#define MAV_DEFAULT_TX_PERIOD_STATUS               100
#define MAV_DEFAULT_TX_PERIOD_CTRL_DEBUG           100

// packet transmit phases (in ms)
#define MAV_DEFAULT_TX_PHASE_POSE                   1
#define MAV_DEFAULT_TX_PHASE_IMU                   10
#define MAV_DEFAULT_TX_PHASE_FLIGHT_STATE          15
#define MAV_DEFAULT_TX_PHASE_RCDATA                 3
#define MAV_DEFAULT_TX_PHASE_STATUS                57
#define MAV_DEFAULT_TX_PHASE_CTRL_DEBUG            77
*/

// flight actions
#define MAV_ACTION_ESTOP          							0x00
#define MAV_ACTION_ADVANCE_STATE  							0x01
#define MAV_ACTION_RETREAT_STATE  							0x02
#define MAV_ACTION_TOGGLE_ENGAGE  							0x03
#define MAV_ACTION_TAKEOFF        							0x04
#define MAV_ACTION_LAND           					    0x05

// flight states
#define MAV_STATE_OFF          									0x00
#define MAV_STATE_ENGAGING                      0x01
#define MAV_STATE_DISENGAGING                   0x02
#define MAV_STATE_IDLE        									0x03
#define MAV_STATE_LANDING      				 					0x04
#define MAV_STATE_FLYING                        0x05
#define MAV_STATE_ERROR       									0xFF

// control modes - for the RPYT axis controllers
#define MAV_CTRL_MODE_DISABLED      0x00
#define MAV_CTRL_MODE_DIRECT        0x01
#define MAV_CTRL_MODE_POSITION      0x02
#define MAV_CTRL_MODE_VELOCITY      0x03

// defines that a packet has to be acknowledged when received

#define MAV_COMM_ACK                0x01

#endif // COMM_H
