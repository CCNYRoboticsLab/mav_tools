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

#ifndef FLYER_INTERFACE_FLYER_INTERFACE_H
#define FLYER_INTERFACE_FLYER_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/service_server.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <mav_msgs/Height.h>
#include <mav_msgs/RC.h>
#include <mav_msgs/Roll.h>
#include <mav_msgs/Pitch.h>
#include <mav_msgs/YawRate.h>
#include <mav_msgs/Thrust.h>

#include <mav_srvs/AdvanceState.h>
#include <mav_srvs/RetreatState.h>
#include <mav_srvs/GetFlightState.h>
#include <mav_srvs/ESTOP.h>
#include <mav_srvs/ToggleEngage.h>
#include <mav_srvs/Takeoff.h>
#include <mav_srvs/Land.h>
#include <mav_srvs/SetCtrlType.h>

#include <mav_common/comm.h>
#include <mav_common/comm_packets.h>
#include <mav_common/comm_util.h>
#include <mav_common/control_types.h>

#include "flyer_interface/serial_communication.h"

#include "flyer_interface/PIDXConfig.h"
#include "flyer_interface/PIDYConfig.h"
#include "flyer_interface/PIDZConfig.h"
#include "flyer_interface/PIDVXConfig.h"
#include "flyer_interface/PIDVYConfig.h"
#include "flyer_interface/PIDVZConfig.h"
#include "flyer_interface/PIDYawConfig.h"
#include "flyer_interface/CommConfig.h"
#include "flyer_interface/CtrlConfig.h"

namespace mav
{

using namespace message_filters::sync_policies;

class FlyerInterface
{
  typedef flyer_interface::PIDXConfig   PIDXConfig;
  typedef flyer_interface::PIDYConfig   PIDYConfig;
  typedef flyer_interface::PIDZConfig   PIDZConfig;
  typedef flyer_interface::PIDVXConfig  PIDVXConfig;
  typedef flyer_interface::PIDVYConfig  PIDVYConfig;
  typedef flyer_interface::PIDVZConfig  PIDVZConfig;
  typedef flyer_interface::PIDYawConfig PIDYawConfig;
  typedef flyer_interface::CommConfig   CommConfig;
  typedef flyer_interface::CtrlConfig   CtrlConfig;

  typedef dynamic_reconfigure::Server<PIDXConfig>   CfgXServer;
  typedef dynamic_reconfigure::Server<PIDYConfig>   CfgYServer;
  typedef dynamic_reconfigure::Server<PIDZConfig>   CfgZServer;
  typedef dynamic_reconfigure::Server<PIDVXConfig>  CfgVXServer;
  typedef dynamic_reconfigure::Server<PIDVYConfig>  CfgVYServer;
  typedef dynamic_reconfigure::Server<PIDVZConfig>  CfgVZServer;
  typedef dynamic_reconfigure::Server<PIDYawConfig> CfgYawServer;
  typedef dynamic_reconfigure::Server<CommConfig>   CfgCommServer;
  typedef dynamic_reconfigure::Server<CtrlConfig>   CfgCtrlServer;

  typedef geometry_msgs::PoseStamped  PoseStamped;
  typedef geometry_msgs::TwistStamped TwistStamped;

  typedef message_filters::sync_policies::ApproximateTime<PoseStamped, TwistStamped> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Subscriber<PoseStamped>  PoseStampedSubscriber; 
  typedef message_filters::Subscriber<TwistStamped> TwistStampedSubscriber;

  public:

    FlyerInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~FlyerInterface();

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // dynamic reconfigure
    CfgCtrlServer cfg_ctrl_srv_;
    CfgXServer cfg_x_srv_;
    CfgYServer cfg_y_srv_;
    CfgZServer cfg_z_srv_;
    CfgVXServer cfg_vx_srv_;
    CfgVYServer cfg_vy_srv_;
    CfgVZServer cfg_vz_srv_;
    CfgYawServer cfg_yaw_srv_;
    CfgCommServer cfg_comm_srv_;

    // subscriber for laser pose and velocity
    boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<PoseStampedSubscriber>  laser_pose_subscriber_;
    boost::shared_ptr<TwistStampedSubscriber> laser_vel_subscriber_;

    // subscriber for laser height
    ros::Subscriber height_subscriber_;

    // subscribers for direct ctrl input
    ros::Subscriber cmd_roll_subscriber_;
    ros::Subscriber cmd_pitch_subscriber_;
    ros::Subscriber cmd_yaw_rate_subscriber_;
    ros::Subscriber cmd_thrust_subscriber_;
    ros::Subscriber cmd_pose_subscriber_;
    ros::Subscriber cmd_vel_subscriber_;

    // publishers
    ros::Publisher pose_publisher_;
    ros::Publisher vel_publisher_;
    ros::Publisher imu_publisher_;
    ros::Publisher flight_state_publisher_;
    ros::Publisher battery_voltage_publisher_;
    ros::Publisher cpu_load_publisher_;
    ros::Publisher cpu_load_avg_publisher_;

    // debug publishers
    ros::Publisher debug_imu_wf_publisher_;
    ros::Publisher debug_cmd_roll_publisher_;
    ros::Publisher debug_cmd_pitch_publisher_;
    ros::Publisher debug_cmd_yaw_rate_publisher_;
    ros::Publisher debug_cmd_thrust_publisher_;
    ros::Publisher debug_cmd_yaw_publisher_;
    ros::Publisher debug_pid_x_i_term_publisher_;
    ros::Publisher debug_pid_y_i_term_publisher_;
    ros::Publisher debug_pid_yaw_i_term_publisher_;
    ros::Publisher debug_pid_z_i_term_publisher_;
    ros::Publisher debug_roll_publisher_;
    ros::Publisher debug_pitch_publisher_;
    ros::Publisher debug_yaw_publisher_;
    ros::Publisher debug_err_x_bf_publisher_;
    ros::Publisher debug_err_y_bf_publisher_;
    ros::Publisher debug_err_vx_bf_publisher_;
    ros::Publisher debug_err_vy_bf_publisher_;
    ros::Publisher debug_ax_bf_publisher_;
    ros::Publisher debug_ay_bf_publisher_;
    ros::Publisher debug_az_publisher_;
    ros::Publisher debug_vx_bf_publisher_;
    ros::Publisher debug_vy_bf_publisher_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::ServiceServer advance_state_srv_;
    ros::ServiceServer retreat_state_srv_;
    ros::ServiceServer toggle_engage_srv_;
    ros::ServiceServer takeoff_srv_;
    ros::ServiceServer land_srv_;
    ros::ServiceServer set_ctrl_type_srv_;
    ros::ServiceServer get_flight_state_srv_;
    ros::ServiceServer estop_srv_;

    ros::Timer cmd_timer_;
  
    // **** state variables    

    boost::mutex state_mutex_;

    bool connected_;
    uint8_t flight_state_;

    std::vector<double> cpu_loads_;
    int cpu_load_index_;

    SerialCommunication comm_;

    float cmd_roll_, cmd_pitch_, cmd_yaw_rate_, cmd_thrust_;

    MAV_TX_FREQ_CFG_PKT tx_freq_cfg_packet_;
    MAV_PID_CFG_PKT     pid_cfg_packet_;
    MAV_CTRL_CFG_PKT    ctrl_cfg_packet_;
  
    // **** parameters
    
    bool publish_debug_;

    bool publish_pose_; // if set to true, the pose received over the serial
                        // interface is published as a pose and tf

    int baudrate_;
    std::string serial_port_tx_;
    std::string serial_port_rx_;

    std::string fixed_frame_;
    std::string base_frame_;

    bool enable_kf_x_;
    bool enable_kf_y_;
    bool enable_kf_z_;
    bool enable_kf_yaw_;

    double q_x_, q_y_, q_z_, q_yaw_;     // process matric covariance
    double r_x_, r_y_, r_z_, r_yaw_;     // measurement matrix covariance
    double r_vx_, r_vy_, r_vz_, r_vz_p_; // measurement matrix covariance (velocities)

    // **** member functions

    void initializeParams();

    void sendCommConfig();    
    void sendKFConfig(bool reset);
    void sendCtrlConfig();
    void sendPIDConfig();    

    void processImuData(uint8_t * buf, uint32_t bufLength);
    void processPoseData(uint8_t * buf, uint32_t bufLength);
    void processRCData(uint8_t * buf, uint32_t bufLength);
    void processFlightStateData(uint8_t * buf, uint32_t bufLength);
    void processTimeSyncData(uint8_t * buf, uint32_t bufLength);
    void processStatusData(uint8_t * buf, uint32_t bufLength);
    void processCtrlDebugData(uint8_t * buf, uint32_t bufLength);

    void laserCallback(const PoseStamped::ConstPtr  pose_msg,
                       const TwistStamped::ConstPtr twist_msg);
    void heightCallback(const mav_msgs::Height::ConstPtr height_msg);

    void cmdRollCallback(const mav_msgs::Roll::ConstPtr roll_msg);
    void cmdPitchCallback(const mav_msgs::Pitch::ConstPtr pitch_msg);
    void cmdYawRateCallback(const mav_msgs::YawRate::ConstPtr yaw_rate_msg);
    void cmdThrustCallback(const mav_msgs::Thrust::ConstPtr thrust_msg);
    void cmdPoseCallback(const geometry_msgs::PoseStamped::ConstPtr cmd_pose_msg);
    void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr cmd_vel_msg);
    
    void cmdTimerCallback(const ros::TimerEvent& event);

    bool advanceState(mav_srvs::AdvanceState::Request  &req,
                      mav_srvs::AdvanceState::Response &res);

    bool retreatState(mav_srvs::AdvanceState::Request  &req,
                      mav_srvs::AdvanceState::Response &res);

    bool getFlightState(mav_srvs::GetFlightState::Request  &req,
                        mav_srvs::GetFlightState::Response &res);

    bool estop(mav_srvs::ESTOP::Request  &req,
               mav_srvs::ESTOP::Response &res);

    bool toggleEngage(mav_srvs::ToggleEngage::Request  &req,
                      mav_srvs::ToggleEngage::Response &res);

    bool land(mav_srvs::Land::Request  &req,
              mav_srvs::Land::Response &res);

    bool takeoff(mav_srvs::Takeoff::Request  &req,
                 mav_srvs::Takeoff::Response &res);

    bool setCtrlType(mav_srvs::SetCtrlType::Request  &req,
                     mav_srvs::SetCtrlType::Response &res);

    void reconfig_x_callback(PIDXConfig& config, uint32_t level);
    void reconfig_y_callback(PIDYConfig& config, uint32_t level);
    void reconfig_z_callback(PIDZConfig& config, uint32_t level);
    void reconfig_vx_callback(PIDVXConfig& config, uint32_t level);
    void reconfig_vy_callback(PIDVYConfig& config, uint32_t level);
    void reconfig_vz_callback(PIDVZConfig& config, uint32_t level);
    void reconfig_yaw_callback(PIDYawConfig& config, uint32_t level);
    void reconfig_comm_callback(CommConfig& config, uint32_t level);
    void reconfig_ctrl_callback(CtrlConfig& config, uint32_t level);
};

} // end namespace mav

#endif // FLYER_INTERFACE_FLYER_INTERFACE_H
