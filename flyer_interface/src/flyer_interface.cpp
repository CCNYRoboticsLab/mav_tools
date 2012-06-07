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

#include "flyer_interface/flyer_interface.h"

namespace mav
{

FlyerInterface::FlyerInterface(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  cfg_ctrl_srv_(ros::NodeHandle(nh_private_, "ctrl")),
  cfg_x_srv_(ros::NodeHandle(nh_private_, "ctrl/pid_x")),
  cfg_y_srv_(ros::NodeHandle(nh_private_, "ctrl/pid_y")),
  cfg_z_srv_(ros::NodeHandle(nh_private_, "ctrl/pid_z")),
  cfg_vx_srv_(ros::NodeHandle(nh_private_, "ctrl/pid_vx")),
  cfg_vy_srv_(ros::NodeHandle(nh_private_, "ctrl/pid_vy")),
  cfg_vz_srv_(ros::NodeHandle(nh_private_, "ctrl/pid_vz")),
  cfg_yaw_srv_(ros::NodeHandle(nh_private_, "ctrl/pid_yaw")),
  cfg_comm_srv_(ros::NodeHandle(nh_private_, "comm")),
  connected_(false)
{
  ROS_INFO("Starting FlyerInterface"); 

  ros::NodeHandle nh_mav (nh_, "mav");

  // **** initialize vaiables

  cmd_roll_     = 0.0;
  cmd_pitch_    = 0.0;
  cmd_yaw_rate_ = 0.0;
  cmd_thrust_   = 0.0;

  cpu_load_index_ = 0;

  cpu_loads_.resize(50);

  // **** get parameters

  initializeParams();

  // **** dynamic reconfigure services

  CfgZServer::CallbackType f1 = boost::bind(&FlyerInterface::reconfig_z_callback, this, _1, _2);
  cfg_z_srv_.setCallback(f1);

  CfgXServer::CallbackType f2 = boost::bind(&FlyerInterface::reconfig_x_callback, this, _1, _2);
  cfg_x_srv_.setCallback(f2);

  CfgYServer::CallbackType f3 = boost::bind(&FlyerInterface::reconfig_y_callback, this, _1, _2);
  cfg_y_srv_.setCallback(f3);

  CfgYawServer::CallbackType f4 = boost::bind(&FlyerInterface::reconfig_yaw_callback, this, _1, _2);
  cfg_yaw_srv_.setCallback(f4);

  CfgCommServer::CallbackType f5 = boost::bind(&FlyerInterface::reconfig_comm_callback, this, _1, _2);
  cfg_comm_srv_.setCallback(f5);

  CfgCtrlServer::CallbackType f6 = boost::bind(&FlyerInterface::reconfig_ctrl_callback, this, _1, _2);
  cfg_ctrl_srv_.setCallback(f6);

  CfgVXServer::CallbackType f7 = boost::bind(&FlyerInterface::reconfig_vx_callback, this, _1, _2);
  cfg_vx_srv_.setCallback(f7);

  CfgVYServer::CallbackType f8 = boost::bind(&FlyerInterface::reconfig_vy_callback, this, _1, _2);
  cfg_vy_srv_.setCallback(f8);

  CfgVZServer::CallbackType f9 = boost::bind(&FlyerInterface::reconfig_vz_callback, this, _1, _2);
  cfg_vz_srv_.setCallback(f9);

  // **** connect

  connected_ = comm_.connect(serial_port_rx_, serial_port_tx_, baudrate_);

  if (!connected_)
  {
    ROS_ERROR("unable to connect");
    return;
  }

  // **** configure

  sendCommConfig();
  sendKFConfig(true); // configure and reset to 0 state
  sendPIDConfig();
  sendCtrlConfig();

  // *** register publishers

  if (publish_pose_)
  { 
    pose_publisher_ = nh_mav.advertise<geometry_msgs::PoseStamped>(
      "pose", 10);
  }
  vel_publisher_ = nh_mav.advertise<geometry_msgs::TwistStamped>(
    "vel", 10);
  imu_publisher_ = nh_mav.advertise<sensor_msgs::Imu>(
    "imu", 10);
  flight_state_publisher_ = nh_mav.advertise<std_msgs::UInt8>(
    "status/flight_state", 1);
  battery_voltage_publisher_ = nh_mav.advertise<std_msgs::Float32>(
    "status/battery_voltage", 1);
  cpu_load_publisher_ = nh_mav.advertise<std_msgs::Float32>(
    "status/cpu_load", 1);
  cpu_load_avg_publisher_ = nh_mav.advertise<std_msgs::Float32>(
    "status/cpu_load_avg", 1);

  if (publish_debug_)
  { 
    debug_cmd_roll_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/cmd/roll", 1);
    debug_cmd_pitch_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/cmd/pitch", 1);
    debug_cmd_yaw_rate_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/cmd/yaw_rate", 1);
    debug_cmd_thrust_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/cmd/thrust", 1);

    debug_cmd_yaw_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/cmd/yaw", 1);

    debug_pid_x_i_term_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/x_i_term", 1);
    debug_pid_y_i_term_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/y_i_term", 1);
    debug_pid_yaw_i_term_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/yaw_i_term", 1);
    debug_pid_z_i_term_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/z_i_term", 1);

    debug_roll_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/roll", 1);
    debug_pitch_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/pitch", 1);
    debug_yaw_publisher_ = nh_mav.advertise<std_msgs::Float32>(
      "debug/yaw", 1);

    debug_err_x_bf_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/err_x_bf", 1);
    debug_err_y_bf_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/err_y_bf", 1);

    debug_err_vx_bf_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/err_vx_bf", 1);
    debug_err_vy_bf_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/err_vy_bf", 1);

    debug_ax_bf_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/acc_x_bf", 1);
    debug_ay_bf_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/acc_y_bf", 1);
    debug_az_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/dvz", 1);

    debug_vx_bf_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/vx_bf", 1);
    debug_vy_bf_publisher_= nh_mav.advertise<std_msgs::Float32>(
      "debug/pid/vy_bf", 1);

    debug_imu_wf_publisher_ = nh_mav.advertise<sensor_msgs::Imu>(
      "debug/imu_wf", 1);
  }

  // **** register callbacks for packets from serial port

  comm_.registerCallback(MAV_IMU_PKT_ID,          &FlyerInterface::processImuData, this);
  comm_.registerCallback(MAV_POSE_PKT_ID,         &FlyerInterface::processPoseData, this);
  comm_.registerCallback(MAV_RCDATA_PKT_ID,       &FlyerInterface::processRCData, this);
  comm_.registerCallback(MAV_FLIGHT_STATE_PKT_ID, &FlyerInterface::processFlightStateData, this);
  comm_.registerCallback(MAV_TIMESYNC_PKT_ID,     &FlyerInterface::processTimeSyncData, this);
  comm_.registerCallback(MAV_STATUS_PKT_ID,       &FlyerInterface::processStatusData, this);

  if (publish_debug_)
    comm_.registerCallback(MAV_CTRL_DEBUG_PKT_ID,   &FlyerInterface::processCtrlDebugData, this);

  // **** register subscribers

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;

  laser_pose_subscriber_.reset(new PoseStampedSubscriber(
    nh_mav, "laser_pose_f", queue_size));
  laser_vel_subscriber_.reset(new TwistStampedSubscriber(
    nh_mav, "laser_vel_f", queue_size));

  sync_.reset(new Synchronizer(
    SyncPolicy(queue_size), *laser_pose_subscriber_, *laser_vel_subscriber_));
  sync_->registerCallback(boost::bind(&FlyerInterface::laserCallback, this, _1, _2));

  height_subscriber_ = nh_mav.subscribe(
    "laser_height_f", 10, &FlyerInterface::heightCallback, this);

  cmd_roll_subscriber_ = nh_mav.subscribe(
    "cmd/roll", 10, &FlyerInterface::cmdRollCallback, this);
  cmd_pitch_subscriber_ = nh_mav.subscribe(
    "cmd/pitch", 10, &FlyerInterface::cmdPitchCallback, this); 
  cmd_yaw_rate_subscriber_ = nh_mav.subscribe(
    "cmd/yaw_rate", 10, &FlyerInterface::cmdYawRateCallback, this);
  cmd_thrust_subscriber_ = nh_mav.subscribe(
    "cmd/thrust", 10, &FlyerInterface::cmdThrustCallback, this);
  cmd_pose_subscriber_ = nh_mav.subscribe(
    "cmd/pose", 10, &FlyerInterface::cmdPoseCallback, this);
  cmd_vel_subscriber_ = nh_mav.subscribe(
    "cmd/vel", 10, &FlyerInterface::cmdVelCallback, this);

  // **** timers

  cmd_timer_ = nh_private_.createTimer(
    ros::Duration(0.05), &FlyerInterface::cmdTimerCallback, this);

  // **** services

  advance_state_srv_ = nh_mav.advertiseService(
    "advanceState", &FlyerInterface::advanceState, this);
  retreat_state_srv_ = nh_mav.advertiseService(
    "retreatState", &FlyerInterface::retreatState, this);
  estop_srv_ = nh_mav.advertiseService(
    "estop", &FlyerInterface::estop, this);

  toggle_engage_srv_ = nh_mav.advertiseService(
    "toggleEngage", &FlyerInterface::toggleEngage, this);
  takeoff_srv_ = nh_mav.advertiseService(
    "takeoff", &FlyerInterface::takeoff, this);
  land_srv_ = nh_mav.advertiseService(
    "land", &FlyerInterface::land, this);

  get_flight_state_srv_ = nh_mav.advertiseService(
    "getFlightState", &FlyerInterface::getFlightState, this);

  set_ctrl_type_srv_ = nh_mav.advertiseService(
    "setCtrlType", &FlyerInterface::setCtrlType, this);
}

FlyerInterface::~FlyerInterface()
{
  ROS_INFO("Destorying Flyer Interface");
}

void FlyerInterface::initializeParams()
{
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/base_link";
  if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = false;
  if (!nh_private_.getParam ("publish_debug", publish_debug_))
    publish_debug_ = false;

  // **** communication parameters

  if (!nh_private_.getParam ("comm/serial_port_rx", serial_port_rx_))
    serial_port_rx_ = "/dev/ttyUSB0";
  if (!nh_private_.getParam ("cogmm/serial_port_tx", serial_port_tx_))
    serial_port_tx_ = "/dev/ttyUSB0";
  if (!nh_private_.getParam ("comm/baudrate", baudrate_))
    baudrate_ = MAV_DEFAULT_BAUDRATE;

  // TODO - use packet and dynamic reconfig for these

  if (!nh_private_.getParam ("enable_kf_x", enable_kf_x_))
    enable_kf_x_ = "true";
  if (!nh_private_.getParam ("enable_kf_y", enable_kf_y_))
    enable_kf_y_ = "true";
  if (!nh_private_.getParam ("enable_kf_z", enable_kf_z_))
    enable_kf_z_ = "true";
  if (!nh_private_.getParam ("enable_kf_yaw", enable_kf_yaw_))
    enable_kf_yaw_ = "true";

  if (!nh_private_.getParam ("q_x", q_x_))
    q_x_ = 0.083;
  if (!nh_private_.getParam ("q_y", q_y_))
    q_y_ = 0.083;
  if (!nh_private_.getParam ("q_z", q_z_))
    q_z_ = 0.7;
  if (!nh_private_.getParam ("q_yaw", q_yaw_))
    q_yaw_ = 0.083;

  if (!nh_private_.getParam ("r_x", r_x_))
    r_x_ = 2.0;
  if (!nh_private_.getParam ("r_y", r_y_))
    r_y_ = 2.0;
  if (!nh_private_.getParam ("r_z", r_z_))
    r_z_ = 5.0;
  if (!nh_private_.getParam ("r_yaw", r_yaw_))
    r_yaw_ = 2.0;

  if (!nh_private_.getParam ("r_vx", r_vx_))
    r_vx_ = 10.0;
  if (!nh_private_.getParam ("r_vy", r_vy_))
    r_vy_ = 10.0;
  if (!nh_private_.getParam ("r_vz", r_vz_))
    r_vz_ = 10.0;
  if (!nh_private_.getParam ("r_vz_p", r_vz_p_))
    r_vz_p_ = 10.0;

  // **** Control mode

  int ctrl_mode_roll, ctrl_mode_pitch, ctrl_mode_yaw_rate, ctrl_mode_thrust;

  if (!nh_private_.getParam ("ctrl/ctrl_mode_roll", ctrl_mode_roll))
    ctrl_mode_roll = MAV_CTRL_MODE_DIRECT;
  if (!nh_private_.getParam ("ctrl/ctrl_mode_pitch", ctrl_mode_pitch))
    ctrl_mode_pitch = MAV_CTRL_MODE_DIRECT;
  if (!nh_private_.getParam ("ctrl/ctrl_mode_yaw_rate", ctrl_mode_yaw_rate))
    ctrl_mode_yaw_rate = MAV_CTRL_MODE_DIRECT;
  if (!nh_private_.getParam ("ctrl/ctrl_mode_thrust", ctrl_mode_thrust))
    ctrl_mode_thrust = MAV_CTRL_MODE_DIRECT;

  ctrl_cfg_packet_.ctrl_mode_roll     = ctrl_mode_roll;
  ctrl_cfg_packet_.ctrl_mode_pitch    = ctrl_mode_pitch;
  ctrl_cfg_packet_.ctrl_mode_yaw_rate = ctrl_mode_yaw_rate;
  ctrl_cfg_packet_.ctrl_mode_thrust   = ctrl_mode_thrust;
}

void FlyerInterface::laserCallback(
  const PoseStamped::ConstPtr  pose_msg,
  const TwistStamped::ConstPtr twist_msg)
{
  MAV_POSE2D_PKT packet;

  packet.x = pose_msg->pose.position.x;
  packet.y = pose_msg->pose.position.y;

  packet.vx = twist_msg->twist.linear.x;
  packet.vy = twist_msg->twist.linear.y;

  packet.yaw = tf::getYaw(pose_msg->pose.orientation);
  normalizeSIAngle2Pi(&packet.yaw);

  ROS_DEBUG("Sending MAV_POSE2D_PKT packet");
  comm_.sendPacket(MAV_POSE2D_PKT_ID, packet);
}

void FlyerInterface::cmdPoseCallback(const geometry_msgs::PoseStamped::ConstPtr cmd_pose_msg)
{
  // TODO
  ros::Duration tf_tol(5.0);  

  // **** transform the message to the correct frame
  std::string frame = tf_listener_.resolve(cmd_pose_msg->header.frame_id);
  geometry_msgs::PoseStamped cmd_pose_fixed_frame;

  if (frame != fixed_frame_)
  {
    ROS_INFO_ONCE("cmd_pose messages does not match the fixed frame (%s, %s), transforming",
      frame.c_str(), fixed_frame_.c_str());
    
    try
    {
      tf_listener_.waitForTransform(
        fixed_frame_, cmd_pose_msg->header.frame_id, cmd_pose_msg->header.stamp, tf_tol);

      tf_listener_.transformPose(fixed_frame_, *cmd_pose_msg, cmd_pose_fixed_frame);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not transform goal pose %s", ex.what());
      return;
    }
  }
  else
  {
    cmd_pose_fixed_frame = *cmd_pose_msg;
  }

  MAV_DES_POSE_PKT des_pose_pkt;

  des_pose_pkt.x = cmd_pose_fixed_frame.pose.position.x;
  des_pose_pkt.y = cmd_pose_fixed_frame.pose.position.y;
  des_pose_pkt.z = cmd_pose_fixed_frame.pose.position.z;

  float cmd_yaw = tf::getYaw(cmd_pose_fixed_frame.pose.orientation);
  normalizeSIAnglePi(&cmd_yaw);

  des_pose_pkt.yaw = cmd_yaw;

  ROS_DEBUG("Sending MAV_DES_POSE_PKT packet");
  comm_.sendPacket(MAV_DES_POSE_PKT_ID, des_pose_pkt);

  // **** publish debug cmd yaw

  std_msgs::Float32 cmd_yaw_msg;
  cmd_yaw_msg.data  = cmd_yaw;
  debug_cmd_yaw_publisher_.publish(cmd_yaw_msg);
}

void FlyerInterface::cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr cmd_vel_msg)   
{
  MAV_DES_VEL_PKT des_vel_pkt;

  des_vel_pkt.vx       = cmd_vel_msg->twist.linear.x;
  des_vel_pkt.vy       = cmd_vel_msg->twist.linear.y;
  des_vel_pkt.vz       = cmd_vel_msg->twist.linear.z;
  des_vel_pkt.yaw_rate = cmd_vel_msg->twist.angular.z;

  ROS_DEBUG("Sending MAV_DES_VEL_PKT packet");
  comm_.sendPacket(MAV_DES_VEL_PKT_ID, des_vel_pkt);
}

void FlyerInterface::heightCallback(const mav_msgs::Height::ConstPtr height_msg)
{
  MAV_HEIGHT_PKT packet;

  packet.z  = height_msg->height;
  packet.vz = height_msg->climb;

  ROS_DEBUG("Sending MAV_HEIGHT_PKT packet");
  comm_.sendPacket(MAV_HEIGHT_PKT_ID, packet);
}

void FlyerInterface::cmdTimerCallback(const ros::TimerEvent& event)
{
  MAV_CTRL_INPUT_PKT ctrl_input_packet;

  ctrl_input_packet.cmd_roll     = cmd_roll_;
  ctrl_input_packet.cmd_pitch    = cmd_pitch_;
  ctrl_input_packet.cmd_yaw_rate = cmd_yaw_rate_;
  ctrl_input_packet.cmd_thrust   = cmd_thrust_;

  ROS_DEBUG("Sending MAV_CTRL_INPUT_PKT packet");
  comm_.sendPacket(MAV_CTRL_INPUT_PKT_ID, ctrl_input_packet);
}

void FlyerInterface::sendKFConfig(bool reset)
{ 
  MAV_KF_CFG_PKT packet;

  uint8_t mask = 0x00;

  if (enable_kf_x_)   mask |= (1 << MAV_KF_X_BIT);
  if (enable_kf_y_)   mask |= (1 << MAV_KF_Y_BIT);
  if (enable_kf_z_)   mask |= (1 << MAV_KF_Z_BIT);
  if (enable_kf_yaw_) mask |= (1 << MAV_KF_YAW_BIT);

  if (reset)          mask |= (1 << MAV_KF_RESET_BIT);

  packet.enable_mask = mask;

  packet.Q_x   = q_x_;
  packet.Q_y   = q_y_;
  packet.Q_z   = q_z_;
  packet.Q_yaw = q_yaw_;

  packet.R_x   = r_x_;
  packet.R_y   = r_y_;
  packet.R_z   = r_z_;
  packet.R_yaw = r_yaw_;

  packet.R_vx   = r_vx_;
  packet.R_vy   = r_vy_;
  packet.R_vz   = r_vz_;
  packet.R_vz_p = r_vz_p_;

  ROS_INFO("Sending MAV_KF_CFG_PKT packet");
  comm_.sendPacketAck(MAV_KF_CFG_PKT_ID, packet);
}

void FlyerInterface::sendPIDConfig()
{ 
  ROS_INFO("Sending MAV_PID_CFG_PKT packet");
  comm_.sendPacketAck(MAV_PID_CFG_PKT_ID, pid_cfg_packet_);
}

void FlyerInterface::sendCommConfig()
{ 
  ROS_INFO("Sending MAV_TX_FREQ_CFG_PKT packet");
  comm_.sendPacketAck(MAV_TX_FREQ_CFG_PKT_ID, tx_freq_cfg_packet_);
}

void FlyerInterface::sendCtrlConfig()
{ 
  ROS_INFO("Sending MAV_CTRL_CFG_PKT packet");
  comm_.sendPacketAck(MAV_CTRL_CFG_PKT_ID, ctrl_cfg_packet_);
}

void FlyerInterface::processCtrlDebugData(uint8_t * buf, uint32_t bufLength)
{
  ROS_DEBUG("MAV_CTRL_DEBUG_PKT Packet Received");

  MAV_CTRL_DEBUG_PKT * ctrl_debug_pkt = (MAV_CTRL_DEBUG_PKT *)buf;

  // **** control commands

  std_msgs::Float32 cmd_roll_msg;
  std_msgs::Float32 cmd_pitch_msg;
  std_msgs::Float32 cmd_yaw_rate_msg;
  std_msgs::Float32 cmd_thrust_msg;

  cmd_roll_msg.data     = ctrl_debug_pkt->cmd_roll;
  cmd_pitch_msg.data    = ctrl_debug_pkt->cmd_pitch;
  cmd_yaw_rate_msg.data = ctrl_debug_pkt->cmd_yaw_rate;
  cmd_thrust_msg.data   = ctrl_debug_pkt->cmd_thrust;

  debug_cmd_roll_publisher_.publish(cmd_roll_msg);
  debug_cmd_pitch_publisher_.publish(cmd_pitch_msg);
  debug_cmd_yaw_rate_publisher_.publish(cmd_yaw_rate_msg);
  debug_cmd_thrust_publisher_.publish(cmd_thrust_msg);

  // **** PID i-terms

  std_msgs::Float32 pid_x_i_term_msg;
  std_msgs::Float32 pid_y_i_term_msg;
  std_msgs::Float32 pid_yaw_i_term_msg;
  std_msgs::Float32 pid_z_i_term_msg;

  pid_x_i_term_msg.data   = ctrl_debug_pkt->pid_x_i_term;
  pid_y_i_term_msg.data   = ctrl_debug_pkt->pid_y_i_term;
  pid_yaw_i_term_msg.data = ctrl_debug_pkt->pid_yaw_i_term;
  pid_z_i_term_msg.data   = ctrl_debug_pkt->pid_z_i_term;

  debug_pid_x_i_term_publisher_.publish(pid_x_i_term_msg);
  debug_pid_y_i_term_publisher_.publish(pid_y_i_term_msg);
  debug_pid_yaw_i_term_publisher_.publish(pid_yaw_i_term_msg);
  debug_pid_z_i_term_publisher_.publish(pid_z_i_term_msg);

  // **** publish world-frame accelerations used in KF

  sensor_msgs::Imu imu_wf;
  imu_wf.header.stamp = ros::Time::now();
  imu_wf.linear_acceleration.x = ctrl_debug_pkt->acc_x_wf;
  imu_wf.linear_acceleration.y = ctrl_debug_pkt->acc_y_wf;
  imu_wf.linear_acceleration.z = ctrl_debug_pkt->acc_z_wf;

  debug_imu_wf_publisher_.publish(imu_wf);

  // **** publish body-frame pid error for position control

  std_msgs::Float32 err_x_bf_msg;
  err_x_bf_msg.data = ctrl_debug_pkt->pid_error_x_bf;
  debug_err_x_bf_publisher_.publish(err_x_bf_msg);

  std_msgs::Float32 err_y_bf_msg;
  err_y_bf_msg.data = ctrl_debug_pkt->pid_error_y_bf;
  debug_err_y_bf_publisher_.publish(err_y_bf_msg);

  // **** publish body-frame pid error for velocity control

  std_msgs::Float32 err_vx_bf_msg;
  err_vx_bf_msg.data = ctrl_debug_pkt->pid_error_vx_bf;
  debug_err_vx_bf_publisher_.publish(err_vx_bf_msg);

  std_msgs::Float32 err_vy_bf_msg;
  err_vy_bf_msg.data = ctrl_debug_pkt->pid_error_vy_bf;
  debug_err_vy_bf_publisher_.publish(err_vy_bf_msg);

  // **** publish body-frame acc for velocity control

  std_msgs::Float32 ax_bf_msg;
  ax_bf_msg.data = ctrl_debug_pkt->ax_bf;
  debug_ax_bf_publisher_.publish(ax_bf_msg);

  std_msgs::Float32 ay_bf_msg;
  ay_bf_msg.data = ctrl_debug_pkt->ay_bf;
  debug_ay_bf_publisher_.publish(ay_bf_msg);

  std_msgs::Float32 az_msg;
  az_msg.data = ctrl_debug_pkt->az;
  debug_az_publisher_.publish(az_msg);

  // **** publish body-frame velocities

  std_msgs::Float32 vx_bf_msg;
  vx_bf_msg.data = ctrl_debug_pkt->vel_x_bf;
  debug_vx_bf_publisher_.publish(vx_bf_msg);

  std_msgs::Float32 vy_bf_msg;
  vy_bf_msg.data = ctrl_debug_pkt->vel_y_bf;
  debug_vy_bf_publisher_.publish(vy_bf_msg);

  // **** Control modes

  ROS_DEBUG("Ctrl Mode: %d %d %d %d", ctrl_debug_pkt->ctrl_mode_roll,
                                      ctrl_debug_pkt->ctrl_mode_pitch,
                                      ctrl_debug_pkt->ctrl_mode_yaw_rate,
                                      ctrl_debug_pkt->ctrl_mode_thrust);

/*
  ROS_DEBUG("LL Angles: %d %d %d", ctrl_debug_pkt->roll,
                                   ctrl_debug_pkt->pitch,
                                   ctrl_debug_pkt->yaw);
*/
}

void FlyerInterface::processStatusData(uint8_t * buf, uint32_t bufLength)
{
  ROS_DEBUG("MAV_STATUS_PKT Packet Received");

  MAV_STATUS_PKT * status_pkt = (MAV_STATUS_PKT *)buf;
 
  std_msgs::Float32 battery_voltage_msg;
  battery_voltage_msg.data = status_pkt->battery_voltage / 1000.0;
  
  battery_voltage_publisher_.publish(battery_voltage_msg);

  std_msgs::Float32 cpu_load_msg;
  cpu_load_msg.data = status_pkt->cpu_load;
  
  cpu_load_publisher_.publish(cpu_load_msg);

  // average load

  cpu_loads_[cpu_load_index_] = (double)status_pkt->cpu_load;
  cpu_load_index_++;
  if (cpu_load_index_ >= 50) cpu_load_index_ = 0;

  double sum = 0.0;
  for (int i = 0; i < 50; ++i)
    sum += cpu_loads_[i];

  double avg = sum / 50.0;

  std_msgs::Float32 cpu_load_avg_msg;
  cpu_load_avg_msg.data = avg;
  cpu_load_avg_publisher_.publish(cpu_load_avg_msg);
}

void FlyerInterface::processPoseData(uint8_t * buf, uint32_t bufLength)
{
  ROS_DEBUG("MAV_POSE_PKT Packet Received"); 
  ros::Time time = ros::Time::now();

  MAV_POSE_PKT* mav_pose_pkt = (MAV_POSE_PKT*)buf;

  if (publish_pose_)
  {
    // **** create ROS pose message from packet

    geometry_msgs::PoseStamped::Ptr pose_msg;
    pose_msg = boost::make_shared<geometry_msgs::PoseStamped>();

    pose_msg->header.frame_id = fixed_frame_;
    pose_msg->header.stamp = time;

    pose_msg->pose.position.x = mav_pose_pkt->x;
    pose_msg->pose.position.y = mav_pose_pkt->y;
    pose_msg->pose.position.z = mav_pose_pkt->z;

    tf::Quaternion q = tf::createQuaternionFromRPY(
      mav_pose_pkt->roll, mav_pose_pkt->pitch, mav_pose_pkt->yaw);
    tf::quaternionTFToMsg(q, pose_msg->pose.orientation);

    // **** publish the pose message

    pose_publisher_.publish(pose_msg);

    // **** publish odometry transform

    tf::Stamped<tf::Pose> f2b;
    tf::poseStampedMsgToTF(*pose_msg, f2b);

    tf::StampedTransform f2b_tf(f2b, time, fixed_frame_, base_frame_);
    tf_broadcaster_.sendTransform(f2b_tf);
  }

  // **** create a ROS twist message from packet

  geometry_msgs::TwistStamped::Ptr twist_msg;
  twist_msg = boost::make_shared<geometry_msgs::TwistStamped>();

  twist_msg->header.frame_id = fixed_frame_;
  twist_msg->header.stamp = time;
  
  twist_msg->twist.linear.x = mav_pose_pkt->vx;
  twist_msg->twist.linear.y = mav_pose_pkt->vy;
  twist_msg->twist.linear.z = mav_pose_pkt->vz;

  // **** publish the velocity message

  vel_publisher_.publish(twist_msg);

  // **** publish debug angles

  std_msgs::Float32 roll_msg, pitch_msg, yaw_msg;
  
  roll_msg.data  = mav_pose_pkt->roll;
  pitch_msg.data = mav_pose_pkt->pitch;
  yaw_msg.data   = mav_pose_pkt->yaw;

  if (yaw_msg.data >   M_PI) yaw_msg.data -= 2.0 * M_PI;
  if (yaw_msg.data <= -M_PI) yaw_msg.data += 2.0 * M_PI;

  debug_roll_publisher_.publish(roll_msg);
  debug_pitch_publisher_.publish(pitch_msg);
  debug_yaw_publisher_.publish(yaw_msg);
}

void FlyerInterface::processRCData(uint8_t * buf, uint32_t bufLength)
{

}

void FlyerInterface::processImuData(uint8_t * buf, uint32_t bufLength)
{
  ROS_DEBUG("IMU Packet Received"); 
  MAV_IMU_PKT * data = (MAV_IMU_PKT *)buf;

  // create imu message

  sensor_msgs::Imu::Ptr imu_msg;
  imu_msg = boost::make_shared<sensor_msgs::Imu>();

  imu_msg->header.frame_id = fixed_frame_;
  imu_msg->header.stamp    = ros::Time::now();
  
  // copy over angles

  double roll  = data->roll;
  double pitch = data->pitch;
  double yaw   = data->yaw;

  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q, imu_msg->orientation);

  // copy over angular velocities

  imu_msg->angular_velocity.x = data->roll_rate;
  imu_msg->angular_velocity.y = data->pitch_rate;  
  imu_msg->angular_velocity.z = data->yaw_rate;

  // copy over accerlerations

  imu_msg->linear_acceleration.x = data->acc_x;
  imu_msg->linear_acceleration.y = data->acc_y;
  imu_msg->linear_acceleration.z = data->acc_z;

  imu_publisher_.publish(imu_msg);
}

void FlyerInterface::processFlightStateData(uint8_t * buf, uint32_t bufLength)
{  
  ROS_DEBUG("MAV_FLIGHT_STATE Packet Received"); 
  MAV_FLIGHT_STATE_PKT * flight_state_pkt = (MAV_FLIGHT_STATE_PKT *)buf;
 
  boost::mutex::scoped_lock lock(state_mutex_); 
  flight_state_ = flight_state_pkt->state;

  // publish the message
  std_msgs::UInt8 flight_state_msg;
  flight_state_msg.data = flight_state_;

  flight_state_publisher_.publish(flight_state_msg);
}

bool FlyerInterface::advanceState(mav_srvs::AdvanceState::Request  &req,
                                  mav_srvs::AdvanceState::Response &res)
{
  MAV_FLIGHT_ACTION_PKT packet;

  packet.action = MAV_ACTION_ADVANCE_STATE;

  ROS_INFO("Sending MAV_ACTION_PKT: Advance State");
  return comm_.sendPacket(MAV_FLIGHT_ACTION_PKT_ID, packet);
}

bool FlyerInterface::retreatState(mav_srvs::AdvanceState::Request  &req,
                                  mav_srvs::AdvanceState::Response &res)
{
  MAV_FLIGHT_ACTION_PKT packet;

  packet.action = MAV_ACTION_RETREAT_STATE;

  ROS_INFO("Sending MAV_ACTION_PKT: Retreat State");
  return comm_.sendPacket(MAV_FLIGHT_ACTION_PKT_ID, packet);
}

bool FlyerInterface::estop(mav_srvs::ESTOP::Request  &req,
                           mav_srvs::ESTOP::Response &res)
{
  MAV_FLIGHT_ACTION_PKT packet;

  packet.action = MAV_ACTION_ESTOP;

  ROS_WARN("Sending MAV_ACTION_PKT: ESTOP");
  return comm_.sendPacket(MAV_FLIGHT_ACTION_PKT_ID, packet);
}

bool FlyerInterface::getFlightState(mav_srvs::GetFlightState::Request  &req,
                                    mav_srvs::GetFlightState::Response &res)
{
  boost::mutex::scoped_lock lock(state_mutex_); 
  res.state = flight_state_;

  return true;
}

bool FlyerInterface::setCtrlType(mav_srvs::SetCtrlType::Request  &req,
                                 mav_srvs::SetCtrlType::Response &res)
{
  if (req.ctrl_type == mav::PositionCtrl)
  {
    ROS_INFO("Ctrl type changed to POSITION:");

    ctrl_cfg_packet_.ctrl_mode_roll     = MAV_CTRL_MODE_POSITION;
    ctrl_cfg_packet_.ctrl_mode_pitch    = MAV_CTRL_MODE_POSITION;
    ctrl_cfg_packet_.ctrl_mode_yaw_rate = MAV_CTRL_MODE_POSITION;
    ctrl_cfg_packet_.ctrl_mode_thrust   = MAV_CTRL_MODE_POSITION;
  }
  else if (req.ctrl_type == mav::VelocityCtrl)
  {
    ROS_INFO("Ctrl type changed to VELOCITY:");

    ctrl_cfg_packet_.ctrl_mode_roll     = MAV_CTRL_MODE_VELOCITY;
    ctrl_cfg_packet_.ctrl_mode_pitch    = MAV_CTRL_MODE_VELOCITY;  
    ctrl_cfg_packet_.ctrl_mode_yaw_rate = MAV_CTRL_MODE_VELOCITY;
    ctrl_cfg_packet_.ctrl_mode_thrust   = MAV_CTRL_MODE_POSITION;  
  }
  else
  {
    ROS_WARN("Unknown ctrl mode requested");
    return false;
  }

  ROS_INFO("\tRoll: %d",     ctrl_cfg_packet_.ctrl_mode_roll);
  ROS_INFO("\tPitch: %d",    ctrl_cfg_packet_.ctrl_mode_pitch);
  ROS_INFO("\tYaw rate: %d", ctrl_cfg_packet_.ctrl_mode_yaw_rate);
  ROS_INFO("\tThrust: %d",   ctrl_cfg_packet_.ctrl_mode_thrust);

  if (connected_) sendCtrlConfig();

  return true;
}

bool FlyerInterface::toggleEngage(mav_srvs::ToggleEngage::Request  &req,
                                  mav_srvs::ToggleEngage::Response &res)
{
  MAV_FLIGHT_ACTION_PKT packet;

  packet.action = MAV_ACTION_TOGGLE_ENGAGE;

  ROS_INFO("Sending MAV_ACTION_PKT: Toggle engage");
  return comm_.sendPacket(MAV_FLIGHT_ACTION_PKT_ID, packet);
}

bool FlyerInterface::land(mav_srvs::Land::Request  &req,
                          mav_srvs::Land::Response &res)
{
  MAV_FLIGHT_ACTION_PKT packet;

  packet.action = MAV_ACTION_LAND;

  ROS_INFO("Sending MAV_ACTION_PKT: Land");
  return comm_.sendPacket(MAV_FLIGHT_ACTION_PKT_ID, packet);
}

bool FlyerInterface::takeoff(mav_srvs::Takeoff::Request  &req,
                             mav_srvs::Takeoff::Response &res)
{
  MAV_FLIGHT_ACTION_PKT packet;

  packet.action = MAV_ACTION_TAKEOFF;

  ROS_INFO("Sending MAV_ACTION_PKT: Takeoff");
  return comm_.sendPacket(MAV_FLIGHT_ACTION_PKT_ID, packet);
}

void FlyerInterface::processTimeSyncData(uint8_t * buf, uint32_t bufLength)
{
  ROS_INFO("MAV_TIMESYNC_PKT received");

  static bool synced_once = false;
  MAV_TIMESYNC_PKT data = *(MAV_TIMESYNC_PKT*)buf;

  if (!synced_once)
  {
    data.ts1 = 1e12; // cause a huge offset to force AP to sync
    synced_once = true;
    ROS_INFO("Forced sync");
  }
  else
  {
    data.ts1 = (uint64_t)(ros::Time::now().toSec() * 1.0e6);
    ROS_INFO("Synced");
  }

  comm_.sendPacket(MAV_TIMESYNC_PKT_ID, data);

  // warn if imu time is too far away from pc time. At low baudrates, the IMU will take longer to sync.
  //ROS_WARN_STREAM_COND(std::abs(status_.timesync_offset) > 0.002, "imu time is off by "<<status_.timesync_offset * 1e3 <<" ms");
}

void FlyerInterface::cmdRollCallback(const mav_msgs::Roll::ConstPtr roll_msg)
{
  cmd_roll_ = roll_msg->roll;
}

void FlyerInterface::cmdPitchCallback(const mav_msgs::Pitch::ConstPtr pitch_msg)
{
  cmd_pitch_ = pitch_msg->pitch;
}

void FlyerInterface::cmdYawRateCallback(const mav_msgs::YawRate::ConstPtr yaw_rate_msg)
{
  cmd_yaw_rate_ = yaw_rate_msg->yaw_rate;
}

void FlyerInterface::cmdThrustCallback(const mav_msgs::Thrust::ConstPtr thrust_msg)
{
  cmd_thrust_ = thrust_msg->thrust;
}

void FlyerInterface::reconfig_x_callback(PIDXConfig& config, uint32_t level)
{
  pid_cfg_packet_.k_p_x        = config.k_p;
  pid_cfg_packet_.k_i_x        = config.k_i;
  pid_cfg_packet_.k_d_x        = config.k_d;
  pid_cfg_packet_.k_d2_x       = config.k_d2;
  pid_cfg_packet_.d_base_x     = config.d_base;
  pid_cfg_packet_.bias_x       = config.bias;
  pid_cfg_packet_.max_i_x      = config.max_i;
  pid_cfg_packet_.max_err_x    = config.max_err;

  if (connected_) sendPIDConfig();
}

void FlyerInterface::reconfig_y_callback(PIDYConfig& config, uint32_t level)
{
  pid_cfg_packet_.k_p_y        = config.k_p;
  pid_cfg_packet_.k_i_y        = config.k_i;
  pid_cfg_packet_.k_d_y        = config.k_d;
  pid_cfg_packet_.k_d2_y       = config.k_d2;
  pid_cfg_packet_.d_base_y     = config.d_base;
  pid_cfg_packet_.bias_y       = config.bias;
  pid_cfg_packet_.max_i_y      = config.max_i;
  pid_cfg_packet_.max_err_y    = config.max_err;

  if (connected_) sendPIDConfig();
}

void FlyerInterface::reconfig_z_callback(PIDZConfig& config, uint32_t level)
{
  pid_cfg_packet_.k_p_z        = config.k_p;
  pid_cfg_packet_.k_i_z        = config.k_i;
  pid_cfg_packet_.k_d_z        = config.k_d;
  pid_cfg_packet_.bias_z       = config.bias;
  pid_cfg_packet_.max_i_z      = config.max_i;
  pid_cfg_packet_.max_err_z    = config.max_err;
  
  if (connected_) sendPIDConfig();
}

void FlyerInterface::reconfig_vx_callback(PIDVXConfig& config, uint32_t level)
{
  pid_cfg_packet_.k_p_vx        = config.k_p;
  pid_cfg_packet_.k_i_vx        = config.k_i;
  pid_cfg_packet_.k_d_vx        = config.k_d;
  pid_cfg_packet_.bias_vx       = config.bias;
  pid_cfg_packet_.max_i_vx      = config.max_i;
  pid_cfg_packet_.max_err_vx    = config.max_err;

  if (connected_) sendPIDConfig();
}

void FlyerInterface::reconfig_vy_callback(PIDVYConfig& config, uint32_t level)
{
  pid_cfg_packet_.k_p_vy        = config.k_p;
  pid_cfg_packet_.k_i_vy        = config.k_i;
  pid_cfg_packet_.k_d_vy        = config.k_d;
  pid_cfg_packet_.bias_vy       = config.bias;
  pid_cfg_packet_.max_i_vy      = config.max_i;
  pid_cfg_packet_.max_err_vy    = config.max_err;

  if (connected_) sendPIDConfig();
}

void FlyerInterface::reconfig_vz_callback(PIDVZConfig& config, uint32_t level)
{
  pid_cfg_packet_.k_p_vz        = config.k_p;
  pid_cfg_packet_.k_i_vz        = config.k_i;
  pid_cfg_packet_.k_d_vz        = config.k_d;
  pid_cfg_packet_.bias_vz       = config.bias;
  pid_cfg_packet_.max_i_vz      = config.max_i;
  pid_cfg_packet_.max_err_vz    = config.max_err;

  if (connected_) sendPIDConfig();
}


void FlyerInterface::reconfig_yaw_callback(PIDYawConfig& config, uint32_t level)
{
  pid_cfg_packet_.k_p_yaw        = config.k_p;
  pid_cfg_packet_.k_i_yaw        = config.k_i;
  pid_cfg_packet_.k_d_yaw        = config.k_d;
  pid_cfg_packet_.bias_yaw       = config.bias;
  pid_cfg_packet_.max_i_yaw      = config.max_i;
  pid_cfg_packet_.max_err_yaw    = config.max_err;

  if (connected_) sendPIDConfig();
}

void FlyerInterface::reconfig_ctrl_callback(CtrlConfig& config, uint32_t level)
{
  ctrl_cfg_packet_.cmd_roll_limit     = config.cmd_roll_limit;
  ctrl_cfg_packet_.cmd_pitch_limit    = config.cmd_pitch_limit;
  ctrl_cfg_packet_.cmd_yaw_rate_limit = config.cmd_yaw_rate_limit;
  ctrl_cfg_packet_.cmd_thrust_limit   = config.cmd_thrust_limit;

  ctrl_cfg_packet_.cmd_roll_delta_limit     = config.cmd_roll_delta_limit;
  ctrl_cfg_packet_.cmd_pitch_delta_limit    = config.cmd_pitch_delta_limit;
  ctrl_cfg_packet_.cmd_yaw_rate_delta_limit = config.cmd_yaw_rate_delta_limit;
  ctrl_cfg_packet_.cmd_thrust_delta_limit   = config.cmd_thrust_delta_limit;

  if (connected_) sendCtrlConfig();
}

void FlyerInterface::reconfig_comm_callback(CommConfig& config, uint32_t level)
{
  tx_freq_cfg_packet_.imu_period = config.imu_period;
  tx_freq_cfg_packet_.imu_phase  = config.imu_phase;

  tx_freq_cfg_packet_.pose_period = config.pose_period;
  tx_freq_cfg_packet_.pose_phase  = config.pose_phase;

  tx_freq_cfg_packet_.rcdata_period = config.rcdata_period;
  tx_freq_cfg_packet_.rcdata_phase  = config.rcdata_phase;

  tx_freq_cfg_packet_.flight_state_period = config.flight_state_period;
  tx_freq_cfg_packet_.flight_state_phase  = config.flight_state_phase;

  tx_freq_cfg_packet_.status_period = config.status_period;
  tx_freq_cfg_packet_.status_phase  = config.status_phase;

  tx_freq_cfg_packet_.ctrl_debug_period = config.debug_period;
  tx_freq_cfg_packet_.ctrl_debug_phase  = config.debug_phase;

  if (connected_) sendCommConfig();
}

} // end namespace mav
