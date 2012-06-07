#include "quad_joy_teleop/quad_joy_teleop.h"

QuadJoyTeleop::QuadJoyTeleop(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO("Starting QuadJoyTeleop"); 

  takeoff_btn_pressed_  = false;
  land_btn_pressed_     = false;
  engage_btn_pressed_   = false;
  estop_btn_pressed_    = false;
  pos_hold_btn_pressed_ = false;
  vel_hold_btn_pressed_ = false;

  last_joy_event_ = ros::Time::now();

  cmd_vel_msg_ = boost::make_shared<geometry_msgs::Twist>();
  cmd_vel_msg_->linear.x  = 0.0;
  cmd_vel_msg_->linear.y  = 0.0;
  cmd_vel_msg_->linear.z  = 0.0;
  cmd_vel_msg_->angular.x  = 0.0;
  cmd_vel_msg_->angular.y  = 0.0;
  cmd_vel_msg_->angular.z  = 0.0;

  initParams();

  ros::NodeHandle nh_mav (nh_, "mav");

  // **** publishers

  // TODO: have all cmd_joy all here?

  cmd_roll_publisher_ = nh_mav.advertise<mav_msgs::Roll>(
    "cmd/roll", 1);
  cmd_pitch_publisher_ = nh_mav.advertise<mav_msgs::Pitch>(
    "cmd/pitch", 1);
  cmd_yaw_rate_publisher_ = nh_mav.advertise<mav_msgs::YawRate>(
    "cmd/yaw_rate", 1);
  cmd_thrust_publisher_ = nh_mav.advertise<mav_msgs::Thrust>(
    "cmd/thrust", 1);
  cmd_vel_publisher_ = nh_mav.advertise<geometry_msgs::TwistStamped>(
    "cmd_joy/vel", 1);

  // **** subscribers

  joy_subscriber_ = nh_.subscribe(
    "joy", 1, &QuadJoyTeleop::joyCallback, this);

  // **** services

  estop_client_ = nh_mav.serviceClient<mav_srvs::ESTOP>(
    "estop");
  takeoff_client_ = nh_mav.serviceClient<mav_srvs::Takeoff>(
    "takeoff");
  land_client_ = nh_mav.serviceClient<mav_srvs::Land>(
    "land");
  change_des_pose_client_ = nh_mav.serviceClient<mav_srvs::ChangeDesPose>(
    "changeDesPose");
  toggle_engage_client_ = nh_mav.serviceClient<mav_srvs::ToggleEngage>(
    "toggleEngage");
  pos_hold_client_ = nh_mav.serviceClient<mav_srvs::PositionHold>(
    "positionHold");
  vel_hold_client_ = nh_mav.serviceClient<mav_srvs::VelocityHold>(
    "velocityHold");
}

QuadJoyTeleop::~QuadJoyTeleop()
{
  ROS_INFO("Destroying QuadJoyTeleop"); 
}

void QuadJoyTeleop::initParams()
{
  // for position commands
  x_step_size_ = 0.25;
  y_step_size_ = 0.25;
  z_step_size_ = 0.25;
  yaw_step_size_ = 0.7853975; // 45 degrees

  // for velocity commands
  cmd_vel_linear_scale_ = 1.0;
  cmd_vel_angular_scale_ = 1.0; // rad
  
  // for direct commands
  cmd_roll_scale_     = 0.5;       
  cmd_pitch_scale_    = 0.5;
  cmd_yaw_rate_scale_ = 1.0;   
  cmd_thrust_scale_   = 100.0;  

  estop_button_    = 999;   
  takeoff_button_  = 12;    // triangle
  land_button_     = 14;    // X
  engage_button_   = 15;    // square
  pos_hold_button_ = 11;    // R1 (right trigger)
  vel_hold_button_ = 10;    // L1 (left trigger)

  roll_axis_   = 2;
  pitch_axis_  = 3;
  yaw_axis_    = 0;
  thrust_axis_ = 1;

  vy_axis_   = 2;
  vx_axis_   = 3;
  vz_axis_   = 1;
  vyaw_axis_ = 0;
}

void QuadJoyTeleop::joyCallback (const sensor_msgs::JoyPtr& joy_msg)
{
/*
  // **** E-STOP
  if (joy_msg->buttons[estop_button_] == 1)
  {
    ROS_WARN("E-STOP button pressed!");

    mav_srvs::ESTOP estop_srv;
    estop_client_.call(estop_srv);
  }
*/
  // ****  takeoff
  if (joy_msg->buttons[takeoff_button_] == 1)
  {
    if (!takeoff_btn_pressed_)
    {
      ROS_INFO("Takeoff button pressed.");

      mav_srvs::Takeoff takeoff_srv;
      takeoff_client_.call(takeoff_srv);

      takeoff_btn_pressed_ = true;
    }
  }
  else
    takeoff_btn_pressed_ = false;

  // **** land
  if (joy_msg->buttons[land_button_] == 1)
  {
    if (!land_btn_pressed_)
    {
      ROS_INFO("Land button pressed.");

      mav_srvs::Land land_srv;
      land_client_.call(land_srv);

      land_btn_pressed_ = true;
    }
  }
  else
    land_btn_pressed_ = false;

  // **** engage / disengage
  if (joy_msg->buttons[engage_button_ ] == 1)
  {
    if (!engage_btn_pressed_)
    {
      ROS_INFO("Toggle-Engage button pressed.");

      mav_srvs::ToggleEngage toggle_engage_srv; 
      toggle_engage_client_.call(toggle_engage_srv);

      engage_btn_pressed_ = true;
    }
  }
  else
    engage_btn_pressed_ = false;

  // **** HOLD
  if (joy_msg->buttons[pos_hold_button_ ] == 1)
  {
    if (!pos_hold_btn_pressed_)
    {
      ROS_INFO("Pos. Hold button pressed.");

      mav_srvs::PositionHold pos_hold_srv; 
      pos_hold_client_.call(pos_hold_srv);

      pos_hold_btn_pressed_ = true;
    }
  }
  else
    pos_hold_btn_pressed_ = false;

  // **** RELEASE
  if (joy_msg->buttons[vel_hold_button_ ] == 1)
  {
    if (!vel_hold_btn_pressed_)
    {
      ROS_INFO("Vel. Hold button pressed.");

      mav_srvs::VelocityHold vel_hold_srv; 
      vel_hold_client_.call(vel_hold_srv);

      vel_hold_btn_pressed_ = true;
    }
  }
  else
    vel_hold_btn_pressed_ = false;

  // restrict to one press per .20 seconds
  if ((ros::Time::now() - last_joy_event_).toSec() > 0.20) 
  {
    mav_srvs::ChangeDesPose change_des_pose_srv;
    change_des_pose_srv.request.delta_x   = 0.0;
    change_des_pose_srv.request.delta_y   = 0.0;
    change_des_pose_srv.request.delta_z   = 0.0;
    change_des_pose_srv.request.delta_yaw = 0.0;

    // **** z: axis 8 

    if (joy_msg->buttons[6] == 1)
    {
      ROS_INFO("Z-down button pressed.");
      change_des_pose_srv.request.delta_z = -z_step_size_;
      change_des_pose_client_.call(change_des_pose_srv);
      last_joy_event_ = ros::Time::now();
    }
    else if (joy_msg->buttons[4] == 1)
    {
      ROS_INFO("Z-up button pressed.");
      change_des_pose_srv.request.delta_z =  z_step_size_;
      change_des_pose_client_.call(change_des_pose_srv);
      last_joy_event_ = ros::Time::now();
    }
  }

  // **** direct yaw/pitch/roll/thrust commands

  mav_msgs::Roll    cmd_roll_msg;
  mav_msgs::Pitch   cmd_pitch_msg;
  mav_msgs::YawRate cmd_yaw_rate_msg;
  mav_msgs::Thrust  cmd_thrust_msg;

  cmd_roll_msg.roll = 
    joy_msg->axes[roll_axis_]   * cmd_roll_scale_ * (-1);
  cmd_pitch_msg.pitch = 
    joy_msg->axes[pitch_axis_]  * cmd_pitch_scale_;
  cmd_yaw_rate_msg.yaw_rate = 
    joy_msg->axes[yaw_axis_]    * cmd_yaw_rate_scale_;
  cmd_thrust_msg.thrust = 
    joy_msg->axes[thrust_axis_] * cmd_thrust_scale_;

  cmd_thrust_msg.thrust = std::max(cmd_thrust_msg.thrust, 0.0);

  cmd_roll_publisher_.publish(cmd_roll_msg);
  cmd_pitch_publisher_.publish(cmd_pitch_msg);
  cmd_yaw_rate_publisher_.publish(cmd_yaw_rate_msg);
  cmd_thrust_publisher_.publish(cmd_thrust_msg);

  // **** velocity commands

  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = ros::Time::now();

  double vx_j   = joy_msg->axes[vx_axis_];
  double vy_j   = joy_msg->axes[vy_axis_];
  double vz_j   = joy_msg->axes[vz_axis_];
  double vyaw_j = joy_msg->axes[vyaw_axis_];

  double vx, vy, vz, vyaw;

  // linear x
  if      (std::abs(vx_j) <= 0.1) vx = 0.0;
  else if (std::abs(vx_j) <= 1.0) vx = vx_j * cmd_vel_linear_scale_;
  else                            vx = 0.0;

  // linear y
  if      (std::abs(vy_j) <= 0.1) vy = 0.0;
  else if (std::abs(vy_j) <= 1.0) vy = vy_j * cmd_vel_linear_scale_;
  else                            vy = 0.0;

  // linear z
  if      (std::abs(vz_j) <= 0.1) vz = 0.0;
  else if (std::abs(vz_j) <= 1.0) vz = vz_j * cmd_vel_linear_scale_;
  else                            vz = 0.0;

  // angular z
  if      (std::abs(vyaw_j) <= 0.1) vyaw = 0.0;
  else if (std::abs(vyaw_j) <= 1.0) vyaw = vyaw_j * cmd_vel_angular_scale_;
  else                              vyaw = 0.0;

  twist_msg.twist.linear.x = vx;
  twist_msg.twist.linear.y = vy;
  twist_msg.twist.linear.z = vz;

  twist_msg.twist.angular.x = 0.0;
  twist_msg.twist.angular.y = 0.0;
  twist_msg.twist.angular.z = vyaw;

  cmd_vel_publisher_.publish(twist_msg);
}
