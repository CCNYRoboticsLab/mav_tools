#ifndef QUAD_JOY_TELEOP_QUAD_JOY_TELEOP_H
#define QUAD_JOY_TELEOP_QUAD_JOY_TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <mav_msgs/Roll.h>
#include <mav_msgs/Pitch.h>
#include <mav_msgs/YawRate.h>
#include <mav_msgs/Thrust.h>

#include <mav_srvs/ChangeDesPose.h>
#include <mav_srvs/ToggleEngage.h>
#include <mav_srvs/Takeoff.h>
#include <mav_srvs/Land.h>
#include <mav_srvs/ESTOP.h>
#include <mav_srvs/PositionHold.h>
#include <mav_srvs/VelocityHold.h>

class QuadJoyTeleop
{
  private:

    // **** ros-related variables

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    //ros::Timer estop_timer_;

    ros::Subscriber joy_subscriber_;
    ros::Publisher  estop_publisher_;

    //ros::Publisher  cmd_vel_publisher_;
    ros::Publisher cmd_roll_publisher_;
    ros::Publisher cmd_pitch_publisher_;
    ros::Publisher cmd_yaw_rate_publisher_;
    ros::Publisher cmd_thrust_publisher_;
    ros::Publisher cmd_vel_publisher_;

    ros::ServiceClient estop_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient change_des_pose_client_;
    ros::ServiceClient toggle_engage_client_;
    ros::ServiceClient pos_hold_client_;
    ros::ServiceClient vel_hold_client_;

    // **** parameters

    std::string controller_; 

    int estop_button_;
    int takeoff_button_;
    int land_button_;
    int engage_button_;
    int pos_hold_button_;
    int vel_hold_button_;

    int roll_axis_;
    int pitch_axis_;
    int yaw_axis_;
    int thrust_axis_;

    int z_axis_;

    int vx_axis_;
    int vy_axis_;
    int vz_axis_;
    int vyaw_axis_;

    double x_step_size_;
    double y_step_size_;
    double z_step_size_;
    double yaw_step_size_;

    double cmd_vel_linear_scale_; // scales stick units to m/s
    double cmd_vel_angular_scale_; // scales stick units to rad/s

    double linear_vel_fast_;
    double linear_vel_slow_;

    double cmd_roll_scale_;       
    double cmd_pitch_scale_;
    double cmd_yaw_rate_scale_;   
    double cmd_thrust_scale_;   

    // **** state variables
  
    bool estop_btn_pressed_;
    bool takeoff_btn_pressed_;
    bool land_btn_pressed_;
    bool engage_btn_pressed_;
    bool pos_hold_btn_pressed_;
    bool vel_hold_btn_pressed_;

    ros::Time last_joy_event_;

    geometry_msgs::Twist::Ptr cmd_vel_msg_;

    // **** member functions

    void initParams();

    void joyCallback (const sensor_msgs::JoyPtr& joy_msg);
    //void estopCallback(const ros::TimerEvent& event);

  public:

    QuadJoyTeleop(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~QuadJoyTeleop();

};

#endif // QUAD_JOY_TELEOP_QUAD_JOY_TELEOP_H
