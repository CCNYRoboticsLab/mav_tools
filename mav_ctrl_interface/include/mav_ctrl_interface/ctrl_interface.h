/*
 *  MAV Ctrl Interface
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

#ifndef MAV_CTRL_INTERFACE_CTRL_INTERFACE_H
#define MAV_CTRL_INTERFACE_CTRL_INTERFACE_H

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/service_server.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <mav_srvs/ChangeDesPose.h>
#include <mav_srvs/PositionHold.h>
#include <mav_srvs/VelocityHold.h>
#include <mav_srvs/SetCtrlType.h>

#include <mav_common/control_types.h>

namespace mav
{

class CtrlInterface
{
  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber cur_pose_subscriber_;
    ros::Subscriber goal2D_subscriber_; // receives goals from rviz

    ros::Subscriber cmd_joy_vel_subscriber_;  //receives cmd_vel from joystick
    ros::Subscriber cmd_plan_vel_subscriber_; //receives cmd_vel from planner

    ros::Publisher cmd_pose_publisher_; // passes cmd_pose to MAV
    ros::Publisher cmd_vel_publisher_;  // passer cmd_vel to MAV

    ros::Publisher goal_publisher_;  // for visualization
    ros::Publisher plan_publisher_;  // for visualization
    ros::Publisher array_publisher_; // for visualization

    ros::ServiceServer change_des_pose_srv_;
    ros::ServiceServer pos_hold_srv_;
    ros::ServiceServer vel_hold_srv_;

    ros::ServiceClient set_ctrl_type_client_;

    ros::Timer cmd_timer_;
    ros::Timer plan_timer_;

    tf::TransformListener tf_listener_;
    costmap_2d::Costmap2DROS costmap_;
    navfn::NavfnROS navfn_;

    // **** state variables    

    boost::mutex mutex_;

    ControlType ctrl_type_; // ROS-level control mode (position, velocity)

    geometry_msgs::PoseStamped cur_pose_;
    geometry_msgs::PoseStamped des_pose_;
    geometry_msgs::PoseStamped cur_goal_;

    std::vector<geometry_msgs::PoseStamped> plan_;
    std::vector<geometry_msgs::PoseStamped> plan_decomposed_;

    geometry_msgs::Twist des_vel_;

    // **** parameters

    std::string fixed_frame_;

    bool direct_pos_ctrl_;

    bool allow_joy_vel_cmd_;
    bool allow_plan_vel_cmd_;

    double wp_dist_max_;         // for waypont pos ctrl, max allowed dist b/n waypoints  
    double yaw_turn_tolerance_;  // when to turn on a dime
    double plan_goal_tolerance_; // for the NavFN planner

    double wp_dist_tolerance_;   // when to consider a waypoint reached
    double wp_angle_tolerance_;  // when to consider a waypoint reached 

    double goal_tf_tolerance_;   // when transfroming goals to the fixed_frame
   
    // **** member functions

    void initializeParams();

    void curPoseCallback(const geometry_msgs::PoseStamped::ConstPtr pose_msg);
    void goal2Dcallback(const geometry_msgs::PoseStamped::ConstPtr pose_msg);

    void cmdJoyVelCallback(const geometry_msgs::TwistStamped::ConstPtr twist_msg);
    void cmdPlanVelCallback(const geometry_msgs::Twist::ConstPtr twist_msg);


    bool changeDesPose(mav_srvs::ChangeDesPose::Request  &req,
                       mav_srvs::ChangeDesPose::Response &res);

    bool positionHold(mav_srvs::PositionHold::Request  &req,
                      mav_srvs::PositionHold::Response &res);
    bool velocityHold(mav_srvs::VelocityHold::Request  &req,
                      mav_srvs::VelocityHold::Response &res);
 
    void cmdTimerCallback(const ros::TimerEvent& event);
    void planTimerCallback(const ros::TimerEvent& event);

    void publishCmdPose();
    void publishCmdVel();
    void publishPlans();

    void computePlan();
    void computeWpFromPlan();

    bool decomposePlan(
      const std::vector<geometry_msgs::PoseStamped>& plan_in, 
      std::vector<geometry_msgs::PoseStamped>& plan_out);

    bool decomposePlan(
      const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& plan_in, 
      std::vector<geometry_msgs::PoseStamped>& plan_out,
      int start, int end);

  public:

    CtrlInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~CtrlInterface();
};

} // end namespace mav

#endif // MAV_CTRL_INTERFACE_CTRL_INTERFACE_H
