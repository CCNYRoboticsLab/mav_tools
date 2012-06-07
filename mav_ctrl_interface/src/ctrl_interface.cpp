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

#include "mav_ctrl_interface/ctrl_interface.h"

namespace mav
{

CtrlInterface::CtrlInterface(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  tf_listener_(ros::Duration(10.0)),
  costmap_("my_costmap", tf_listener_)
{
  ROS_INFO("Starting CtrlInterface"); 

  ros::NodeHandle nh_mav (nh_, "mav");

  // **** get parameters

  initializeParams();

  // **** initialize vaiables

  ctrl_type_ = mav::PositionCtrl;

  des_vel_.linear.x = 0.0;
  des_vel_.linear.y = 0.0;
  des_vel_.linear.z = 0.0;
  des_vel_.angular.x = 0.0;
  des_vel_.angular.y = 0.0;
  des_vel_.angular.z = 0.0;

  tf::Pose identity;
  identity.setIdentity();
  tf::poseTFToMsg(identity, cur_pose_.pose);  
  cur_pose_.header.frame_id = fixed_frame_;

  cur_goal_ = des_pose_ = cur_pose_;

  navfn_.initialize("my_navfn_planner", &costmap_);

  // *** register publishers

  cmd_pose_publisher_ = nh_mav.advertise<geometry_msgs::PoseStamped>(
    "cmd/pose", 10);
  cmd_vel_publisher_ = nh_mav.advertise<geometry_msgs::TwistStamped>(
    "cmd/vel", 10);

  goal_publisher_= nh_mav.advertise<geometry_msgs::PoseStamped>(
    "cmd/goal", 10);
  plan_publisher_ = nh_mav.advertise<nav_msgs::Path>(
    "plan", 10);
  array_publisher_ = nh_mav.advertise<geometry_msgs::PoseArray>(
    "plan_array", 10);

  // **** register subscribers

  cur_pose_subscriber_ = nh_mav.subscribe(
    "pose", 10, &CtrlInterface::curPoseCallback, this);
  goal2D_subscriber_ = nh_mav.subscribe(
    "goal2D_rviz", 10, &CtrlInterface::goal2Dcallback, this);
  cmd_joy_vel_subscriber_ = nh_mav.subscribe(
    "cmd_joy/vel", 10, &CtrlInterface::cmdJoyVelCallback, this);
  cmd_plan_vel_subscriber_ = nh_mav.subscribe(
    "cmd_plan/vel", 10, &CtrlInterface::cmdPlanVelCallback, this);

  // **** register service servers

  change_des_pose_srv_ = nh_mav.advertiseService(
    "changeDesPose", &CtrlInterface::changeDesPose, this);
  pos_hold_srv_ = nh_mav.advertiseService(
    "positionHold", &CtrlInterface::positionHold, this);
  vel_hold_srv_ = nh_mav.advertiseService(
    "velocityHold", &CtrlInterface::velocityHold, this);

  set_ctrl_type_client_ = nh_mav.serviceClient<mav_srvs::SetCtrlType>(
    "setCtrlType");

  // **** timers

  cmd_timer_ = nh_private_.createTimer(
    ros::Duration(0.100), &CtrlInterface::cmdTimerCallback, this);
  plan_timer_ = nh_private_.createTimer(
    ros::Duration(0.100), &CtrlInterface::planTimerCallback, this);
}

CtrlInterface::~CtrlInterface()
{
  ROS_INFO("Destroying QuadSimplePlanner"); 

}

void CtrlInterface::initializeParams()
{
  yaw_turn_tolerance_ = 30.0 * (M_PI / 180.0);
  plan_goal_tolerance_ = 0.10;

  goal_tf_tolerance_ = 5.0;

  if (!nh_private_.getParam("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";

  if (!nh_private_.getParam("direct_pos_ctrl", direct_pos_ctrl_))
    direct_pos_ctrl_ = true;

  if (!nh_private_.getParam("allow_joy_vel_cmd", allow_joy_vel_cmd_))
    allow_joy_vel_cmd_ = true;
  if (!nh_private_.getParam("allow_plan_vel_cmd", allow_plan_vel_cmd_))
    allow_plan_vel_cmd_ = true;

  if (!nh_private_.getParam("wp_dist_max", wp_dist_max_))
    wp_dist_max_ = 1.50;
  if (!nh_private_.getParam("wp_dist_tolerance", wp_dist_tolerance_))
    wp_dist_tolerance_ = 0.10;
  if (!nh_private_.getParam("wp_angle_tolerance", wp_angle_tolerance_))
    wp_angle_tolerance_ = 10.0 * (M_PI / 180.0);

  if (allow_joy_vel_cmd_) 
    ROS_INFO("Joystick velocity commands are enabled.");
  else
    ROS_INFO("Joystick velocity commands are disabled.");

  if (allow_plan_vel_cmd_) 
    ROS_INFO("Planner velocity commands are enabled.");
  else
    ROS_INFO("Planner velocity commands are disabled.");

  fixed_frame_ = tf_listener_.resolve(fixed_frame_);
}


void CtrlInterface::planTimerCallback(const ros::TimerEvent& event)
{
  boost::mutex::scoped_lock(mutex_);

  if (ctrl_type_ == mav::PositionCtrl && !direct_pos_ctrl_)
  { 
    computeWpFromPlan();
  }
}

void CtrlInterface::computeWpFromPlan()
{
  // **** if the plan is mepty, don't do anything
  
  if (plan_decomposed_.empty()) return;

  // **** find the first waypoint which is outside of tolerance

  unsigned int wp_idx;

  for (wp_idx = 0; wp_idx < plan_decomposed_.size(); ++wp_idx)
  {
    const geometry_msgs::PoseStamped& plan_wp = plan_decomposed_[wp_idx];

    // calculate distance to waypoint
    float dx = plan_wp.pose.position.x - cur_pose_.pose.position.x;
    float dy = plan_wp.pose.position.y - cur_pose_.pose.position.y;
    float wp_dist = sqrt(dx*dx + dy*dy);

    // calculate angle to waypoint
    tf::Quaternion cur_q, wp_q;
    tf::quaternionMsgToTF(cur_pose_.pose.orientation, cur_q);
    tf::quaternionMsgToTF(plan_wp.pose.orientation, wp_q);
    float cur_angle = tf::getYaw(cur_q);
    float wp_angle = tf::getYaw(wp_q);
    float wp_angle_diff = wp_angle - cur_angle;
    if      (wp_angle_diff >= M_PI) wp_angle_diff -= 2.0 * M_PI;
    else if (wp_angle_diff < -M_PI) wp_angle_diff += 2.0 * M_PI; 

    // if waypoint isnt reached, use it as des_pose
    if (wp_dist > wp_dist_tolerance_ || 
        std::abs(wp_angle_diff) > wp_angle_tolerance_)
    {
      // TODO: some check for collision here

      des_pose_.pose.position.x = plan_wp.pose.position.x;
      des_pose_.pose.position.y = plan_wp.pose.position.y;
      des_pose_.pose.orientation = plan_wp.pose.orientation;

      break;
    }
  }
 
  // **** erase old (reached) waypoints
  plan_decomposed_.erase(plan_decomposed_.begin(), 
                         plan_decomposed_.begin() + wp_idx);

}

void CtrlInterface::cmdTimerCallback(const ros::TimerEvent& event)
{
  boost::mutex::scoped_lock(mutex_);

  if (ctrl_type_ == mav::PositionCtrl) 
  {
    publishCmdPose();
  }
}

void CtrlInterface::cmdJoyVelCallback(const geometry_msgs::TwistStamped::ConstPtr twist_msg)
{
  boost::mutex::scoped_lock(mutex_);

  if (!allow_joy_vel_cmd_)
  {
    ROS_WARN("Received joystick velocity command, but allow_joy_vel_cmd is set to false");
    return;
  }

  if (ctrl_type_ != mav::VelocityCtrl)
  {
    ROS_WARN("Received joystick velocity command, but ctrl_type is not VELOCITY");
    return;
  }

  // TODO: check for frame here

  ROS_INFO("Executing velocity command from joystick.");

  des_vel_ = twist_msg->twist;

  publishCmdVel();
}

void CtrlInterface::cmdPlanVelCallback(const geometry_msgs::Twist::ConstPtr twist_msg)
{
  boost::mutex::scoped_lock(mutex_);

  if (!allow_plan_vel_cmd_)
  {
    ROS_WARN("Received planner velocity command, but allow_plan_vel_cmd is set to false");
    return;
  }

  if (ctrl_type_ != mav::VelocityCtrl)
  {
    ROS_WARN("Received joystick velocity command, but ctrl_type is not VELOCITY");
    return;
  }

  // TODO: check for frame here

  ROS_INFO("Executing velocity command from planner.");

  des_vel_ = *twist_msg;

  publishCmdVel();
}

void CtrlInterface::curPoseCallback(const geometry_msgs::PoseStamped::ConstPtr pose_msg)
{
  boost::mutex::scoped_lock(mutex_);

  std::string frame = tf_listener_.resolve(pose_msg->header.frame_id);

  if (frame != fixed_frame_)
  {
    ROS_INFO_ONCE("Pose message does not match the fixed frame (%s, %s), transforming",
      frame.c_str(), fixed_frame_.c_str());
    
    try
    {
      tf_listener_.waitForTransform(
        fixed_frame_, pose_msg->header.frame_id, pose_msg->header.stamp, ros::Duration(1.0));

      tf_listener_.transformPose(fixed_frame_, *pose_msg, cur_pose_);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not transform goal pose %s", ex.what());
      return;
    }
  }
  else
  {
    cur_pose_ = *pose_msg;
  }
}

void CtrlInterface::goal2Dcallback(const geometry_msgs::PoseStamped::ConstPtr goal_msg)
{
  if (ctrl_type_ == mav::PositionCtrl)
  {
    // **** transform the message to the correct frame
    std::string frame = tf_listener_.resolve(goal_msg->header.frame_id);
    geometry_msgs::PoseStamped goal_fixed_frame;

    if (frame != fixed_frame_)
    {
      ROS_INFO_ONCE("Goal messages does not match the fixed frame (%s, %s), transforming",
        frame.c_str(), fixed_frame_.c_str());
      
      try
      {
        tf_listener_.waitForTransform(
          fixed_frame_, goal_msg->header.frame_id, goal_msg->header.stamp, ros::Duration(goal_tf_tolerance_));

        tf_listener_.transformPose(fixed_frame_, *goal_msg, goal_fixed_frame);
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("Could not transform goal pose %s", ex.what());
        return;
      }
    }
    else
    {
      goal_fixed_frame = *goal_msg;
    }

    boost::mutex::scoped_lock(mutex_);

    if (direct_pos_ctrl_)
    {
      // **** direct pos. ctrl - no obstacle avoidance
      ROS_INFO("Goal(2d) received. Executing direct poistion control to goal.");

      // set 2D porion of pose to the new goal
      des_pose_.pose.position.x = goal_fixed_frame.pose.position.x;
      des_pose_.pose.position.y = goal_fixed_frame.pose.position.y;

      double des_yaw = tf::getYaw(goal_fixed_frame.pose.orientation);
      tf::Quaternion des_q = tf::createQuaternionFromYaw(des_yaw);
      tf::quaternionTFToMsg(des_q, des_pose_.pose.orientation);

      publishCmdPose();
    }
    else
    {
      // **** obstacle avoidance
      ROS_INFO("Goal(2d) received. Will use for path planning");

      // set 2D porion of pose to the new goal
      cur_goal_.pose.position.x = goal_fixed_frame.pose.position.x;
      cur_goal_.pose.position.y = goal_fixed_frame.pose.position.y;

      double goal_yaw = tf::getYaw(goal_fixed_frame.pose.orientation);
      tf::Quaternion goal_q = tf::createQuaternionFromYaw(goal_yaw);
      tf::quaternionTFToMsg(goal_q, cur_goal_.pose.orientation);

      computePlan();

      // for visualization
      publishPlans();
    }
  }
  else if (ctrl_type_ == mav::VelocityCtrl)
  {
    ROS_WARN("Received Goal(2d) while in velocity node, ignoring.");
  }
  else
  {
    ROS_WARN("Unknown control mode");
  }
}

bool CtrlInterface::positionHold(mav_srvs::PositionHold::Request  &req,
                                 mav_srvs::PositionHold::Response &res)
{
  ROS_INFO("Position hold request received");

  boost::mutex::scoped_lock(mutex_);

  // cancel previous commands - freeze to current point
  des_pose_ = cur_pose_;

  // clear the plan
  cur_goal_ = cur_pose_;
  plan_.clear();
  plan_decomposed_.clear();

  publishPlans();
  publishCmdPose();

  // set controller to position control
  ROS_INFO("Setting ctrl mode to POSITION control");
  ctrl_type_ = mav::PositionCtrl;

  // call the flyer interface service
  mav_srvs::SetCtrlType set_ctrl_type_srv;
  set_ctrl_type_srv.request.ctrl_type = ctrl_type_;
  return set_ctrl_type_client_.call(set_ctrl_type_srv);
}

void CtrlInterface::publishPlans()
{
  // **** publish the goal
  
  goal_publisher_.publish(cur_goal_);

  // **** publish the original plan

  navfn_.publishPlan(plan_, 1.0, 0.0, 1.0, 1.0);

  // **** publish the decomposed plan

  nav_msgs::Path path_msg;
  path_msg.header.frame_id = cur_goal_.header.frame_id;
  path_msg.poses = plan_decomposed_;
  
  plan_publisher_.publish(path_msg);

  // **** publish the plan array

  geometry_msgs::PoseArray array_msg;
  array_msg.header = path_msg.header;

  for (unsigned int i = 0; i < path_msg.poses.size(); ++i)
    array_msg.poses.push_back(path_msg.poses[i].pose);

  array_publisher_.publish(array_msg);

  // **** printf for debug
  /*
  for (unsigned int i = 0; i < plan_decomposed_.size(); ++i)
  {
    geometry_msgs::Pose& a = plan_decomposed_[i].pose;

    tf::Quaternion q;
    tf::quaternionMsgToTF(a.orientation, q);

    printf("[%d] %f %f %f\n", i, a.position.x, a.position.y, tf::getYaw(q));
  }
  */
}

bool CtrlInterface::velocityHold(mav_srvs::VelocityHold::Request  &req,
                                 mav_srvs::VelocityHold::Response &res)
{
  ROS_INFO("Velocity hold request received");

  boost::mutex::scoped_lock(mutex_);

  // cancel prev. vel commands - freeze to 0 velocity
  des_vel_.linear.x = 0.0;
  des_vel_.linear.y = 0.0;
  des_vel_.linear.z = 0.0;
  des_vel_.angular.x = 0.0;
  des_vel_.angular.y = 0.0;
  des_vel_.angular.z = 0.0;
  publishCmdVel();
  
  // set controller to velocity control
  ROS_INFO("Setting ctrl type to VELOCITY control");
  ctrl_type_ = mav::VelocityCtrl;

  // call the flyer interface service
  mav_srvs::SetCtrlType set_ctrl_type_srv;
  set_ctrl_type_srv.request.ctrl_type = ctrl_type_; 
  return set_ctrl_type_client_.call(set_ctrl_type_srv);
}

bool CtrlInterface::changeDesPose(mav_srvs::ChangeDesPose::Request  &req,
                                  mav_srvs::ChangeDesPose::Response &res)
{
  ROS_INFO("ChangeDesPose request received");

  boost::mutex::scoped_lock(mutex_);

  publishCmdPose();
 
  des_pose_.pose.position.x += req.delta_x;
  des_pose_.pose.position.y += req.delta_y;
  des_pose_.pose.position.z += req.delta_z;

  publishCmdPose();

  return true;
}

void CtrlInterface::publishCmdPose()
{
	geometry_msgs::PoseStamped cmd_pose;
  
  cmd_pose = des_pose_;
  cmd_pose.header.stamp = ros::Time::now();

  cmd_pose_publisher_.publish(cmd_pose);
}

void CtrlInterface::publishCmdVel()
{
	geometry_msgs::TwistStamped cmd_vel;
  
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = fixed_frame_;
  cmd_vel.twist = des_vel_;

  cmd_vel_publisher_.publish(cmd_vel);
}

void CtrlInterface::computePlan()
{
  // **** clear previous plans

  plan_.clear();
  plan_decomposed_.clear();

  // **** create a path plan using the map

  ros::Time start_time_plan = ros::Time::now();
  bool result_plan = navfn_.makePlan(cur_pose_, cur_goal_, plan_goal_tolerance_, plan_);
  ros::Time end_time_plan = ros::Time::now();

  if (result_plan)
  {
    ROS_INFO("Plan computed in %.1f ms", (end_time_plan - start_time_plan).toSec() * 1000.0);
  }
  else
  {
    // TODO: handle this, maybe using position hold?
    ROS_WARN("Plan could not be computed!");
    return;
  }

  // **** decompose the path plan into a waypoint plan

  ros::Time start_time_dec = ros::Time::now();
  bool result_dec = decomposePlan(plan_, plan_decomposed_);
  ros::Time end_time_dec = ros::Time::now();

  if (result_dec)
  {
    ROS_INFO("Plan decomposed in %.1f ms", (end_time_dec - start_time_dec).toSec() * 1000.0);
  }
  else
  {
    // TODO: handle this, maybe using position hold?
    ROS_WARN("Plan could not be decomposed!");
    return;
  }
}

bool CtrlInterface::decomposePlan(
  const std::vector<geometry_msgs::PoseStamped>& plan_in, 
  std::vector<geometry_msgs::PoseStamped>& plan_out)
{
  plan_out.clear();

  int size = plan_in.size();

  int start = 0;
  int end = size - 1;

  costmap_2d::Costmap2D costmap;
  costmap_.getCostmapCopy(costmap);

  bool result = decomposePlan(costmap, plan_in, plan_out, start, end);

  if (!result) return false;

  // **** compute orientation for each WP

  // set start point and end point orientations to cur_pose and cur_goal
  geometry_msgs::PoseStamped& s_p = plan_out[0];
  geometry_msgs::PoseStamped& e_p = plan_out[plan_out.size() - 1];  

  s_p.pose.orientation = cur_pose_.pose.orientation;
  e_p.pose.orientation = cur_goal_.pose.orientation;

  plan_out.push_back(plan_out[plan_out.size() - 1]);

  // set orientations for inside waypoints
  for (unsigned int i = 0; i < plan_out.size() - 2; ++i)
  {
    geometry_msgs::PoseStamped& a = plan_out[i];
    geometry_msgs::PoseStamped& b = plan_out[i+1];    

    float dx = b.pose.position.x - a.pose.position.x;
    float dy = b.pose.position.y - a.pose.position.y;

    float angle = atan2(dy, dx);
  
    tf::Quaternion b_q = tf::createQuaternionFromYaw(angle);    
    tf::quaternionTFToMsg(b_q, b.pose.orientation);
  }

  // **** introduce turn-on-dime waypoints

  std::vector<geometry_msgs::PoseStamped> plan_out_f;
  for (unsigned int i = 0; i < plan_out.size() - 1; ++i)
  {
    geometry_msgs::PoseStamped& a = plan_out[i];
    geometry_msgs::PoseStamped& b = plan_out[i+1];         

    tf::Quaternion q_a, q_b;

    tf::quaternionMsgToTF(a.pose.orientation, q_a);
    tf::quaternionMsgToTF(b.pose.orientation, q_b);
  
    float yaw_a = tf::getYaw(q_a);
    float yaw_b = tf::getYaw(q_b);

    float d_angle = (yaw_b - yaw_a);
    if      (d_angle >= M_PI) d_angle -= 2.0 * M_PI;
    else if (d_angle < -M_PI) d_angle += 2.0 * M_PI;

    plan_out_f.push_back(a);

    if (std::abs(d_angle) > yaw_turn_tolerance_)
    {
      geometry_msgs::PoseStamped m = a;
      m.pose.orientation = b.pose.orientation;
      plan_out_f.push_back(m);
    }
  }

  plan_out = plan_out_f;

  return true;
}

bool CtrlInterface::decomposePlan(
  const costmap_2d::Costmap2D& costmap,
  const std::vector<geometry_msgs::PoseStamped>& plan_in, 
  std::vector<geometry_msgs::PoseStamped>& plan_out,
  int start, int end)
{
  // 0. params - TODO: move these out of funciton

  float resolution = 0.05;  // TODO: get these from map
  int cost_threshold = 100;

  // 1. check for a straight line between start and end

  if (end - start == 1)
  {
    plan_out.resize(2);
    plan_out[0] = plan_in[start];
    plan_out[1] = plan_in[end];
    return true;
  }

  if (start == end)
  {
    printf("ERROR\n");
    return false;
  }

  const geometry_msgs::Point& p_start = plan_in[start].pose.position;
  const geometry_msgs::Point& p_end   = plan_in[end].pose.position;

  double dx = p_end.x - p_start.x;
  double dy = p_end.y - p_start.y;
  double L = sqrt(dx*dx + dy*dy);
  
  // if length is reasonably small
  if (L < wp_dist_max_)
  {
    dx /= L;
    dy /= L;
    
    dx *= resolution;
    dy *= resolution;

    double l = 0.0;
    double x = p_start.x;
    double y = p_start.y;

    bool obstacle_found = false;

    while(l < L)
    {
      unsigned int mx, my;
      costmap.worldToMap(x, y, mx, my);
      int cost = costmap.getCost(mx, my);

      if (cost > cost_threshold)
      {
        obstacle_found = true;
        break;
      }  
      x += dx;
      y += dy;
      l += resolution;
    }

    if (!obstacle_found)
    {
      // straight line exists
      plan_out.resize(2);
      plan_out[0] = plan_in[start];
      plan_out[1] = plan_in[end];
      return true;
    }
  }

  // 2. if no straight line exists, or length too big,
  // divide and recurse

  int mid = (start + end) / 2;

  std::vector<geometry_msgs::PoseStamped> plan_l; 
  std::vector<geometry_msgs::PoseStamped> plan_r;

  bool left_result  = decomposePlan(costmap, plan_in, plan_l, start, mid);
  bool right_result = decomposePlan(costmap, plan_in, plan_r, mid,   end);

  if (!left_result || !right_result) return false;

  plan_out.reserve(plan_l.size() + plan_r.size() - 1);
  plan_out.insert(plan_out.begin(), plan_l.begin(), plan_l.end());
  plan_out.insert(plan_out.end(), plan_r.begin() + 1, plan_r.end());

  return true;
}


} // end namespace mav
