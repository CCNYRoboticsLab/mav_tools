/*
 *  Alpha-Beta Filter
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

#include "ab_filter/ab_filter_pose.h"

namespace mav
{

ABFilterPose::ABFilterPose(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false)
{
  ROS_INFO("Starting ABFilterPose"); 

  ros::NodeHandle nh_mav (nh_, "mav");

  // **** get parameters

  initializeParams();

  // *** register publishers

  pose_publisher_  = nh_mav.advertise<Pose>(
    "pose_f", 10);
  twist_publisher_  = nh_mav.advertise<Twist>(
    "twist_f", 10);

  if (publish_unfiltered_)
  {
    twist_unf_publisher_ = nh_mav.advertise<Twist>(
      "twist_unf", 10);
  }

  // **** register subscribers

  subscriber_ = nh_mav.subscribe(
    "pose", 10, &ABFilterPose::poseCallback, this);
}

ABFilterPose::~ABFilterPose()
{
  ROS_INFO("Destroying ABFilterPose"); 

}

void ABFilterPose::initializeParams()
{
  if (!nh_private_.getParam ("alpha", alpha_))
    alpha_ = 0.9;
  if (!nh_private_.getParam ("beta", beta_))
    beta_ = 0.5;
  if (!nh_private_.getParam ("publish_unfiltered", publish_unfiltered_))
    publish_unfiltered_ = false;
}


void ABFilterPose::poseCallback(const Pose::ConstPtr pose_msg)
{
  ros::Time time = pose_msg->header.stamp;

  tf::Vector3 pos_reading;
  tf::pointMsgToTF(pose_msg->pose.position, pos_reading);

  tf::Quaternion q_reading;
  tf::quaternionMsgToTF(pose_msg->pose.orientation, q_reading);

  // first message
  if(!initialized_)
  {
    initialized_ = true;

    // set initial position and orientation to the first pose message
    pos_ = pos_reading;
    q_   = q_reading;

    // set initial velocities to 0
    lin_vel_ = tf::Vector3(0.0, 0.0, 0.0);
    ang_vel_ = tf::Vector3(0.0, 0.0, 0.0); 
  }
  else
  {
    double dt = (time - last_update_time_).toSec();
    double bdt = beta_ / dt;

    if (publish_unfiltered_)
    {
      lin_vel_unf_ = (pos_reading - pos_) / dt;
    }

    // **** calculate position

    tf::Vector3 pos_pred = pos_ + dt * lin_vel_;
    tf::Vector3 r_pos = pos_reading - pos_pred;
    pos_ = pos_pred + alpha_ * r_pos;
    lin_vel_ += bdt * r_pos;

    // **** calculate orientation

    tf::Vector3 w = dt * ang_vel_;
    tf::Quaternion qw;
    qw.setRPY(w.getX(), w.getY(), w.getZ());

    tf::Quaternion q_pred = qw * q_;
    tf::Quaternion q_new = q_pred.slerp(q_reading, alpha_);  

    tf::Quaternion r = q_reading * q_.inverse();
    
    q_ = q_new;

    double r_roll, r_pitch, r_yaw;
    MyMatrix r_m(r);
    r_m.getRPY(r_roll, r_pitch, r_yaw);
    tf::Vector3 ang_vel_meas(r_roll/dt, r_pitch/dt, r_yaw/dt);

    tf::Vector3 ang_vel_pred = ang_vel_;
    
    ang_vel_ = (1.0 - beta_) * ang_vel_pred + beta_ * ang_vel_meas;

    // **** calculate unfiltered velocity 
    if (publish_unfiltered_)
    {
      ang_vel_unf_ = ang_vel_meas;
    }
  }

  last_update_time_ = time;

  publishPose(pose_msg->header);
}

void ABFilterPose::publishPose(const std_msgs::Header& header)
{
  // **** publish pose message

  Pose::Ptr pose = boost::make_shared<Pose>();
  pose->header = header;

  tf::pointTFToMsg(pos_, pose->pose.position);
  tf::quaternionTFToMsg(q_, pose->pose.orientation);

  pose_publisher_.publish(pose);

  // **** publish twist message

  Twist::Ptr twist = boost::make_shared<Twist>();
  twist->header = header;
  
  tf::vector3TFToMsg(lin_vel_, twist->twist.linear);
  tf::vector3TFToMsg(ang_vel_, twist->twist.angular);

  twist_publisher_.publish(twist);

  // **** publish unfiltered twist message

  if (publish_unfiltered_)
  {
    Twist::Ptr twist_unf = boost::make_shared<Twist>();
    twist_unf->header = header;

    tf::vector3TFToMsg(lin_vel_unf_, twist_unf->twist.linear);
    tf::vector3TFToMsg(ang_vel_unf_, twist_unf->twist.angular);

    twist_unf_publisher_.publish(twist_unf);
  }
}

void ABFilterPose::normalizeAngle2Pi(double& angle)
{
  while (angle  <        0.0) angle += 2.0 * M_PI;
  while (angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;
}

void ABFilterPose::normalizeAnglePi(double& angle)
{
  while (angle <= -M_PI) angle += 2.0 * M_PI;
  while (angle >   M_PI) angle -= 2.0 * M_PI;
}

} // end namespace asctec
