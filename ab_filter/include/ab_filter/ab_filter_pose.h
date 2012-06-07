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

#ifndef AB_FILTER_AB_FILTER_POSE_H
#define AB_FILTER_AB_FILTER_POSE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

namespace mav
{

class ABFilterPose
{
  typedef geometry_msgs::PoseStamped Pose;
  typedef geometry_msgs::TwistStamped Twist;

// for campatibility b/n ROS Electric and Fuerte
#if ROS_VERSION_MINIMUM(1, 8, 0)
  typedef tf::Matrix3x3 MyMatrix;
#else
  typedef btMatrix3x3 MyMatrix;
#endif

  private:

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber subscriber_;
    ros::Publisher pose_publisher_;
    ros::Publisher twist_publisher_;
    ros::Publisher twist_unf_publisher_;

    // **** state variables    

    bool initialized_;
    ros::Time last_update_time_;

    tf::Vector3 pos_;
    tf::Vector3 lin_vel_;
    tf::Vector3 lin_vel_unf_;

    tf::Quaternion q_;
    tf::Vector3 ang_vel_;
    tf::Vector3 ang_vel_unf_;

    double roll_, pitch_, yaw_;
    double v_roll_, v_pitch_, v_yaw_;

    // **** parameters
  
    double alpha_;
    double beta_;
    bool publish_unfiltered_;

    // **** member functions

    void initializeParams();

    void poseCallback(const Pose::ConstPtr pose_msg);
    void publishPose(const std_msgs::Header& header);

    // [0, 2pi)
    void normalizeAngle2Pi(double& angle);

    // [-pi, pi)
    void normalizeAnglePi(double& angle);

  public:

    ABFilterPose(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~ABFilterPose();
};

} // end namespace mav

#endif // AB_FILTER_AB_FILTER_POSE2D_H
