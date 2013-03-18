#include "mav_odom_interface/odom_interface.h"

namespace mav {

OdomInterface::OdomInterface(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO("Starting OdomInterface"); 

  // ****  initialize variables
   
  pose_.pose.position.x = 0.0;
  pose_.pose.position.y = 0.0;
  pose_.pose.position.z = 0.0;

  odom2base_.setIdentity();
  odomvo2base_prev_.setIdentity();
  tf::poseTFToMsg(odom2base_, pose_.pose);

  ros::NodeHandle nh_mav (nh_, "mav");

  // **** parameters

  if (!nh_private_.getParam ("fixed_frame_vo", fixed_frame_vo_))
    fixed_frame_vo_ = "odom_vo";
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("use_vo_yaw", use_vo_yaw_))
    use_vo_yaw_ = true;
  
  pose_.header.frame_id = fixed_frame_;

  // **** ros publishers

  pose_publisher_ = nh_mav.advertise<geometry_msgs::PoseStamped>(
    "pose", 1);
  odom_publisher_ = nh_mav.advertise<nav_msgs::Odometry>(
    "odom", 1);
  path_pub_ = nh_mav.advertise<nav_msgs::Path>(
    "path_odom", 1);

  // **** ros subscribers

  rgbd_pose_subscriber_ = nh_mav.subscribe(
    "rgbd_pose", 1, &OdomInterface::rgbdPoseCallback, this);
  imu_subscriber_ = nh_mav.subscribe(
    "imu/data", 1, &OdomInterface::imuCallback, this);
  height_subscriber_ = nh_mav.subscribe(
    "height_to_base", 1, &OdomInterface::heightCallback, this);

  // publish initial identity pose
  pose_.header.stamp = ros::Time::now();
  publishPose();
}

OdomInterface::~OdomInterface()
{
  ROS_INFO("Destroying QuadPoseEst"); 
}

void OdomInterface::rgbdPoseCallback(const PoseStamped::ConstPtr& rgbd_pose_msg)
{
  pose_mutex_.lock();

  tf::Transform odomvo2base;
  tf::poseMsgToTF(rgbd_pose_msg->pose, odomvo2base);

  // vo motion, measured in the base frame
  tf::Transform d_base_frame =  odomvo2base_prev_.inverse() * odomvo2base;
  
  // apply the motion
  odom2base_ = odom2base_ * d_base_frame;

  // caluclate the correction between the odom and odom_vo
  tf::Transform odom2odomvo = odom2base_ * odomvo2base.inverse();
  
  // save for next timestamp
  odomvo2base_prev_ = odomvo2base;
  
  // update the pose header
  pose_.header.stamp = rgbd_pose_msg->header.stamp; 
  
  // publish the correction from odom to odomvo
  tf_broadcaster_.sendTransform(
    tf::StampedTransform(odom2odomvo,
      pose_.header.stamp, fixed_frame_, fixed_frame_vo_));
  
  // publish the pose
  tf::poseTFToMsg(odom2base_, pose_.pose);
  publishPose();
  publishPath();
  pose_mutex_.unlock();
}

void OdomInterface::imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  pose_mutex_.lock();

  tf::quaternionMsgToTF(imu_msg->orientation, curr_imu_q_);

  // mix 
  double roll, pitch, yaw;
  
  tf::Matrix3x3 m_imu(curr_imu_q_);
  m_imu.getRPY(roll, pitch, yaw);
  
  if (use_vo_yaw_)
    yaw = tf::getYaw(odom2base_.getRotation());

  tf::Quaternion q_mixed;
  q_mixed.setRPY(roll, pitch, yaw);
  odom2base_.setRotation(q_mixed);
  
  odom2base_.setRotation(curr_imu_q_);  
  pose_mutex_.unlock();
}

void OdomInterface::heightCallback (const mav_msgs::Height::ConstPtr& height_msg)
{
  pose_mutex_.lock();

  // use z from the height message

  pose_.pose.position.z = height_msg->height;

  // publish with the timestamp from this message

  pose_.header.stamp = height_msg->header.stamp;
  publishPose();

  pose_mutex_.unlock();
}

void OdomInterface::publishPose()
{
  // **** create a copy of the pose and publish as shared pointer

  geometry_msgs::PoseStamped::Ptr pose_message = 
    boost::make_shared<geometry_msgs::PoseStamped>(pose_);

  pose_publisher_.publish(pose_message);

  // **** broadcast the transform
  
  nav_msgs::Odometry::Ptr odom_message = 
    boost::make_shared<nav_msgs::Odometry>();

  odom_message->header = pose_.header;
  odom_message->child_frame_id = base_frame_;
  odom_message->pose.pose = pose_message->pose;
  
  odom_publisher_.publish(odom_message);
}

void OdomInterface::publishPath()
{
  path_msg_.header.stamp = pose_.header.stamp;
  path_msg_.header.frame_id = fixed_frame_;
     
  path_msg_.poses.push_back(pose_);
  path_pub_.publish(path_msg_);
}

} // namespace mav

