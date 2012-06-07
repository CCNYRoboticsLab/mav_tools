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

  tf::Quaternion q;
  q.setRPY(0,0,0);
  tf::quaternionTFToMsg(q, pose_.pose.orientation);

  ros::NodeHandle nh_mav (nh_, "mav");

  // **** parameters

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";

  pose_.header.frame_id = fixed_frame_;

  // **** ros publishers

  pose_publisher_ = nh_mav.advertise<geometry_msgs::PoseStamped>(
    "pose", 1);
  odom_publisher_ = nh_mav.advertise<nav_msgs::Odometry>(
    "odom", 1);

  // **** ros subscribers

  laser_pose_subscriber_ = nh_mav.subscribe(
    "laser_pose", 1, &OdomInterface::laserPoseCallback, this);
  imu_subscriber_ = nh_mav.subscribe(
    "imu", 1, &OdomInterface::imuCallback, this);
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

void OdomInterface::laserPoseCallback(const PoseStamped::ConstPtr& laser_pose_msg)
{
  pose_mutex_.lock();

  // use x and y from laser
  pose_.pose.position.x = laser_pose_msg->pose.position.x;
  pose_.pose.position.y = laser_pose_msg->pose.position.y;

  // use roll and pitch from IMU
  double roll, pitch, unused;
  tf::Quaternion q_pose;
  tf::quaternionMsgToTF(pose_.pose.orientation, q_pose);
  MyMatrix m_pose(q_pose);
  m_pose.getRPY(roll, pitch, unused);

  // use yaw from laser
  double yaw = tf::getYaw(laser_pose_msg->pose.orientation);

  // combine r, p, y
  tf::Quaternion q_result;
  q_result.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q_result, pose_.pose.orientation);

  // publish with the timestamp from this message
  pose_.header.stamp = laser_pose_msg->header.stamp;
  publishPose();

  pose_mutex_.unlock();
}

void OdomInterface::imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  pose_mutex_.lock();

  // use roll and pitch from IMU, and yaw from laser

  double r, p, y, unused;

  tf::Quaternion q_imu;
  tf::quaternionMsgToTF(imu_msg->orientation, q_imu);
  MyMatrix m_imu(q_imu);
  m_imu.getRPY(r, p, unused);

  tf::Quaternion q_pose;
  tf::quaternionMsgToTF(pose_.pose.orientation, q_pose);
  MyMatrix m_pose(q_pose);
  m_pose.getRPY(unused, unused, y);

  tf::Quaternion q_result;
  q_result.setRPY(r, p, y);
  tf::quaternionTFToMsg(q_result, pose_.pose.orientation);

  // publish with the timestamp from this message

  pose_.header.stamp = imu_msg->header.stamp;
  //publishPose();

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

  tf::Stamped<tf::Pose> tf_pose;
  tf::poseStampedMsgToTF(pose_, tf_pose);
  tf::StampedTransform odom_to_base_link_tf(
    tf_pose, pose_.header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform(odom_to_base_link_tf);

  // *** publish odometry message
  // TODO: add velociyies from ab filters
  
  nav_msgs::Odometry::Ptr odom_message = 
    boost::make_shared<nav_msgs::Odometry>();

  odom_message->header = pose_.header;
  odom_message->child_frame_id = "base_link";
  odom_message->pose.pose = pose_message->pose;
  
  odom_publisher_.publish(odom_message);
}

} // namespace mav

