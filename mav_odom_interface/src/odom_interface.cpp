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
  tango2base_prev_.setIdentity();
  
  //tf::poseTFToMsg(odom2base_, pose_.pose); 	
  //tf::Quaternion q;
  //q.setRPY(0,0,0);
  //tf::quaternionTFToMsg(q, pose_.pose.orientation);

  ros::NodeHandle nh_mav (nh_, "mav");

  // **** parameters

  if (!nh_private_.getParam ("fixed_frame_vo", fixed_frame_vo_))
    fixed_frame_vo_ = "odom_vo";
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("sensor_frame", sensor_frame_))
    sensor_frame_ = "tango";
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

  tango_pose_subscriber_ = nh_mav.subscribe(
    "tango/pose_stamped", 1, &OdomInterface::tangoPoseCallback, this);
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

void OdomInterface::tangoPoseCallback(const PoseStamped::ConstPtr& tango_pose_msg)
{
  pose_mutex_.lock();
  PoseStamped tango_pose;
  tf::Transform odomo2tango;
  tango_pose.pose = tango_pose_msg->pose;
  tango_pose.header = tango_pose_msg->header;
  tango_pose.header.stamp = tango_pose_msg->header.stamp;
  tango_pose.header.frame_id = "odom";
  tf::poseMsgToTF(tango_pose.pose, odomo2tango);

  ros::Time now = ros::Time::now(); 
  tf::StampedTransform tango2base;
  tango2base.setIdentity();
  
  try
    {
      tf_listener_.waitForTransform(
	base_frame_, sensor_frame_, now, ros::Duration(5.0));

      tf_listener_.lookupTransform(base_frame_, sensor_frame_, now, tango2base);
      		
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not transform tango pose %s", ex.what());
      return;
    } 
  //tango motion, measured in the base frame
  
 

  tf_broadcaster_.sendTransform(
    tf::StampedTransform(tango2base,
      ros::Time::now(), base_frame_, sensor_frame_));

  //tango2base.setIdentity();
  
  // apply the transf
  odom2base_ = odomo2tango * tango2base.inverse();

  // update the pose header
  pose_.header.stamp = tango_pose_msg->header.stamp; 
 
  //tf::Transform odom2base;
  tf_broadcaster_.sendTransform(
    tf::StampedTransform(odom2base_,
      ros::Time::now(), fixed_frame_, base_frame_));
  // publish the pose
  tf::poseTFToMsg(odom2base_, pose_.pose);
 
  publishPose();
  publishPath();
  pose_mutex_.unlock();
/***********/

/* NOT USING IMU FOR NOW!!!!!!!!!!!!!!!!!!
  // use roll and pitch from IMU
  double roll, pitch, unused;
  tf::Quaternion q_pose;
  tf::quaternionMsgToTF(pose_.pose.orientation, q_pose);
  MyMatrix m_pose(q_pose);
  m_pose.getRPY(roll, pitch, unused);

  // use yaw from rgbd
  double yaw = tf::getYaw(rgbd_pose_msg->pose.orientation);

  // combine r, p, y
  tf::Quaternion q_result;
  q_result.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q_result, pose_.pose.orientation);
*/
  // publish with the timestamp from this message
  //pose_.header.stamp = tango_pose_msg->header.stamp;
  //publishPose();

  //pose_mutex_.unlock();
}

void OdomInterface::imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  pose_mutex_.lock();

  // use roll and pitch from IMU, and yaw from rgbd

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

  //tf::poseStampedMsgToTF(pose_, tf_pose);
  //tf::StampedTransform odom_to_frame_tango_tf(
   // tf_pose, pose_.header.stamp, fixed_frame_, "tango");
  //tf_broadcaster_.sendTransform(odom_to_frame_tango_tf);  //after-ROBERTO

  //tf::StampedTransform odom_to_base_link_tf(
   // tf_pose, pose_.header.stamp, fixed_frame_, base_frame_);
  //tf_broadcaster_.sendTransform(odom_to_base_link_tf);

  // *** publish odometry message
  // TODO: add velociyies from ab filters
  
  nav_msgs::Odometry::Ptr odom_message = 
    boost::make_shared<nav_msgs::Odometry>();

  odom_message->header = pose_.header;
   //odom_message->child_frame_id = "tango"; //AFTER-ROBERTO//
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

