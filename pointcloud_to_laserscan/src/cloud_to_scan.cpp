/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "dynamic_reconfigure/server.h"
#include "pointcloud_to_laserscan/CloudScanConfig.h"
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

namespace pointcloud_to_laserscan
{
class CloudToScan : public nodelet::Nodelet
{
  typedef geometry_msgs::PoseStamped PoseMsg;

public:
  //Constructor
  CloudToScan(): min_height_(0.30),
                 max_height_(1.0),
                 angle_min_(-M_PI/2),
                 angle_max_(M_PI/2),
                 angle_increment_(M_PI/180.0/2.0),
                 scan_time_(1.0/30.0),
                 range_min_(0.45),
                 range_max_(10.0)
  {
  };

  ~CloudToScan()
  {
    delete srv_;
  }

private:
  typedef pcl::PointXYZ           PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  boost::mutex connect_mutex_;
  // Dynamic reconfigure server
  dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>* srv_;

  virtual void onInit()
  {
    nh_ = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    initialized_ = false;

    // set initial orientation to 0
    world_to_cloud_.setIdentity();
    ortho_to_cloud_.setIdentity();

    // **** parameters
    if (!private_nh.getParam ("fixed_frame", world_frame_))
      world_frame_ = "/world";
    if (!private_nh.getParam ("base_frame", base_frame_))
      base_frame_ = "/base_link";
    if (!private_nh.getParam ("ortho_frame", ortho_frame_))
      ortho_frame_ = "/base_ortho";

    if (!private_nh.getParam ("use_pose", use_pose_))
      use_pose_ = true;
    if (!private_nh.getParam ("publish_tf", publish_tf_))
      publish_tf_ = true;

    private_nh.getParam("min_height", min_height_);
    private_nh.getParam("max_height", max_height_);

    private_nh.getParam("angle_min", angle_min_);
    private_nh.getParam("angle_max", angle_max_);
    private_nh.getParam("angle_increment", angle_increment_);
    private_nh.getParam("scan_time", scan_time_);
    private_nh.getParam("range_min", range_min_);
    private_nh.getParam("range_max", range_max_);

    range_min_sq_ = range_min_ * range_min_;

    srv_ = new dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>(private_nh);
    dynamic_reconfigure::Server<pointcloud_to_laserscan::CloudScanConfig>::CallbackType f = boost::bind(&CloudToScan::reconfigure, this, _1, _2);
    srv_->setCallback(f);

    // Lazy subscription to point cloud topic
    ros::AdvertiseOptions scan_ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      "scan", 10,
      boost::bind( &CloudToScan::connectCB, this),
      boost::bind( &CloudToScan::disconnectCB, this), ros::VoidPtr(), nh_.getCallbackQueue());

    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    laser_pub_ = nh_.advertise(scan_ao);
  };

  void connectCB() {
      boost::lock_guard<boost::mutex> lock(connect_mutex_);
      if (laser_pub_.getNumSubscribers() > 0) {
          NODELET_DEBUG("Connecting to point cloud topic.");
          cloud_subscriber_ = nh_.subscribe<PointCloudT>("cloud", 10, &CloudToScan::cloudCallback, this);

          if (use_pose_)
          {
            pose_subscriber_ = nh_.subscribe(
              "pose", 10, &CloudToScan::poseCallback, this);
          }
      }
  }

  void disconnectCB() {
      boost::lock_guard<boost::mutex> lock(connect_mutex_);
      if (laser_pub_.getNumSubscribers() == 0) {
          NODELET_DEBUG("Unsubscribing from point cloud topic.");
          cloud_subscriber_.shutdown();
          NODELET_DEBUG("Unsubscribing from base topic.");
          pose_subscriber_.shutdown();
      }
  }

  void reconfigure(pointcloud_to_laserscan::CloudScanConfig &config, uint32_t level)
  {
    use_pose_ = config.use_pose;
    publish_tf_ = config.publish_tf;
    min_height_ = config.min_height;
    max_height_ = config.max_height;
    angle_min_ = config.angle_min;
    angle_max_ = config.angle_max;
    angle_increment_ = config.angle_increment;
    scan_time_ = config.scan_time;
    range_min_ = config.range_min;
    range_max_ = config.range_max;

    range_min_sq_ = range_min_ * range_min_;
  }

  void cloudCallback(const PointCloudT::ConstPtr& cloud_in)
  {
    if(!initialized_)
    {
      initialized_ = getBaseToCloudTf(cloud_in);

      if (initialized_)
        {
         // TODO: ??? createCache(cloud_in);
        }
      else return;
    }

    bool has_c2o_tf = false;

    if(!use_pose_)
    {
      // obtain transform between fixed and base frame
      tf::StampedTransform world_to_base_tf;
      try
      {
        has_c2o_tf = tf_listener_.waitForTransform (
          world_frame_, base_frame_, cloud_in->header.stamp, ros::Duration(0.1));
        tf_listener_.lookupTransform (
          world_frame_, base_frame_, cloud_in->header.stamp, world_to_base_tf);
      }
      catch (tf::TransformException& ex)
      {
        // transform unavailable - skip scan
        ROS_WARN("Skipping cloud %s", ex.what());
        return;
      }

      // calculate world to ortho frame transform
      tf::Transform world_to_ortho;
      getOrthoTf(world_to_base_tf, world_to_ortho);
    }


//    if(has_c2o_tf)
    {
      // Apply the Transform for the cloud into the output frame
      PointCloudT::Ptr cloud_output =
        boost::shared_ptr<PointCloudT>(new PointCloudT());

      pcl::transformPointCloud(*cloud_in, *cloud_output, eigenFromTf(ortho_to_cloud_));
      cloud_output->header.frame_id = ortho_frame_;

      // Setup virtual laser
      sensor_msgs::LaserScanPtr laser_output(new sensor_msgs::LaserScan());
      laser_output->header = cloud_in->header;
      laser_output->header.frame_id = ortho_frame_; // Set output frame. Point clouds come from "optical" frame, scans come from corresponding ortho frame
      laser_output->angle_min = angle_min_;
      laser_output->angle_max = angle_max_;
      laser_output->angle_increment = angle_increment_;
      laser_output->time_increment = 0.0;
      laser_output->scan_time = scan_time_;
      laser_output->range_min = range_min_;
      laser_output->range_max = range_max_;

      uint32_t ranges_size = std::ceil((laser_output->angle_max - laser_output->angle_min) / laser_output->angle_increment);
      laser_output->ranges.assign(ranges_size, laser_output->range_max + 1.0);

      for (PointCloudT::const_iterator it = cloud_output->begin(); it != cloud_output->end(); ++it)
      {
        const float &x = it->x;
        const float &y = it->y;
        const float &z = it->z;

        if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
        {
          NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
          continue;
        }

        if (z < max_height_ && z > min_height_)
        {
          double range_sq = y*y+x*x;
          if (range_sq < range_min_sq_) {
            NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
          }

          double angle = atan2(y, x);
          if (angle < laser_output->angle_min || angle > laser_output->angle_max)
          {
            NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, laser_output->angle_min, laser_output->angle_max);
            continue;
          }
          int index = (angle - laser_output->angle_min) / laser_output->angle_increment;


          if (laser_output->ranges[index] * laser_output->ranges[index] > range_sq)
            laser_output->ranges[index] = sqrt(range_sq);
        }
        else
        {
          NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", x, min_height_, max_height_);
          continue;
        }
      }

      laser_pub_.publish(laser_output);
    }
  }


  void poseCallback(const PoseMsg::ConstPtr& pose_msg)
  {
    // obtain world to base frame transform from the pose message
    tf::Transform world_to_base;
    tf::poseMsgToTF(pose_msg->pose, world_to_base);

    // calculate world to ortho frame transform
    tf::Transform world_to_ortho;
    getOrthoTf(world_to_base, world_to_ortho);

    if (publish_tf_)
    {
      tf::StampedTransform world_to_ortho_tf(
        world_to_ortho, pose_msg->header.stamp, world_frame_, ortho_frame_);
      tf_broadcaster_.sendTransform(world_to_ortho_tf);
    }

    // calculate ortho to cloud tf, and save it for when scans arrive
    world_to_cloud_ = world_to_base * base_to_cloud_;
    ortho_to_cloud_ = world_to_ortho.inverse() * world_to_cloud_;
  }

  void getOrthoTf(const tf::Transform& world_to_base, tf::Transform& world_to_ortho)
  {
    const tf::Vector3&    w2b_o = world_to_base.getOrigin();
    const tf::Quaternion& w2b_q = world_to_base.getRotation();

    tf::Vector3 wto_o(w2b_o.getX(), w2b_o.getY(), 0.0);
    tf::Quaternion wto_q = tf::createQuaternionFromYaw(tf::getYaw(w2b_q));

    world_to_ortho.setOrigin(wto_o);
    world_to_ortho.setRotation(wto_q);
  }

  bool getBaseToCloudTf (const PointCloudT::ConstPtr& cloud_msg)
  {
    tf::StampedTransform base_to_cloud_tf;
    try
    {
      tf_listener_.waitForTransform(
        base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(1.0));
      tf_listener_.lookupTransform (
        base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, base_to_cloud_tf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("CloudToScan: Could not get initial base-to-cloud transform(%s)", ex.what());
      return false;
    }
    base_to_cloud_ = base_to_cloud_tf;

    return true;
  }

  Eigen::Matrix4f eigenFromTf(const tf::Transform& tf)
  {
     Eigen::Matrix4f out_mat;

     double mv[12];
     tf.getBasis().getOpenGLSubMatrix(mv);

     tf::Vector3 origin = tf.getOrigin();

     out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
     out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
     out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

     out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
     out_mat (0, 3) = origin.x ();
     out_mat (1, 3) = origin.y ();
     out_mat (2, 3) = origin.z ();

     return out_mat;
  }

  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_, range_min_sq_;
  bool use_pose_;
  bool publish_tf_;

  bool initialized_;  ///< state variable

  std::string world_frame_;
  std::string base_frame_;
  std::string ortho_frame_;

  ros::NodeHandle nh_;
  ros::Publisher laser_pub_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber pose_subscriber_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  tf::Transform base_to_cloud_; // static, cached
  tf::Transform world_to_cloud_; ///< Obtained with world_to_base * base_to_cloud_;
  tf::Transform ortho_to_cloud_; ///< Obtained with world_to_ortho.inverse() * world_to_cloud_;

};

PLUGINLIB_DECLARE_CLASS(pointcloud_to_laserscan, CloudToScan, pointcloud_to_laserscan::CloudToScan, nodelet::Nodelet);
}
