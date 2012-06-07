/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010,
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of CCNY Robotics Lab nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H
#define LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <boost/thread/mutex.hpp>

#include <mav_msgs/Height.h>

namespace mav
{

class LaserHeightEstimation
{
  public:

    LaserHeightEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~LaserHeightEstimation();

  private:

    // **** ros-related variables

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber scan_subscriber_;
    ros::Publisher  floor_publisher_;
    ros::Publisher  height_to_base_publisher_;
    ros::Publisher  height_to_footprint_publisher_;
    tf::TransformListener tf_listener_;

    // **** state variables

    boost::mutex mutex_;

    bool initialized_;
    bool first_time_;
    double floor_height_;
    double prev_height_;

    tf::Transform base_to_laser_;
    tf::Transform base_to_footprint_;
    tf::Transform world_to_base_;

    sensor_msgs::Imu latest_imu_msg_;

    ros::Time last_update_time_;

    // **** parameters

    std::string world_frame_;
    std::string base_frame_;
    std::string footprint_frame_;
    int min_values_;
    double max_stdev_;
    double max_height_jump_;
    bool use_imu_;
    bool use_segmentation_;
    double bin_size_;         // for histogram segmentation

    // **** member functions

    void scanCallback (const sensor_msgs::LaserScanPtr& scan_msg);
    void imuCallback  (const sensor_msgs::ImuPtr&       imu_msg);
    bool setBaseToLaserTf(const sensor_msgs::LaserScanPtr& scan_msg);
    void getStats(const std::vector<double> values, double& ave, double& stdev);

    void histogramSegmentation(
      const std::vector<double>& input,
      std::vector<double>& output);

    void kMeansSegmentation(
      const std::vector<double>& input,
      std::vector<double>& output);

    bool kMeansSegmentation(
      const std::vector<double>& input,
      std::vector<bool>& mask,
      double& mean_a, double& mean_b,
      int& count_a, int& count_b);
};

}; // namespace mav

#endif // LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H

