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

#include "ab_filter/ab_filter_height.h"

namespace mav
{

ABFilterHeight::ABFilterHeight(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false)
{
  ROS_INFO("Starting ABFilterHeight"); 

  ros::NodeHandle nh_mav (nh_, "mav");

  // **** initialize vaiables

  // **** get parameters

  initializeParams();

  // *** register publishers

  publisher_  = nh_mav.advertise<HeightMsg>(
    "laser_height_f", 10);

  // **** register subscribers

  height_subscriber_ = nh_mav.subscribe(
    "laser_height", 10, &ABFilterHeight::heightCallback, this);
}

ABFilterHeight::~ABFilterHeight()
{
  ROS_INFO("Destroying ABFilterHeight"); 

}

void ABFilterHeight::heightCallback(const HeightMsg::ConstPtr height_msg)
{
  //ros::Time time = ros::Time::now();
  ros::Time time = height_msg->header.stamp;

  if(!initialized_)
  {
    // first message
  
    initialized_ = true;

    height_.height = height_msg->height;
    height_.climb  = 0.0;
  }
  else
  {
    double dt = (time - last_update_time_).toSec();

    double z_pred = (height_.height + dt * height_.climb);

    double r_z = height_msg->height - z_pred;

    height_.height = z_pred + alpha_ * r_z;
    height_.climb += beta_  * r_z   / dt;
  }

  last_update_time_ = time;
  height_.header.stamp = time;
  publishHeight();
}

void ABFilterHeight::publishHeight()
{
  HeightMsg::Ptr msg;
  msg = boost::make_shared<HeightMsg>(height_);
  publisher_.publish(msg);
}

void ABFilterHeight::initializeParams()
{
  if (!nh_private_.getParam ("alpha", alpha_))
    alpha_ = 0.9;
  if (!nh_private_.getParam ("beta", beta_))
    beta_ = 0.9;
}

} // end namespace asctec
