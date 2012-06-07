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

#ifndef AB_FILTER_AB_FILTER_HEIGHT_H
#define AB_FILTER_AB_FILTER_HEIGHT_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

#include <mav_msgs/Height.h>

namespace mav
{

class ABFilterHeight
{
  typedef mav_msgs::Height HeightMsg;

  private:

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber height_subscriber_;

    ros::Publisher publisher_;

    // **** state variables    

    bool initialized_;
    ros::Time last_update_time_;

    HeightMsg height_;

    // **** parameters
  
    double alpha_;
    double beta_;

    // **** member functions

    void initializeParams();

    void heightCallback(const HeightMsg::ConstPtr height_msg);
    void publishHeight();


  public:

    ABFilterHeight(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~ABFilterHeight();
};

} // end namespace mav

#endif // AB_FILTER_AB_FILTER_HEIGHT_H
