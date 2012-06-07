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

#include "ab_filter/ab_filter_pose_nodelet.h"

typedef mav::ABFilterPoseNodelet ABFilterPoseNodelet;

PLUGINLIB_DECLARE_CLASS (ab_filter, ABFilterPoseNodelet, 
                         ABFilterPoseNodelet, nodelet::Nodelet);

void mav::ABFilterPoseNodelet::onInit ()
{
  NODELET_INFO("Initializing ABFilterPose Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  ab_filter_ = new mav::ABFilterPose(nh, nh_private);  
}
