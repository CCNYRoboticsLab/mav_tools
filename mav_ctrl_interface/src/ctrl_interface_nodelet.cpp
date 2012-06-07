/*
 *  MAV Ctrl Interface
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

#include "mav_ctrl_interface/ctrl_interface_nodelet.h"

typedef mav::CtrlInterfaceNodelet CtrlInterfaceNodelet;

PLUGINLIB_DECLARE_CLASS (mav_ctrl_interface, CtrlInterfaceNodelet, 
                         CtrlInterfaceNodelet, nodelet::Nodelet);

void mav::CtrlInterfaceNodelet::onInit ()
{
  NODELET_INFO("Initializing CtrlInterfaceNodelet Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  ctrl_interface_ = new mav::CtrlInterface(nh, nh_private);  
}
