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

#ifndef MAV_CTRL_INTERFACE_CTRL_INTERFACE_NODELET_H
#define MAV_CTRL_INTERFACE_CTRL_INTERFACE_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mav_ctrl_interface/ctrl_interface.h"

namespace mav
{
  class CtrlInterfaceNodelet : public nodelet::Nodelet
  {
    public:
      virtual void onInit();

    private:
      mav::CtrlInterface * ctrl_interface_;  // FIXME: change to smart pointer
  };
}

#endif // MAV_CTRL_INTERFACE_CTRL_INTERFACE_NODELET_H
