/*
 *  Asctec Autopilot Interface
 *  Copyright (C) 2011, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
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

#ifndef FLYER_INTERFACE_FLYER_INTERFACE_NODELET_H
#define FLYER_INTERFACE_FLYER_INTERFACE_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "flyer_interface/flyer_interface.h"

namespace mav
{
  class FlyerInterfaceNodelet : public nodelet::Nodelet
  {
    public:
      virtual void onInit ();

    private:
      FlyerInterface * flyer_interface_;  // FIXME: change to smart pointer
  };
}

#endif // FLYER_INTERFACE_FLYER_INTERFACE_NODELET_H
