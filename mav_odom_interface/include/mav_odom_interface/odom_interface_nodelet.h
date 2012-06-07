#ifndef MAV_ODOM_INTERFACE_ODOM_INTERFACE_NODELET_H
#define MAV_ODOM_INTERFACE_ODOM_INTERFACE_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mav_odom_interface/odom_interface.h"

namespace mav {

class OdomInterfaceNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit ();

  private:
    OdomInterface * odom_interface_;  // FIXME: change to smart pointer
};

} // namespace mav

#endif // QUAD_POSE_EST_QUAD_POSE_EST_NODELET_H
