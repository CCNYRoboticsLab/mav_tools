#ifndef QUAD_JOY_TELEOP_QUAD_JOY_TELEOP_NODELET_H
#define QUAD_JOY_TELEOP_QUAD_JOY_TELEOP_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "quad_joy_teleop/quad_joy_teleop.h"

class QuadJoyTeleopNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit ();

  private:
    QuadJoyTeleop * quad_joy_teleop_;  // FIXME: change to smart pointer
};


#endif // QUAD_JOY_TELEOP_QUAD_JOY_TELEOP_NODELET_H
