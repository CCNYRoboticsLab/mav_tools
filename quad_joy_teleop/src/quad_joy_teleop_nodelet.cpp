#include "quad_joy_teleop/quad_joy_teleop_nodelet.h"

PLUGINLIB_DECLARE_CLASS (quad_joy_teleop, QuadJoyTeleopNodelet, QuadJoyTeleopNodelet, nodelet::Nodelet);

void QuadJoyTeleopNodelet::onInit ()
{
  NODELET_INFO("Initializing QuadJoyTeleop Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  quad_joy_teleop_ = new QuadJoyTeleop(nh, nh_private);  
}
