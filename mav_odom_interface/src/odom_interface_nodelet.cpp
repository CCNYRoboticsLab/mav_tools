#include "mav_odom_interface/odom_interface_nodelet.h"

typedef mav::OdomInterfaceNodelet OdomInterfaceNodelet ;

PLUGINLIB_DECLARE_CLASS (mav_odom_interface, OdomInterfaceNodelet , 
                         OdomInterfaceNodelet , nodelet::Nodelet);

void OdomInterfaceNodelet ::onInit()
{
  NODELET_INFO("Initializing OdomInterface Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  odom_interface_ = new OdomInterface(nh, nh_private);  
}
