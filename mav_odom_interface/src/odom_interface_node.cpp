#include "mav_odom_interface/odom_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OdomInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  mav::OdomInterface odom_interface(nh, nh_private);
  ros::spin();
  return 0;
}
