#include "quad_joy_teleop/quad_joy_teleop.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "QuadJoyTeleop");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  QuadJoyTeleop quad_joy_teleop(nh, nh_private);
  ros::spin();
  return 0;
}
