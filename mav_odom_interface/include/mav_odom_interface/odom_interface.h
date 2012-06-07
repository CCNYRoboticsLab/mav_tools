#ifndef MAV_ODOM_INTERFACE_ODOM_INTERFACE_H
#define MAV_ODOM_INTERFACE_ODOM_INTERFACE_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mav_msgs/Height.h>

namespace mav {

class OdomInterface
{

// for campatibility b/n ROS Electric and Fuerte
#if ROS_VERSION_MINIMUM(1, 8, 0)
  typedef tf::Matrix3x3 MyMatrix;
#else
  typedef btMatrix3x3 MyMatrix;
#endif

  public:

    OdomInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~OdomInterface();

  private:

    typedef geometry_msgs::PoseStamped  PoseStamped;
    typedef geometry_msgs::TwistStamped TwistStamped;

    // **** ros-related variables

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher  pose_publisher_;
    ros::Publisher  odom_publisher_;

    ros::Subscriber laser_pose_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber height_subscriber_;
    tf::TransformBroadcaster tf_broadcaster_;

    // **** parameters

    std::string base_frame_;
    std::string fixed_frame_;

    // **** state variables

    boost::mutex pose_mutex_;

    PoseStamped pose_;

    // **** member functions

    void laserPoseCallback(const PoseStamped::ConstPtr& laser_pose_msg);
    void imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg);
    void heightCallback (const mav_msgs::Height::ConstPtr& height_msg);

    void publishPose();
};

} // namespace mav

#endif // MAV_ODOM_INTERFACE_ODOM_INTERFACE_H
