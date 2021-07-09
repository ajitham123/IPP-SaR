#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <glog/logging.h>
#include <mav_trajectory_generation_ros/trajectory_sampling.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mav_msgs/conversions.h>

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_message)
{
  std::ofstream f;
  f.open("/home/ajith/Desktop/ajith_SaR/tmplanner/tmplanner_discrete/debug/loon_recordings/odometry.dat",std::ofstream::app);
  CHECK(f.is_open()) << "Error opening file!";
  ROS_INFO("I heard: [%f,%f,%f]", odometry_message->pose.pose.position.x,odometry_message->pose.pose.position.y,odometry_message->pose.pose.position.z);
  f<<odometry_message->pose.pose.position.x<<","<<odometry_message->pose.pose.position.y<<","<<odometry_message->pose.pose.position.z<<";\n";
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_listener");
	ros::NodeHandle n;
	ros::Subscriber sub_odometry = n.subscribe("/loon/msf_core/odometry", 3, odometryCallback);
	ros::spin();
	return 0;

}