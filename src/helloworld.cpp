#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hello_world_cpp_node");
  ros::NodeHandle nh;
  
  ROS_INFO("Hello, World! from C++");

  ros::spinOnce();
  return 0;
}
