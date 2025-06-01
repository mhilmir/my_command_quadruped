#include "ros/ros.h"
#include "std_msgs/Int32.h"

int depth_;
void depthCallback(const std_msgs::Int32::ConstPtr& msg){
    depth_ = msg->data;
    ROS_INFO("DEPTH FROM CB : %d", depth_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hello_world_cpp_node");
  ros::NodeHandle nh;

  ros::Subscriber depth_sub_ = nh.subscribe("/tracked_depth", 1, depthCallback);
  
  ROS_INFO("Hello, World! from C++");

  ros::spin();
  return 0;
}
