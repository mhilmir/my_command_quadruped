#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void move(ros::Publisher& pub, double linear_x, double linear_y, double duration)
{
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = linear_y;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    ros::Rate rate(10); // 10 Hz
    int ticks = duration * 10; // number of ticks to publish

    for (int i = 0; i < ticks; ++i)
    {
        pub.publish(cmd);
        rate.sleep();
    }

    // Stop after moving
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    pub.publish(cmd);
    ros::Duration(1.0).sleep(); // small pause
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_square_node");
    ros::NodeHandle nh;

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Duration(2.0).sleep(); // wait for publisher connection

    double speed = 0.2; // m/s
    double distance = 1.0; // meters
    double duration = distance / speed; // seconds

    // Move in a square
    move(cmd_pub,  speed,  0.0, duration); // forward
    move(cmd_pub,  0.0, -speed, duration); // right
    move(cmd_pub, -speed,  0.0, duration); // backward
    move(cmd_pub,  0.0,  speed, duration); // left

    ROS_INFO("Finished moving in a square.");
    return 0;
}
