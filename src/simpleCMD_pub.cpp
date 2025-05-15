#include <ros/ros.h>
#include <message_transformer/SimpleCMD.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simpleCMD_pub_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<message_transformer::SimpleCMD>("/simple_cmd", 10);

    // Wait for publisher to establish connection
    ros::Duration(0.5).sleep();

    message_transformer::SimpleCMD msg;
    msg.cmd_code = 0x21010202;
    msg.cmd_value = 0;
    msg.type = 0;

    pub.publish(msg);
    ROS_INFO("Published SimpleCMD: code=%d, value=%d, type=%d", msg.cmd_code, msg.cmd_value, msg.type);

    // Give ROS time to send the message before shutting down
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    return 0;
}
