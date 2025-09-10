#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "check_nav_pose_node");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while (nh.ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("map", "base_link", ros::Time(0), transform);

            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            double z = transform.getOrigin().z();

            double roll, pitch, yaw;
            tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

            ROS_INFO("Pose in map frame -> x: %.2f, y: %.2f, yaw: %.2f", x, y, yaw);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        rate.sleep();
    }
}