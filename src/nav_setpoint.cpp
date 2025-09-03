#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

// theta 0 itu depan robot
void sendGoal(double x, double y, double theta) {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
    ROS_INFO("Waiting for the move_base action server...");
    client.waitForServer();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
    goal.target_pose.pose.orientation.x = quat.x();
    goal.target_pose.pose.orientation.y = quat.y();
    goal.target_pose.pose.orientation.z = quat.z();
    goal.target_pose.pose.orientation.w = quat.w();

    ROS_INFO("Sending goal : x=%f, y=%f, theta=%f", x, y, theta);
    client.sendGoal(goal);
    client.waitForResult();

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached successfully.");
    } else {
        ROS_WARN("Failed to reach goal.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_setpoint_node");
    ros::NodeHandle nh;

    // TW2 Lt9
    // sendGoal(3.28, 0.665, 0.0);  // kira2 tengah depan ruangan 
    // sendGoal(3.62, 1.6, 0.0);  // depan meja solder
    // sendGoal(0.888, 0.626, 0.0);  // kira2 tengah belakang ruangan

    // TF
    sendGoal(3.82, 2.77, 1.5708);  // Depan Raisa
    sendGoal(0.687, 0.272, 0.0);  // Posisi awal

    return 0;
}
