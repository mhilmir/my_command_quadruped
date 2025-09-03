#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

// Global flag
bool search_ = false;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* client_ptr = nullptr;

// Callback
void searchstatusCallback(const std_msgs::Bool::ConstPtr& msg) {
    search_ = msg->data;
}

void send_goal(double x, double y, double theta, bool search_mode=false) {
    if (search_mode && !search_)
        return;

    ROS_INFO("Waiting for the move_base action server...");
    client_ptr->waitForServer();
    ROS_INFO("Got it");

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

    ROS_INFO("Sending goal: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
    client_ptr->sendGoal(goal);

    ros::Rate rate(10);
    while (ros::ok() && !client_ptr->getState().isDone()) {
        ros::spinOnce();

        if (search_mode && !search_ &&
            (client_ptr->getState() == actionlib::SimpleClientGoalState::ACTIVE ||
             client_ptr->getState() == actionlib::SimpleClientGoalState::PENDING)) {
            ROS_WARN("Goal canceled due to external signal.");
            client_ptr->cancelGoal();
            break;
        }
        rate.sleep();
    }

    if (search_mode && !search_) {
        ROS_INFO("Goal was canceled.");
    } else if (client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached successfully.");
    } else {
        ROS_WARN("Goal failed or interrupted. State: %s",
                 client_ptr->getState().toString().c_str());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_setpoint_cancel_node");
    ros::NodeHandle nh;

    ros::Subscriber search_sub = nh.subscribe("/search_status", 10, searchstatusCallback);

    // Create action client AFTER init()
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
    client_ptr = &client;

    ros::Duration(1.0).sleep();  // Let subscriber connect

    // Example goals
    send_goal(3.82, 2.77, 1.5708);       // Depan Raisa
    send_goal(0.687, 0.272, 0.0, true);  // Posisi awal

    return 0;
}