#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <message_transformer/SimpleCMD.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <string>
#include <map>

#define pi 3.14159265358979323846
#define S_TWO_PI 6.283185307179586
#define S_THREEHALF_PI 4.71238898038

class MyRobot{
public:
    MyRobot(){
        tracked_status_sub_ = nh_.subscribe("/tracked_status", 1, &MyRobot::trackstatusCallback, this);
        center_sub_ = nh_.subscribe("/tracked_center", 1, &MyRobot::centerCallback, this);
        depth_sub_ = nh_.subscribe("/tracked_depth", 1, &MyRobot::depthCallback, this);
        odom_sub_ = nh_.subscribe("/leg_odom2", 1, &MyRobot::odomCallback, this);
        search_status_sub_ = nh_.subscribe("/search_status", 1, &MyRobot::searchstatusCallback, this);
        goto_status_sub_ = nh_.subscribe("/goto_status", 1, &MyRobot::gotostatusCallback, this);
        location_chosen_sub_ = nh_.subscribe("/location_chosen", 1, &MyRobot::locationchosenCallback, this);

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        simpleCMD_pub_ = nh_.advertise<message_transformer::SimpleCMD>("/simple_cmd", 10);

        frame_width_ = 640;
        center_threshold_ = 40;
        desired_distance_ = 650;      // mm
        distance_threshold_ = 150;

        aligned_time_ = ros::Time(0);
        aligned_ = false;
    }

    void move(double linear_x, double linear_y, double angular_z, double duration){
        geometry_msgs::Twist vel;
        vel.linear.x = linear_x;
        vel.linear.y = linear_y;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = angular_z;

        ros::Rate rate(10); // 10 Hz
        int ticks = duration * 10; // number of ticks to publish

        for (int i = 0; i < ticks; ++i){
            vel_pub_.publish(vel);
            rate.sleep();
        }

        // Stop after moving
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.angular.z = 0.0;
        vel_pub_.publish(vel);
        ros::Duration(0.5).sleep(); // small pause
    }

    void simpleCMD_send(int cmd_code, int cmd_value, int type){
        message_transformer::SimpleCMD msg;
        msg.cmd_code = cmd_code;
        msg.cmd_value = cmd_value;
        msg.type = type;

        simpleCMD_pub_.publish(msg);
        ROS_INFO("Published SimpleCMD: code=%d, value=%d, type=%d", msg.cmd_code, msg.cmd_value, msg.type);
        ros::Duration(0.5).sleep();  // small pause
    }

    std::map<std::string, std::vector<double>> loadWaypoints(const std::string& room) {
        std::map<std::string, std::vector<double>> room_waypoints;

        XmlRpc::XmlRpcValue room_data;
        if (!nh_.getParam(room, room_data)) {
            ROS_WARN("No waypoints found for room: %s", room.c_str());
            return room_waypoints;
        }

        for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = room_data.begin(); it != room_data.end(); ++it) {
            std::string wp = it->first;
            XmlRpc::XmlRpcValue wp_data = it->second;

            std::vector<double> pose;
            for (int i = 0; i < wp_data.size(); ++i) {
                pose.push_back(static_cast<double>(wp_data[i]));
            }

            std::string wp_name = room + "_" + wp;
            room_waypoints[wp_name] = pose;

            // ROS_INFO("%s: x=%f, y=%f, theta=%f", wp_name.c_str(), pose[0], pose[1], pose[2]);
        }

        return room_waypoints;
    }

    void displayWaypointsInRoom(const std::map<std::string, std::vector<double>>& room_waypoints) {
        for (const auto& pair : room_waypoints) {
            const std::string& wp_name = pair.first;
            const std::vector<double>& pose = pair.second;

            ROS_INFO("%s: x=%f, y=%f, theta=%f", wp_name.c_str(), pose[0], pose[1], pose[2]);
        }
    }

    void approach_sit(){
        double lin_speed = 0.2;  // m/s
        double ang_speed = 0.30;  // rad/s
        // double linx_distance = 0.15;  // meter
        double liny_distance = 0.75;  // meter
        double ang_distance = pi/2;  // rad
        move(0, 0, -ang_speed, ang_distance/ang_speed);  // 90 degrees turn right
        // move(-lin_speed, 0, 0, linx_distance/lin_speed);  // move backward little bit
        move(0, lin_speed, 0, liny_distance/lin_speed);  // going left little bit
        ros::Duration(2).sleep();
        simpleCMD_send(0x21010202, 0, 0);  // robot sit/stand

        ROS_INFO("Finish, now Sit down");
    }

    void approach(){
        ros::Rate rate(10);  // 10 Hz
        ROS_INFO("Object decided, try to approach the object");
        while (ros::ok()){
            geometry_msgs::Twist vel;

            if (tracked_){
                double x_error = center_.x - (frame_width_ / 2.0);
                if (std::abs(x_error) > center_threshold_){
                    vel.angular.z = -0.002 * x_error;  // Turn towards the object
                    ROS_INFO("x error: %f", x_error);
                } else{
                    // ROS_INFO("yaw aligned");
                }
                
                int distance_error = desired_distance_ - depth_;
                if (std::abs(distance_error) > distance_threshold_){
                    vel.linear.x = -0.00025 * distance_error;  // Move forward/backward
                    ROS_INFO("distance error: %f", distance_error);
                } else{
                    // ROS_INFO("distance ok");
                }



                bool yaw_aligned = std::abs(x_error) < center_threshold_;
                bool distance_ok = std::abs(distance_error) < distance_threshold_;
                if(yaw_aligned && distance_ok){
                    if(!aligned_){
                        ROS_INFO("yaw aligned and distance is ok");
                        aligned_time_ = ros::Time::now();
                        aligned_ = true;
                    } else if((ros::Time::now() - aligned_time_).toSec() >= 4.0){
                        ROS_INFO("Robot is going to Approach..");
                        break;
                    }
                } else{
                    aligned_ = false;
                }
            } else{
                aligned_ = false;
            }

            vel_pub_.publish(vel);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void search(){
        ros::Rate rate(10);  // 10 Hz
        while(ros::ok()){
            if(search_ && !tracked_){
                ROS_INFO("Search the area until object detected");

                // // rangkaian move() (belum ada NAV)
                // double lin_speed = 0.2;  // m/s
                // double ang_speed = 0.30;  // rad/s
                // double linx_distance = 2.5;  // meter
                // move(lin_speed, 0, 0, linx_distance / lin_speed, true);  // forward 2.5 m
                // move(0, 0, -ang_speed, (pi) / ang_speed, true);  // yaw 180
                // move(0, 0, ang_speed, (pi/2) / ang_speed, true);  // yaw 90 cc

                // move(lin_speed, 0, 0, linx_distance / lin_speed, true);  // repeat
                // move(0, 0, -ang_speed, (pi) / ang_speed, true);
                // move(0, 0, ang_speed, (pi/2) / ang_speed, true);

                // move(lin_speed, 0, 0, linx_distance / lin_speed, true);  // repeat
                // move(0, 0, -ang_speed, (pi) / ang_speed, true);
                // move(0, 0, ang_speed, (pi/2) / ang_speed, true);

                // move(lin_speed, 0, 0, linx_distance / lin_speed, true);  // repeat
                // move(0, 0, -ang_speed, (pi) / ang_speed, true);
                // move(0, 0, ang_speed, (pi/2) / ang_speed, true);

                // bikin disini

                
            }

            if(tracked_ && !search_){
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

    // theta 0 itu depan robot
    void send_goal(double x, double y, double theta) {
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

    void goto_location(){
        ros::Rate rate(10);  // 10Hz

        ROS_INFO("Waiting for location input...");
        while (ros::ok() && !goto_){

            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("Location target determined !!");

        // ambil dari param
        std::map<std::string, std::vector<double>> room_waypoints = loadWaypoints(location_chosen_);
        displayWaypointsInRoom(room_waypoints);

        // menuju lokasi
        // send_goal();   
    }

private:
    void trackstatusCallback(const std_msgs::Bool::ConstPtr& msg){
        tracked_ = msg->data;
    }

    void centerCallback(const geometry_msgs::Point::ConstPtr& msg){
        center_ = *msg;
    }

    void depthCallback(const std_msgs::Int32::ConstPtr& msg){
        depth_ = msg->data;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        odom_ = *msg;
    }

    void searchstatusCallback(const std_msgs::Bool::ConstPtr& msg){
        search_ = msg->data;
    }

    void gotostatusCallback(const std_msgs::Bool::ConstPtr& msg){
        goto_ = msg->data;
    }

    void locationchosenCallback(const std_msgs::String::ConstPtr& msg){
        location_chosen_ = msg->data;
    }

    ros::NodeHandle nh_;
    ros::Subscriber tracked_status_sub_, center_sub_, depth_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber search_status_sub_;
    ros::Subscriber location_chosen_sub_;
    ros::Subscriber goto_status_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher simpleCMD_pub_;

    geometry_msgs::Point center_;
    nav_msgs::Odometry odom_;
    bool search_ = false;
    bool goto_ = false;
    int depth_ = 0;
    bool tracked_ = false;
    std::string location_chosen_;

    int frame_width_;
    int center_threshold_;
    int desired_distance_;
    int distance_threshold_;

    ros::Time aligned_time_;
    bool aligned_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "semi_autonomous_control_node");
    MyRobot my_robot;

    my_robot.goto_location();
    // my_robot.search();
    // my_robot.approach();
    // my_robot.approach_sit();
    
    return 0;
}
