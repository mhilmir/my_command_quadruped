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
    MyRobot() : client_("move_base", true) {
        tracked_status_sub_ = nh_.subscribe("/tracked_status", 1, &MyRobot::trackstatusCallback, this);
        center_sub_ = nh_.subscribe("/tracked_center", 1, &MyRobot::centerCallback, this);
        depth_sub_ = nh_.subscribe("/tracked_depth", 1, &MyRobot::depthCallback, this);
        // odom_sub_ = nh_.subscribe("/leg_odom2", 1, &MyRobot::odomCallback, this);
        search_status_sub_ = nh_.subscribe("/search_status", 1, &MyRobot::searchstatusCallback, this);
        goto_status_sub_ = nh_.subscribe("/goto_status", 1, &MyRobot::gotostatusCallback, this);
        location_chosen_sub_ = nh_.subscribe("/location_chosen", 1, &MyRobot::locationchosenCallback, this);
        navman_sub_ = nh_.subscribe("/navman_comm", 1, &MyRobot::navmanCallback, this);

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        simpleCMD_pub_ = nh_.advertise<message_transformer::SimpleCMD>("/simple_cmd", 10);
        navman_pub_ = nh_.advertise<std_msgs::Bool>("/navman_comm", 1);
        nav_status_pub_ = nh_.advertise<std_msgs::String>("/nav_status", 1);
        
        // ROS_INFO("Waiting for action server to start...");
        // client_->waitForServer();  // Optional: wait in constructor
        // ROS_INFO("Action server started.");
        
        frame_width_ = 640;
        center_threshold_ = 40;
        desired_distance_ = 650;      // mm
        distance_threshold_ = 180;
        
        // aligned_time_ = ros::Time(0);
        // aligned_ = false;

        active_ = false;
    }

    double constrain_vel_x(double value) {
        if (value > 0.2)
            return 0.2;
        else if (value < -0.2)
            return -0.2;
        else
            return value;
    }

    void move(double linear_x, double linear_y, double angular_z, double duration){
        ROS_INFO("Robot move for %f seconds, with speed:\n  lin_x:%f lin_y:%f ang_z:%f", duration, linear_x, linear_y, angular_z);

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
        ros::Duration(7.0).sleep();  // small pause
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

    // theta 0 itu depan robot
    void send_goal(double x, double y, double theta, bool search_mode=false) {

        if(search_mode && !search_)
            return;

        ROS_INFO("Waiting for the move_base action server...");
        client_.waitForServer();
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
        client_.sendGoal(goal);

        ros::Rate rate(10);  // 10 Hz loop rate
        while (ros::ok() && !client_.getState().isDone()) {
            ros::spinOnce();
            
            if (search_mode && !search_ && 
                (client_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
                client_.getState() == actionlib::SimpleClientGoalState::PENDING)) {
                    ROS_WARN("Goal canceled due to external signal.");
                    client_.cancelGoal();
                    break;
            }
            rate.sleep();
        }

        if (search_mode && !search_) {
            ROS_INFO("Goal was canceled.");
        } else if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached successfully.");
        } else {
            ROS_WARN("Goal failed or interrupted. State: %s", client_.getState().toString().c_str());
        }
    }

    void approach_sit(){
        ros::Rate rate(10);
        nav_status_msg.data = "Approach And Sit Next to the Object";
        nav_status_pub_.publish(nav_status_msg);
        ros::spinOnce();
        rate.sleep();

        ROS_INFO("ROBOT IS GOING TO SIT NEXT TO THE OBJECT");
        double lin_speed = 0.2;  // m/s
        double ang_speed = 0.30;  // rad/s
        double linx_distance, liny_distance, ang_distance;

        // Method A kiri
        // liny_distance = 0.75;  // meter
        // ang_distance = pi/2;  // rad
        // move(0, 0, -ang_speed, ang_distance/ang_speed);  // 90 degrees turn right
        // move(0, lin_speed, 0, liny_distance/lin_speed);  // going left
        // ros::Duration(2).sleep();
        // Method A kanan
        linx_distance = 0.15;
        liny_distance = 0.65;  // meter
        ang_distance = pi/2;  // rad
        move(0, 0, ang_speed, ang_distance/ang_speed);  // 90 degrees turn left
        move(0, -lin_speed, 0, liny_distance/lin_speed);  // going right
        move(lin_speed, 0, 0, linx_distance/lin_speed);  // go forward little bit
        ros::Duration(2).sleep();
        
        // // Method B
        // linx_distance = 0.60;  // meter
        // liny_distance = 0.15;  // meter
        // ang_distance = pi/2;  // rad
        // move(lin_speed, 0, 0, linx_distance/lin_speed);  // move forward
        // move(0, 0, -ang_speed, ang_distance/ang_speed);  // 90 degrees turn right
        // move(0, lin_speed, 0, liny_distance/lin_speed);  // move left
        // ros::Duration(2).sleep();

        ROS_INFO("Finish, now Sit down");
        simpleCMD_send(0x21010202, 0, 0);  // robot sit/stand
        ros::Duration(3).sleep();
    }

    void approach(){
        ROS_INFO("ROBOT IS APPROACHING THE OBJECT");
        ros::Rate rate(10);  // 10 Hz
        aligned_time_ = ros::Time(0);
        aligned_ = false;
        while (ros::ok()){
            geometry_msgs::Twist vel;

            if (tracked_){
                // ROS_INFO("Center_.x:%f, depth_:%d", center_.x, depth_);

                double x_error = center_.x - (frame_width_ / 2.0);
                if (std::abs(x_error) > center_threshold_){
                    vel.angular.z = -0.001 * x_error;  // Turn towards the object
                    ROS_INFO("vel ang z : %f", vel.angular.z);
                    ROS_INFO("x error: %f", x_error);
                } else{
                    // ROS_INFO("yaw aligned");
                }
                
                int distance_error = desired_distance_ - depth_;
                if (std::abs(distance_error) > distance_threshold_){
                    vel.linear.x = -0.00030 * distance_error;  // Move forward/backward
                    vel.linear.x = constrain_vel_x(vel.linear.x);
                    ROS_INFO("vel lin x : %f", vel.linear.x);
                    ROS_INFO("distance error: %d", distance_error);
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
                        // ROS_INFO("Robot is going to sit next to the object");
                        break;
                    }
                } else{
                    aligned_ = false;
                }
            } else{
                aligned_ = false;
            }

            vel_pub_.publish(vel);

            nav_status_msg.data = "Tracking the Object to Approach It";
            nav_status_pub_.publish(nav_status_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void search(){
        ros::Rate rate(10);  // 10 Hz

        // ambil dari param
        std::string search_points_data = location_chosen_ + "_spoints";
        std::map<std::string, std::vector<double>> search_waypoints = loadWaypoints(search_points_data);
        displayWaypointsInRoom(search_waypoints);
        ROS_INFO("Search points for current location is loaded.");
        ROS_INFO("ROBOT READY TO SEARCH");

        while(ros::ok()){
            if(search_ && !tracked_){
                ROS_INFO("Search the area until object detected");

                for (const auto& pair : search_waypoints) {
                    const std::string& wp_name = pair.first;
                    const std::vector<double>& pose = pair.second;

                    ROS_INFO("[Searching]... Go To %s", wp_name.c_str());
                    // send_goal(pose[0], pose[1], pose[2], true);
                    ros::spinOnce();
                    rate.sleep();
                    if(!search_ || tracked_) break;
                }
                
            }

            if(tracked_ && !search_){
                ROS_INFO("Object determined. Start to track and approach it");
                break;
            }

            nav_status_msg.data = "Robot Searching The Object";
            nav_status_pub_.publish(nav_status_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void goto_location(){
        ros::Rate rate(10);  // 10Hz

        ROS_INFO("Waiting for location input...");
        while (ros::ok() && !goto_){
            nav_status_msg.data = "Waiting For Target Location Input";
            nav_status_pub_.publish(nav_status_msg);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Location target determined !!");
        ROS_INFO("ROBOT GOTO TARGETED LOCATION / ROOM");
        initial_room_ = location_chosen_;

        // ambil dari param
        std::map<std::string, std::vector<double>> goal_waypoints = loadWaypoints(location_chosen_);
        displayWaypointsInRoom(goal_waypoints);
        ROS_INFO("Waypoints Loaded for the respective location");

        nav_status_msg.data = "Go To Target Location";
        nav_status_pub_.publish(nav_status_msg);
        ros::spinOnce();
        rate.sleep();

        // menuju lokasi
        for (const auto& pair : goal_waypoints) {
            const std::string& wp_name = pair.first;
            const std::vector<double>& pose = pair.second;

            ROS_INFO("[Navigating]... Go To %s", wp_name.c_str());
            // send_goal(pose[0], pose[1], pose[2]);
        }
        ROS_INFO("Robot now in %s", location_chosen_.c_str());
    }

    void goto_initial_room(){
        ros::Rate rate(10);  // 10Hz
        nav_status_msg.data = "Go Back to Initial Location or Room";
        nav_status_pub_.publish(nav_status_msg);
        ros::spinOnce();
        rate.sleep();

        // ROS_INFO("ROBOT WILL GO BACK TO THE INITIAL ROOM");

        // // ambil dari param
        // std::map<std::string, std::vector<double>> goal_waypoints = loadWaypoints(initial_room_);
        // displayWaypointsInRoom(goal_waypoints);
        // ROS_INFO("Waypoints Loaded for the respective location");

        // // menuju lokasi
        // for (const auto& pair : goal_waypoints) {
        //     const std::string& wp_name = pair.first;
        //     const std::vector<double>& pose = pair.second;

        //     ROS_INFO("[Navigating]... Go To %s", wp_name.c_str());
        //     // send_goal(pose[0], pose[1], pose[2]);
        // }
        // ROS_INFO("Robot now in %s", initial_room_.c_str());

        // Untuk Sementara
        double lin_speed = 0.2;  // m/s
        double ang_speed = 0.30;  // rad/s
        double linx_distance, liny_distance, ang_distance;
        linx_distance = 1.0;
        ang_distance = pi/2;  // rad
        move(-lin_speed, 0, 0, linx_distance/lin_speed);  // go backward
        move(0, 0, ang_speed, ang_distance/ang_speed);  // 90 degrees turn left
        move(lin_speed, 0, 0, linx_distance/lin_speed);  // go forward little bit
        ros::Duration(2).sleep();

    }

    void spin(){
        ros::Rate rate(10);
        while(ros::ok()){
            
            goto_location();
            search();
            approach();
            approach_sit();
            active_ = false;
            navman_msg_.data = true;
            navman_pub_.publish(navman_msg_);
            ROS_INFO("handoff to arm\n");

            // wait for the arm robot grasp
            ROS_INFO("wait for the arm finish the grasping job");
            while(ros::ok() && (active_==false)){
                // ROS_INFO("...");
                nav_status_msg.data = "Waiting For The Arm to Finish Grasping Job";
                nav_status_pub_.publish(nav_status_msg);
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("arm has been finished the grasping job\n");
            
            ros::Duration(2).sleep();
            simpleCMD_send(0x21010202, 0, 0);  // robot sit or stand
            ros::Duration(3).sleep();
            goto_initial_room();
            ros::Duration(2).sleep();
            simpleCMD_send(0x21010202, 0, 0);  // robot sit or stand
            ros::Duration(3).sleep();
            active_ = false;
            navman_msg_.data = true;
            navman_pub_.publish(navman_msg_);
            ROS_INFO("handoff to arm\n");

            // wait for the arm robot place the object
            ROS_INFO("wait for the arm finish the placement job");
            while(ros::ok() && (active_==false)){
                // ROS_INFO("...");
                nav_status_msg.data = "Waiting For The Arm to Finish Placement Job";
                nav_status_pub_.publish(nav_status_msg);
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("arm has been finished the placement job\n");
            
            ros::Duration(2).sleep();
            simpleCMD_send(0x21010202, 0, 0);  // robot sit or stand
            ros::Duration(3).sleep();
        }
            
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
        // ROS_INFO("DEPTH FROM CB : %d", depth_);
    }

    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //     odom_ = *msg;
    // }

    void searchstatusCallback(const std_msgs::Bool::ConstPtr& msg){
        search_ = msg->data;
    }

    void gotostatusCallback(const std_msgs::Bool::ConstPtr& msg){
        goto_ = msg->data;
    }

    void locationchosenCallback(const std_msgs::String::ConstPtr& msg){
        location_chosen_ = msg->data;
    }

    void navmanCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        active_ = !(msg->data);
    }

    ros::NodeHandle nh_;
    ros::Subscriber tracked_status_sub_, center_sub_, depth_sub_;
    // ros::Subscriber odom_sub_;
    ros::Subscriber search_status_sub_;
    ros::Subscriber location_chosen_sub_;
    ros::Subscriber goto_status_sub_;
    ros::Subscriber navman_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher simpleCMD_pub_;
    ros::Publisher navman_pub_;
    ros::Publisher nav_status_pub_;

    std_msgs::Bool navman_msg_; 
    std_msgs::String nav_status_msg;
    bool active_;  // indicates whether this node should run navigation or wait for manipulation task
    std::string initial_room_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client_;

    geometry_msgs::Point center_;
    // nav_msgs::Odometry odom_;
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
    my_robot.spin();

    return 0;
}
