#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <message_transformer/SimpleCMD.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#define pi 3.14159265358979323846
#define S_TWO_PI 6.283185307179586
#define S_THREEHALF_PI 4.71238898038

class MyRobot{
public:
    MyRobot(){
        ros::NodeHandle nh;

        tracked_status_sub_ = nh.subscribe("/tracked_status", 1, &MyRobot::trackstatusCallback, this);
        center_sub_ = nh.subscribe("/tracked_center", 1, &MyRobot::centerCallback, this);
        depth_sub_ = nh.subscribe("/tracked_depth", 1, &MyRobot::depthCallback, this);
        odom_sub_ = nh.subscribe("/leg_odom2", 1, &MyRobot::odomCallback, this);
        search_status_sub_ = nh.subscribe("/search_status", 1, &MyRobot::searchstatusCallback, this);

        vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        simpleCMD_pub_ = nh.advertise<message_transformer::SimpleCMD>("/simple_cmd", 10);

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
    // fungsi move() untuk search object
    void move(double linear_x, double linear_y, double angular_z, double duration, bool is_search){
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
            if(tracked_ || !search_){
                break;
            }

            vel_pub_.publish(vel);
            ros::spinOnce();
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
                ROS_INFO("Circle the area until object detected");

                // rangkaian move() (belum ada NAV)
                double lin_speed = 0.2;  // m/s
                double ang_speed = 0.30;  // rad/s
                double linx_distance = 2.5;  // meter
                move(lin_speed, 0, 0, linx_distance / lin_speed, true);  // forward 2.5 m
                move(0, 0, -ang_speed, (pi) / ang_speed, true);  // yaw 180
                move(0, 0, ang_speed, (pi/2) / ang_speed, true);  // yaw 90 cc

                move(lin_speed, 0, 0, linx_distance / lin_speed, true);  // repeat
                move(0, 0, -ang_speed, (pi) / ang_speed, true);
                move(0, 0, ang_speed, (pi/2) / ang_speed, true);

                move(lin_speed, 0, 0, linx_distance / lin_speed, true);  // repeat
                move(0, 0, -ang_speed, (pi) / ang_speed, true);
                move(0, 0, ang_speed, (pi/2) / ang_speed, true);

                move(lin_speed, 0, 0, linx_distance / lin_speed, true);  // repeat
                move(0, 0, -ang_speed, (pi) / ang_speed, true);
                move(0, 0, ang_speed, (pi/2) / ang_speed, true);
            }

            if(tracked_){
                break;
            }

            ros::spinOnce();
            rate.sleep();
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
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        odom_ = *msg;
    }

    void searchstatusCallback(const std_msgs::Bool::ConstPtr& msg){
        search_ = msg->data;
    }

    ros::Subscriber tracked_status_sub_, center_sub_, depth_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber search_status_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher simpleCMD_pub_;

    geometry_msgs::Point center_;
    nav_msgs::Odometry odom_;
    bool search_ = false;
    int depth_ = 0;
    bool tracked_ = false;

    int frame_width_;
    int center_threshold_;
    int desired_distance_;
    int distance_threshold_;

    ros::Time aligned_time_;
    bool aligned_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "move_approach_node");
    MyRobot my_robot;

    // my_robot.search();
    my_robot.approach();
    my_robot.approach_sit();
    
    return 0;
}












    // geometry_msgs::Twist rotate_xy_vector(float x, float y, float yaw){
    //     geometry_msgs::Twist vel;
    //     float cos_yaw = cos(yaw);
    //     float sin_yaw = sin(yaw);
    //     vel.linear.x = x * cos_yaw + y * sin_yaw;
    //     vel.linear.y = x * sin_yaw - y * cos_yaw;
    
    //     return vel;
    // }

    // // Batasi radian dari lower_bound sampe upper_bound
    // double NormalizeRad(double rad, double lower_bound = -pi, double upper_bound = pi){
    //     while(rad > upper_bound){
    //         rad -= S_TWO_PI;
    //     }
    //     while(rad < lower_bound){
    //         rad += S_TWO_PI;
    //     }
    //     return rad;

    // }

    // double CalcYawVel(double targetTheta, double currentTheta){
    //     targetTheta = NormalizeRad(targetTheta, -pi, pi);
    //     currentTheta = NormalizeRad(currentTheta, -pi, pi);

    //     double shortestDiff;
    //     double diff1 = targetTheta - currentTheta;
    //     double diff2 = diff1 - S_TWO_PI;
    //     double diff3 = diff1 + S_TWO_PI;

    //     if(std::abs(diff1) <= std::abs(diff2)){
    //         if(std::abs(diff1) <= std::abs(diff3)){
    //             shortestDiff = diff1;
    //         }else{
    //             shortestDiff = diff3;
    //         }
    //     }else{
    //         if(std::abs(diff2) <= std::abs(diff3)){
    //             shortestDiff = diff2;
    //         }else{
    //             shortestDiff = diff3;
    //         }
    //     }

    //     return shortestDiff;
    // }

    // //Calculate angular z velocity from current theta to target theta using input PID
    // double CalcYawVelPID(double targetTheta, double currentTheta){  // (, PID &PID_YAW)
    //     double shortestDiff = CalcYawVel(targetTheta, currentTheta);
    //     // double vel = PID_YAW.calculate(shortestDiff, 0);
    //     double vel = shortestDiff * 0.25;  // very simple pid
    //     return vel; 
    // }

    // bool nearEqual(double a, double b, double precision)    {
    //     return std::fabs(a - b) < precision;
    // }




    // // karena vel xy yaw langsung dikasi gede ?? hmm keknya ga
    // // karena udah dikirim cmd_vel tapi auto mode belom nyala, terus pas nyalain auto mode lalu ngirim lagi, jadi ada suatu buffer yg bikin jadi gajelas ??
    // void go_to(geometry_msgs::PoseStamped& target_pose){
    //     geometry_msgs::Twist vel;
    //     bool done = false;
    //     ros::Rate rate(10); // 10 Hz
    //     double ALLOWED_LINEAR_ERROR = 0.3;  // toleransi error
    //     double ALLOWED_ANGULAR_ERROR = 0.3;  // toleransi error

    //     while(ros::ok() && !done){
    //         // Reset vel
    //         vel.linear.x = 0;
    //         vel.linear.y = 0;
    //         vel.linear.z = 0;
    //         vel.angular.x = 0;
    //         vel.angular.y = 0;
    //         vel.angular.z = 0;
            
    //         // Convert quaternion to rpy
    //         double cur_roll, cur_pitch, cur_yaw;  // current
    //         tf::Quaternion cur_q(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
    //         tf::Matrix3x3(cur_q).getRPY(cur_roll, cur_pitch, cur_yaw);
            
    //         double target_roll, target_pitch, target_yaw;
    //         tf::Quaternion target_q(target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);
    //         tf::Matrix3x3(target_q).getRPY(target_roll, target_pitch, target_yaw);
            
    //         ROS_INFO("Go To x:%f y:%f yaw:%f\nCurrent x:%f y:%f yaw:%f\n", target_pose.pose.position.x, target_pose.pose.position.y, target_yaw, odom_.pose.pose.position.x, odom_.pose.pose.position.y, cur_yaw);
            
    //         // calculate error
    //         double error_x = target_pose.pose.position.x - odom_.pose.pose.orientation.x;
    //         double error_y = target_pose.pose.position.y - odom_.pose.pose.orientation.y;
    //         double error_yaw = target_yaw - cur_yaw;

    //         // rotate error (xy vector) because /cmd_vel is relative to robot but error_xy is relative to the world
    //         vel = rotate_xy_vector(error_x, error_y, cur_yaw);

    //         // vel.linear.x = pid_x.calculate(vel.linear.x, 0);
    //         // vel.linear.y = pid_y.calculate(vel.linear.y, 0);
    //         vel.linear.x = vel.linear.x * 0.25;  // very simple pid
    //         vel.linear.y = vel.linear.y * 0.25;
    //         vel.angular.z = CalcYawVelPID(target_yaw, cur_yaw);

    //         // apply error tolerance
    //         if (std::fabs(error_x) < ALLOWED_LINEAR_ERROR) vel.linear.x = 0;
    //         if (std::fabs(error_y) < ALLOWED_LINEAR_ERROR) vel.linear.y = 0;
    //         if (std::fabs(error_yaw) < ALLOWED_ANGULAR_ERROR) vel.angular.z = 0;

    //         vel_pub_.publish(vel);
    //         ROS_INFO("vel_x: %f, vel_y: %f, ang_z: %f", vel.linear.x, vel.linear.y, vel.angular.z);

    //         // if(nearEqual(odom_.pose.pose.position.x, target_pose.pose.position.x, ALLOWED_LINEAR_ERROR) && nearEqual(odom_.pose.pose.position.y, target_pose.pose.position.y, ALLOWED_LINEAR_ERROR)){
    //         //     ROS_INFO("Go To x:%f y:%f z:%f | Done", target_pose.pose.position.x, target_pose.pose.position.y);
    //         //     done = true;
    //         // }
            
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    // }