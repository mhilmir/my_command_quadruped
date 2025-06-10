#include <ros/ros.h>
#include <std_msgs/Bool.h>

class NodeA {
public:
    NodeA() {
        pub_ = nh_.advertise<std_msgs::Bool>("/toggle_node", 1);
        sub_ = nh_.subscribe("/toggle_node", 1, &NodeA::callback, this);
        // active_ = true;
        // sent_ = false;
        // start_time_ = ros::Time::now();
        // ROS_INFO("Node A started.");
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            // if (active_) {
            //     ROS_INFO_THROTTLE(1, "Node A is running...");
            //     if (!sent_ && (ros::Time::now() - start_time_).toSec() > 2.0) {
            //         std_msgs::Bool msg;
            //         msg.data = true;
            //         pub_.publish(msg);
            //         ROS_INFO("Node A: Handoff to Node B.");
            //         active_ = false;
            //         sent_ = true;
            //     }
            // }
            // ros::spinOnce();
            // rate.sleep();

            ROS_INFO("quadruped move until sit next to an object");
            ros::Duration(3.0).sleep();
            msg.data = true; pub_.publish(msg);
            ROS_INFO("handoff to arm\n");

            ROS_INFO("wait for the arm finish the grasping job");
            while(ros::ok() && !(toggle_data==false)){
                ROS_INFO("...");
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("arm has been finished the grasping job\n");

            ROS_INFO("quadruped move to destination then sit");
            ros::Duration(3.0).sleep();
            msg.data = true; pub_.publish(msg);
            ROS_INFO("handoff to arm\n");

            ROS_INFO("wait for the arm finish the placement job");
            while(ros::ok() && !(toggle_data==false)){
                ROS_INFO("...");
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("arm has been finished the placement job\n");
        }
    }

private:
    void callback(const std_msgs::Bool::ConstPtr& msg) {
        // if (!msg->data) {
        //     ROS_INFO("Node A: Resuming operation.");
        //     active_ = true;
        //     sent_ = false;
        //     start_time_ = ros::Time::now();
        // }
        toggle_data = msg->data;
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    bool toggle_data;
    std_msgs::Bool msg; 
    // bool active_;
    // bool sent_;
    // ros::Time start_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_comm_quadruped_node");
    NodeA node;
    node.spin();
    return 0;
}
