#include <ros/ros.h>
#include <std_msgs/Bool.h>

class NodeA {
public:
    NodeA() {
        pub_ = nh_.advertise<std_msgs::Bool>("/toggle_node", 1);
        sub_ = nh_.subscribe("/toggle_node", 1, &NodeA::callback, this);
        active = false;
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {

            ROS_INFO("quadruped move until sit next to an object");
            ros::Duration(10.0).sleep();
            active = false;
            msg.data = true; pub_.publish(msg);
            ROS_INFO("handoff to arm\n");

            ROS_INFO("wait for the arm finish the grasping job");
            while(ros::ok() && (active==false)){
                ROS_INFO("...");
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("arm has been finished the grasping job\n");

            ROS_INFO("quadruped move to destination then sit");
            ros::Duration(10.0).sleep();
            active = false;
            msg.data = true; pub_.publish(msg);
            ROS_INFO("handoff to arm\n");

            ROS_INFO("wait for the arm finish the placement job");
            while(ros::ok() && (active==false)){
                ROS_INFO("...");
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("arm has been finished the placement job\n");
        }
    }

private:
    void callback(const std_msgs::Bool::ConstPtr& msg) {
        toggle_data = msg->data;
        active = !toggle_data;
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    bool toggle_data;
    std_msgs::Bool msg; 
    bool active;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_comm_quadruped_node");
    NodeA node;
    node.spin();
    return 0;
}
