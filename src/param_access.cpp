#include <ros/ros.h>
#include <vector>
#include <string>

void loadWaypoints(ros::NodeHandle& nh, const std::string& room) {
    XmlRpc::XmlRpcValue room_data;
    if (!nh.getParam(room, room_data)) {
        ROS_WARN("No waypoints found for room: %s", room.c_str());
        return;
    }

    for (auto& wp : room_data) {
        std::string wp_name = wp.first;
        std::vector<double> pose;
        for (int i = 0; i < wp.second.size(); ++i) {
            pose.push_back(static_cast<double>(wp.second[i]));
        }
        ROS_INFO("%s/%s: x=%f, y=%f, theta=%f", room.c_str(), wp_name.c_str(), pose[0], pose[1], pose[2]);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "param_access_node");
    ros::NodeHandle nh;

    loadWaypoints(nh, "Ruangan_901");
    loadWaypoints(nh, "Ruangan_903");
    loadWaypoints(nh, "Lift_Barat");

    ros::spin();
    return 0;
}
