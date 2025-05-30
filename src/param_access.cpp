#include <ros/ros.h>
#include <vector>
#include <string>
#include <map>

std::map<std::string, std::vector<double>> loadWaypoints(ros::NodeHandle& nh, const std::string& room) {
    std::map<std::string, std::vector<double>> room_waypoints;

    XmlRpc::XmlRpcValue room_data;
    if (!nh.getParam(room, room_data)) {
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "param_access_node");
    ros::NodeHandle nh;

    ROS_INFO("TES");

    // Ambil semua waypoint dan simpan ke variabel
    auto room901_waypoints = loadWaypoints(nh, "Ruangan_901");

    displayWaypointsInRoom(room901_waypoints);

    ros::spin();
    return 0;
}
