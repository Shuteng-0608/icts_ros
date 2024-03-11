#include <ros/ros.h>
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <string>
#include <iostream>


void sub_callback(const nav_msgs::OccupancyGrid &map){
    ROS_INFO("Map Reading......");
    int grid_x = map.info.width;
    int grid_y = map.info.height;
    double resolution = map.info.resolution;
    std::string frame_id = map.header.frame_id;
    ROS_INFO("Map Width : %d", grid_x);
    ROS_INFO("Map Height : %d", grid_y);
    ROS_INFO("Map Resolution : %f", resolution);
    ROS_INFO("Map frame_id : %s", frame_id.c_str());
    // int obstacles[grid_x][grid_y];
    int obstacles = 0;
    for (int i = 0; i < grid_x * grid_y; i++) {
        if (map.data[i] > 0) {
            obstacles++;
        }
    }
    ROS_INFO("Number of obstacles : %d", obstacles);
  
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goal_client");
 
    ros::NodeHandle nh;
    // ROS_INFO("Subscribe to the")
    ros::Subscriber sub = nh.subscribe("/rb_0/map", 10, sub_callback);
    ros::spin();
 
    return 0;
}