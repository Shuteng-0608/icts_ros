#include <ros/ros.h>
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"


void sub_callback(const nav_msgs::OccupancyGrid &map){
    ROS_INFO("Map Reading......");
    int grid_x = map.info.width;
    int grid_y = map.info.height;
    ROS_INFO("Map Width : %d", grid_x);
    ROS_INFO("Map Height : %d", grid_y);
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
 
    ros::Subscriber sub = nh.subscribe("/map", 10, sub_callback);
    ros::spin();
 
    return 0;
}