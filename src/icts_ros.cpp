#include <fstream>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include <icts/icts.hpp>
#include <icts/ICTS_.hpp>
#include "mapf.hpp"

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"
#include <boost/thread.hpp>

#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace ICT;

void sendGoal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac, int rb_name) {
    
    ac.sendGoal(goal);
    ROS_INFO("[Agent %d] Goal (%.2f, %.2f) Received",
            rb_name,
            goal.target_pose.pose.position.x, 
            goal.target_pose.pose.position.y );
    ac.waitForResult();
    ROS_INFO("[Agent %d] Goal (%.2f, %.2f) Arrived",
            rb_name,
            goal.target_pose.pose.position.x, 
            goal.target_pose.pose.position.y );

}

void sub_callback(const nav_msgs::OccupancyGrid &map){
    // Load agents (start & goal) 
    YAML::Node config = YAML::LoadFile("/home/shuteng/catkin_ws/src/icts_ros/example/two_agents.yaml");
    // Map info init
    int dimx = 0;
    int dimy = 0;
    double resolution = 0;
    std::vector<std::pair<int, int> > goals;
    std::vector<std::pair<int, int> > starts;
    std::vector<std::pair<int, int> > obstacles;

    // === Read map === //
    ROS_INFO("Reading Map......");
    dimx = map.info.width;
    dimy = map.info.height;
    resolution = map.info.resolution;
    ROS_INFO("Map Width : %d", dimx);
    ROS_INFO("Map Height : %d", dimy);
    ROS_INFO("Map Resolution : %f", resolution);

    // === Read obstacles === //
    int num = 0;
    for (int i = 0; i < dimx * dimy; i++) {
        if (map.data[i] > 0.196 ) {
            obstacles.emplace_back(std::make_pair(i / dimx, i % dimx));
            num++;
        }
    }
    ROS_INFO("Number of obstacles : %d", num);

    for (const auto& node : config["agents"]) {
        const auto& start = node["start"];
        const auto& goal = node["goal"];
        const auto& name = node["name"];
        starts.emplace_back(std::make_pair(start[0].as<int>(), start[1].as<int>()));
        // std::cout << "s: " << startStates.back() << std::endl;
        goals.emplace_back(std::make_pair(goal[0].as<int>(), goal[1].as<int>()));

        ROS_INFO("Reading agent %d info : from start[%d, %d] to goal[%d, %d]",
                 name.as<int>(), start[0].as<int>(), start[1].as<int>(), goal[0].as<int>(), goal[1].as<int>());
    }

    ROS_INFO("====== ICTS Begin! ======");
    mapf_adapters::mapf mapf(dimx, dimy, obstacles, goals);
    
    std::pair<int, std::vector< std::vector< std::pair<int, int> > > > solution;
    std::pair<int, std::vector< std::vector< std::pair<int, int> > > > solution1;
    ICT::ICTS<mapf_adapters::mapf> mapf_icts;
    ICT_NEW::ICTS<mapf_adapters::mapf> mapf_icts_;

    auto icts_start = std::chrono::system_clock::now();
    bool success = mapf_icts_.search(mapf, starts, &solution);
    // int makespan = solution.first / 2;
    // ROS_INFO("===== Makespan : %d =====", makespan);
    // std::cout<<"Cost :: "<<solution.first<<std::endl;
    auto icts_end = std::chrono::system_clock::now();
    auto icts_time = std::chrono::duration<double>(icts_end - icts_start).count();
    //mapf_icts.search(mapf, starts, &solution1);//
    std::vector<std::pair<int, int> > output;
    std::vector<std::pair<int, int> > temp_;
    auto temp = solution.second.begin();
    temp_ = *temp;
    int path[2][temp_.size()][2];
    int makespan = 0;

    // For path visualization
    nav_msgs::Path global_path_msg0;
    global_path_msg0.header.stamp = ros::Time::now();
    global_path_msg0.header.frame_id = "map";
    nav_msgs::Path global_path_msg1;
    global_path_msg1.header.stamp = ros::Time::now();
    global_path_msg1.header.frame_id = "map";

    if (success) {
        global_path_msg0.poses.clear();
        global_path_msg1.poses.clear();
        // std::cout << "Planning successful! " << std::endl;
        ROS_INFO("===== Planning successful! =====");

        std::ofstream out("/home/shuteng/catkin_ws/src/icts_ros/example/output_icts_two_agents.yaml");
        out << "statistics:" << std::endl;
        out << "  cost: " << solution.first << std::endl;
        out << "  runtime: " << icts_time << std::endl;
        out << "schedule:" << std::endl;
        
        int count = 0;
        for (auto it = solution.second.begin(); it != solution.second.end(); ++it) {
            out << "  agent" << count << ":" << std::endl;
            // std::vector<std::pair<int, int> > output;
            output = *it;
            for (int i = 0; i < output.size(); i++) {
                path[count][i][0] = output[i].first;
                path[count][i][1] = output[i].second;
                out << "    - x: " << output[i].first << std::endl
                << "      y: " << output[i].second << std::endl
                << "      t: " << i << std::endl;

                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = output[i].first * resolution;
                pose.pose.position.y = output[i].second * resolution;
                pose.pose.orientation.w = 1.0;
                pose.header.stamp = ros::Time::now();  
                pose.header.frame_id = "map";  

                if (count == 0) {
                    global_path_msg0.poses.push_back(pose);
                } else if (count == 1) {
                    global_path_msg1.poses.push_back(pose);
                }
            }
            count++;
        }
        makespan = output.size();

        ROS_INFO("===== Makespan : %d =====", makespan);

    } else {
        ROS_INFO("===== Planning NOT successful! =====");
    }

    ROS_INFO("===== TIME TAKEN  : %f =====", icts_time);

    // Publish global path
    ros::NodeHandle nh_v;
    ros::Publisher global_path_pub0 = nh_v.advertise<nav_msgs::Path>("/mapf_base/rb_0/plan", 1, true);
    global_path_pub0.publish(global_path_msg0);
    ros::Publisher global_path_pub1 = nh_v.advertise<nav_msgs::Path>("/mapf_base/rb_1/plan", 1, true);
    global_path_pub1.publish(global_path_msg1);



    // publish path
    ROS_INFO("Publishing path to agents......");
    // Create action clients
    MoveBaseClient ac_0("/rb_0/move_base", true);
    while(!ac_0.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the rb_0/move_base/goal action server to come up");
    }
 
    MoveBaseClient ac_1("/rb_1/move_base", true);
    while(!ac_1.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the rb_1/move_base/goal action server to come up");
    }
    

    


    // Create goals
    move_base_msgs::MoveBaseGoal goal_0, goal_1;
    goal_0.target_pose.header.frame_id = "map";
    goal_1.target_pose.header.frame_id = "map";

    for (int i = 0; i < makespan; i++) {
        int rb_0 = 0;
        goal_0.target_pose.header.stamp = ros::Time::now();
        goal_0.target_pose.pose.position.x = path[0][i][0] * resolution;
        goal_0.target_pose.pose.position.y = path[0][i][1] * resolution;
        goal_0.target_pose.pose.orientation.w = 1;
        int rb_1 = 1;
        goal_1.target_pose.header.stamp = ros::Time::now();
        goal_1.target_pose.pose.position.x = path[1][i][0] * resolution;
        goal_1.target_pose.pose.position.y = path[1][i][1] * resolution;
        goal_1.target_pose.pose.orientation.w = 1;

        ROS_INFO("[=== At Time Stamp %d ===]", i);
        // Create threads to send goals for each robot
        boost::thread thread_0(boost::bind(&sendGoal, goal_0, boost::ref(ac_0), rb_0));
        boost::thread thread_1(boost::bind(&sendGoal, goal_1, boost::ref(ac_1), rb_1));
 
        // Join threads
        thread_0.join();
        thread_1.join();
    }
    ROS_INFO("==== MAPF Navigation Finished ====");
}



int main(int argc, char* argv[]) {

    ros::init(argc, argv, "send_goal_client");
    ros::NodeHandle nh;
 
    // ===== for costmap ===== //
    ros::Subscriber sub = nh.subscribe("/mapf_base/map", 10, sub_callback);
    ros::spin();
    return 0;
}