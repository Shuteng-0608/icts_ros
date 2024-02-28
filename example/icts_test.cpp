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


using namespace ICT;

int main(int argc, char* argv[]) {

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");

  std::string inputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }
  // read map, agents and obstacles
  YAML::Node config = YAML::LoadFile(inputFile);

  std::vector<std::pair<int, int> > obstacles;
  std::vector<std::pair<int, int> > goals;
  std::vector<std::pair<int, int> > starts;

  const auto& dim = config["map"]["dimensions"];

  // ===== for costmap ===== //

  // costmap_ = costmap_ros->getCostmap();
  // global_frame_ = costmap_ros->getGlobalFrameID();
  // int dimx = costmap_->getSizeInCellsX()
  // int dimy = costmap_->getSizeInCellsY()
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  // // get obstacles from costmap
  // const unsigned char *costarr = costmap_->getCharMap();
  // YAML::detail::iterator_value obstacles_;
  // int offset = 0, num_obs = 0;
  // for (int i = 0; i < dimy; ++i) {
  //   for (int j = 0; j < dimx; ++j) {
  //     if (costarr[offset] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
  //       obstacle_.emplace_back(std::make_pair(j, i));
  //       num_obs++;
  //     }
  //     offset++;
  //   }
  // }
  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.emplace_back(std::make_pair(node[0].as<int>(), node[1].as<int>()));
  }



  // starts and goals for each agent
  // ros publish the start and goal points
  // ros::init(argc, argv, "icts_send_goal_client");
 
  // ros::NodeHandle nh;

  // ros::Publisher pub_g0 = nh.advertise<geometry_msgs::PoseStamped>("/mapf_base/rb_0/goal", 1000);
  // ros::Publisher pub_g1 = nh.advertise<geometry_msgs::PoseStamped>("/mapf_base/rb_1/goal", 1000);
  
  // pub_g0.publish(global_r1);
  // YAML::detail::iterator_value start_;
  // YAML::detail::iterator_value goal_;
  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    starts.emplace_back(std::make_pair(start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(std::make_pair(goal[0].as<int>(), goal[1].as<int>()));
  }

  mapf_adapters::mapf mapf(dimx, dimy, obstacles, goals);
  
  std::pair<int, std::vector< std::vector< std::pair<int, int> > > > solution;
  std::pair<int, std::vector< std::vector< std::pair<int, int> > > > solution1;
  ICT::ICTS<mapf_adapters::mapf> mapf_icts;
  ICT_NEW::ICTS<mapf_adapters::mapf> mapf_icts_;

  auto icts_start = std::chrono::system_clock::now();
  bool success = mapf_icts_.search(mapf, starts, &solution);
  std::cout<<"Cost :: "<<solution.first<<std::endl;
  auto icts_end = std::chrono::system_clock::now();
  auto icts_time = std::chrono::duration<double>(icts_end - icts_start).count();
  //mapf_icts.search(mapf, starts, &solution1);//
  if (success) {
    std::cout << "Planning successful! " << std::endl;

    std::ofstream out("../example/output_icts_two_agents.yaml");
    out << "statistics:" << std::endl;
    out << "  cost: " << solution.first << std::endl;
    out << "  runtime: " << icts_time << std::endl;
    out << "schedule:" << std::endl;

    int count = 0;

    for(auto it=solution.second.begin(); it!=solution.second.end();++it){
      out << "  agent" << count << ":" << std::endl;
      std::vector<std::pair<int, int> > output;
      output = *it;
      // makespan = output.size()
      // multi threads
      for(int i=0; i<output.size(); i++){
        // geometry_msgs::PoseStamped global_r0;
        // global_r0.pose.position.x = output[i].first;
        // global_r0.pose.position.y = output[i].second;
        out << "    - x: " << output[i].first << std::endl
          << "      y: " << output[i].second << std::endl
          << "      t: " << i << std::endl;
        // pub_goal.publish(global_r0);
        // pub_goal.wait_for_result();
      }
      count++;
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  std::cout<<"TIME TAKEN TO COMPLETE THE TASK ::"<<std::endl
            <<"ICTS :: "<<icts_time<<std::endl<<std::endl<<std::endl;



  return 0;
}