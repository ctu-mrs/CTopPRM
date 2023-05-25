#include <log4cxx/basicconfigurator.h>
#include <log4cxx/logger.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <chrono>

#include "common.hpp"
#include "timer.hpp"
#include "prm.hpp"

#include "topological_prm.hpp"
#include "topological_prm_clustering.hpp"
#include "raptor.hpp"
#include "pdr.hpp"

using namespace log4cxx;

std::string planner_config_file = "./config_files/1-2-1.yaml";

std::string problemFile;

std::string canvasOutput = "";

YAML::Node config;
std::vector<double> start_loaded;
std::vector<double> end_loaded;
std::vector<Point2D> borders_loaded;
std::vector<Point2D> gates_loaded;
std::string world_file;
// std::vector<MeshObject *> objects;
std::vector<std::vector<double>> array;

std::string logfile;
std::string name;
std::string map_file;
std::string map_type;
std::string output_folder{"./prints/"};
Vector<3> start;
Vector<3> goal;

YAML::Node planner_config;
Scalar min_clearance_;
bool check_collisions_;
Scalar collision_distance_check_;

void parse_cmd_args(int argc, char **argv) {
  // overwrite the variables from the yaml with the one from command line

  try {
    TCLAP::CmdLine cmd("Topological planning program", ' ', "0.0");

    TCLAP::ValueArg<std::string> nameArg("", "name", "name", false,
                                         std::string("no_name"), "string", cmd);

    TCLAP::ValueArg<std::string> logfileArg("", "logfile", "logfile", false,
                                            logfile, "string", cmd);

    TCLAP::ValueArg<std::string> output_folderArg("", "output_folder",
                                                  "output_folder", false,
                                                  output_folder, "string", cmd);

    TCLAP::ValueArg<std::string> mapArg("", "map", "map", false, map_file,
                                        "string", cmd);

    TCLAP::ValueArg<LoadVector<double>> start_p_Arg("", "start_p", "start_p",
                                                    false, LoadVector<double>(),
                                                    "Vector<3>", cmd);
    TCLAP::ValueArg<LoadVector<double>> goal_p_Arg(
        "", "goal_p", "goal_p", false, LoadVector<double>(), "Vector<3>", cmd);

    cmd.parse(argc, argv);

    name = nameArg.getValue();
    // INFO("loaded name " << name)
    output_folder = output_folderArg.getValue();
    map_file = mapArg.getValue();
    logfile = logfileArg.getValue();

    // INFO("loaded logfile " << logfile)
    // INFO("map_file " << map_file)
    // INFO("creating output_folder " << output_folder)

    std::filesystem::create_directories(output_folder);

    if (start_p_Arg.isSet()) {
      LoadVector<double> start_cmd = start_p_Arg.getValue();
      Vector<3> new_start(start_cmd.vector[0], start_cmd.vector[1],
                          start_cmd.vector[2]);
      // INFO_CYAN("changing start from " << start.transpose() << " to "
      //                                 << new_start.transpose())
      start = new_start;
    }

    if (goal_p_Arg.isSet()) {
      LoadVector<double> goal_cmd = goal_p_Arg.getValue();
      Vector<3> new_goal(goal_cmd.vector[0], goal_cmd.vector[1],
                         goal_cmd.vector[2]);
      // INFO_CYAN("changing end from " << goal.transpose() << " to "
      //                               << new_goal.transpose())
      goal = new_goal;
      // exit(1);
    }

  } catch (TCLAP::ArgException &e) {
    std::cerr << "cmd args error: " << e.error() << " for arg " << e.argId()
              << std::endl;
    exit(1);
  }
}

std::string to_string_raw(Vector<3> data) {
  std::stringstream ss;
  ss << data(0) << "," << data(1) << "," << data(2);
  return ss.str();
}

void savePath(std::string filename, std::vector<HeapNode<Vector<3>> *> path) {
  // // INFO("save TopologicalPRM map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t ni = 1; ni < path.size(); ni++) {
      std::string from_str = to_string_raw(path[ni - 1]->data);
      std::string to_str = to_string_raw(path[ni]->data);
      myfile << from_str << "," << to_str << std::endl;
    }
    myfile.close();
  }
}

void savePaths(std::string output_folder, std::string method, path_with_length<Vector<3>> path, int pi) {
    std::stringstream path_ss;
    // // INFO("shortened length " << shortened_paths[pi].length)
    path_ss << output_folder << method << "_roadmap_shortened_unique_path" << 0 << "_"
            << pi << ".csv";
    // // INFO("juchuuuuu")
    savePath(path_ss.str(), path.plan);
    // // INFO("saved");
}

void save_double(std::string filename, double d) {
  std::ofstream myfile;
  myfile.open(filename.c_str(), std::ios_base::app);

  if (myfile.is_open()) {
    std::string d_str = std::to_string(d);
    myfile << d_str << std::endl;
    myfile.close();
  }
}

void removeMethodFiles(std::string output_folder) {

  std::stringstream ss_roadmap;
  ss_roadmap << output_folder;
  std::string ss_roadmap_str = ss_roadmap.str();
  std::cout << ss_roadmap_str << std::endl;

  for (const auto &entry :
       std::filesystem::directory_iterator(output_folder)) {
    // std::cout << entry.path().string() << std::endl;
    const std::string path_name = entry.path().string();
    if (path_name.compare(0, ss_roadmap_str.length(), ss_roadmap_str) == 0) {
      std::cout << "removing " << path_name << std::endl;
      std::filesystem::remove(entry.path());
    }
  }
}

void removeRoadmapFiles(std::string output_folder) {

  std::stringstream ss_roadmap;
  ss_roadmap << output_folder << "roadmap";
  std::string ss_roadmap_str = ss_roadmap.str();
  std::cout << ss_roadmap_str << std::endl;

  for (const auto &entry :
       std::filesystem::directory_iterator(output_folder)) {
    // std::cout << entry.path().string() << std::endl;
    const std::string path_name = entry.path().string();
    if (path_name.compare(0, ss_roadmap_str.length(), ss_roadmap_str) == 0) {
      std::cout << "removing " << path_name << std::endl;
      std::filesystem::remove(entry.path());
    }
  }
}

int test_planner(int argc, char **argv) {

  removeMethodFiles(output_folder);
  removeRoadmapFiles(output_folder);
  removeRoadmapFiles("./");

  planner_config = YAML::LoadFile(planner_config_file);

  map_type = loadParam<std::string>(planner_config, "map_type");
  map_file = loadParam<std::string>(planner_config, "map");
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");
  check_collisions_ = loadParam<bool>(planner_config, "check_collisions");
  collision_distance_check_ =
      loadParam<double>(planner_config, "collision_distance_check");

  if (planner_config["start"] && planner_config["end"]) {
    // define start pos
    if (planner_config["start"]["position"]) {
      std::vector<double> start_pos;
      parseArrayParam(planner_config["start"], "position", start_pos);
      start(0) = start_pos[0];
      start(1) = start_pos[1];
      start(2) = start_pos[2];
    } else {
      INFO_RED("you must specify start position");
      exit(1);
    }
    if (planner_config["end"]["position"]) {
      std::vector<double> end_pos;
      parseArrayParam(planner_config["end"], "position", end_pos);
      goal(0) = end_pos[0];
      goal(1) = end_pos[1];
      goal(2) = end_pos[2];
    } else {
      INFO_RED("you must specify end position");
      exit(1);
    }
  } else {
    INFO_RED("you must specify start and end position");
    exit(1);
  }
  // SST sst(planner_config, drone_config);
  parse_cmd_args(argc, argv);
  // singnal_handler_ = &sst;
  // register singal for killing
  // std::signal(SIGINT, signal_callback_sst);
  // sst.iterate();

  std::shared_ptr<BaseMap> map;
  // load map
  if (map_type == "ESDF") {
    map = std::make_shared<ESDFMap>();
  } else if (map_type == "PC") {
    ERROR("map type " << map_type << " not implemented")
  } else {
    ERROR("map type " << map_type << " not recognized")
    exit(1);
  }
  map->load(map_file);

  std::vector<Vector<3>> gates_with_start_end_poses;
  gates_with_start_end_poses.push_back(start);
  gates_with_start_end_poses.push_back(goal);
  // INFO("start:" << start.transpose() << " goal " << goal.transpose());

  std::stringstream clT_ss, clL_ss, clN_ss;
  clT_ss << output_folder << planner_config_file << "method_clustering_time.csv";
  clL_ss << output_folder << planner_config_file << "method_clustering_length.csv";
  clN_ss << output_folder << planner_config_file << "method_clustering_num.csv";

  auto begin_c = std::chrono::high_resolution_clock::now();
  std::vector<std::vector<path_with_length<Vector<3>>>> paths_between_gates_clustering =
      TopologicalPRMClustering<Vector<3>>::find_geometrical_paths(
          planner_config, map, gates_with_start_end_poses, output_folder);
  auto end_c = std::chrono::high_resolution_clock::now();
  auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);

  INFO_GREEN("computation time clustering " << elapsed_c.count() * 1e-9)
  INFO_GREEN("number of paths found " << paths_between_gates_clustering[0].size())
  save_double(clT_ss.str(), elapsed_c.count() * 1e-9);
  save_double(clN_ss.str(), paths_between_gates_clustering[0].size());
  for (int i=0; i<paths_between_gates_clustering[0].size(); i++) {
    // INFO("path " << i << " with length " << paths_between_gates_clustering[0][i].length)
    savePaths(output_folder, "roadmap_clustering", paths_between_gates_clustering[0][i], i);
    save_double(clL_ss.str(), paths_between_gates_clustering[0][i].length);
  }
  // INFO_GREEN("finished");
  return 0;
}

int main(int argc, char **argv) {
  startLogger("main");

  seed();

  test_planner(argc, argv);
  return 1;
}
