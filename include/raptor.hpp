#pragma once

#include <algorithm>
#include <flann/flann.hpp>
#include <limits>
#include <memory>
#include <vector>
#include <time.h>
#include <filesystem>

#include "common.hpp"
#include "esdf_map.hpp"
#include "base_map.hpp"
#include "tree_node.hpp"
#include "distinct_path_dfs.hpp"
#include "prm.hpp"

#define PRECISION_PRM_CLUSTERING (10e-6)
#define PRECISION (1E-4)

template<class T>
class Raptor {
 public:
  Raptor(const YAML::Node& planner_config, std::shared_ptr<BaseMap> map,
                 T start, T end);

  void sample_point();
  void setBorders(Vector<3> min_position, Vector<3> max_position);
  void saveRoadmap(std::string filename);
  static void savePath(std::string filename, std::vector<HeapNode<T>*> path);
  static path_with_length<T> shorten_path(path_with_length<T> path,
                                          bool forward = true);
  std::vector<path_with_length<T>> findDistinctPaths();  
  static std::vector<std::vector<path_with_length<Vector<3>>>>
  find_geometrical_paths(const YAML::Node &planner_config,
                         std::shared_ptr<BaseMap> map,
                         std::vector<Vector<3>> &gates_with_start_end_poses,
                         std::string output_folder);
  static std::vector<path_with_length<T>>
  removeEquivalentPaths(std::vector<path_with_length<T>> paths);
  static std::vector<path_with_length<T>> 
  removeTooLongPaths(std::vector<path_with_length<T>> paths);

 private:

  static std::string to_string_raw(T data);  
  static double distance(Vector<3> from, Vector<3> to) { return (from - to).norm(); }

  std::vector<HeapNode<T>*> getVisibleGuards(HeapNode<T>* node);
  std::map<HeapNode<T>*, double> nodesBetweenNodes(HeapNode<T>* node1,
                                                   HeapNode<T>* node2);
  // static double distance(HeapNode<T>* from, HeapNode<T>* to);


  // std::vector<HeapNode<T>*> cities_nodes_;
  std::vector<HeapNode<T>*> guard_nodes_;
  std::vector<HeapNode<T>*> connection_nodes_;
  static std::shared_ptr<BaseMap> map_;
  static double collision_distance_check_;
  static double min_clearance_;
  static int num_samples;
  double ellipse_ratio_major_axis_focal_length_;
  static double cutoof_distance_ratio_to_shortest_;
  HeapNode<T>* start_;
  HeapNode<T>* end_;
  bool planar_;

  Vector<3> min_position_, max_position_, position_range_;
};

template <class T> std::shared_ptr<BaseMap> Raptor<T>::map_;
template <class T>
double Raptor<T>::collision_distance_check_;
template <class T> int Raptor<T>::num_samples;
template <class T> double Raptor<T>::min_clearance_;
template <class T> double Raptor<T>::cutoof_distance_ratio_to_shortest_;


template<class T>
Raptor<T>::Raptor(const YAML::Node& planner_config, 
                                  std::shared_ptr<BaseMap> map, T start,
                                  T end) {
  map_ = map;
  start_ = new HeapNode<T>(start);
  end_ = new HeapNode<T>(end);
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");
  planar_ = planner_config["planar"].as<bool>();
  cutoof_distance_ratio_to_shortest_ = 
    loadParam<double>(planner_config, "cutoof_distance_ratio_to_shortest");
  collision_distance_check_ =
    loadParam<double>(planner_config, "collision_distance_check");
  ellipse_ratio_major_axis_focal_length_ = loadParam<double>(
    planner_config, "ellipse_ratio_major_axis_focal_length");
  num_samples = loadParam<int>(
    planner_config, "num_samples_between_gate");

  if (collision_distance_check_ == 0) {
    ERROR(
      "you need to specify collision_distance_check for sampling-based motion "
      "planning");
    exit(1);
  }

  guard_nodes_.push_back(start_);
  guard_nodes_.push_back(end_);
}

template<class T>
void Raptor<T>::sample_point() {
  // random point
  // // INFO("sample point")
  // // INFO("whaaaaat")
  HeapNode<T>* new_node = new HeapNode<T>();
  // // INFO("new node created")
  // // INFO("new_node " << new_node)
  // // INFO("start_ " << start_)
  // // INFO("end_ " << end_)
  // fillRandomState(new_node);

  map_->fillRandomStateInEllipse(new_node, start_, end_, ellipse_ratio_major_axis_focal_length_, planar_);


  std::vector<HeapNode<T>*> visible_cities = getVisibleGuards(new_node);
  // // INFO("visible_cities.size() " << visible_cities.size())
  if (visible_cities.size() == 0) {
    // // INFO("new guard")
    guard_nodes_.push_back(new_node);
  } else if (visible_cities.size() == 2) {
    bool distinct = true;

    // now there is path between visible_cities[0] new_node and
    // visible_cities[1]
    std::map<HeapNode<T>*, double> between_nodes =
      nodesBetweenNodes(visible_cities[0], visible_cities[1]);

    for (const auto& nb : between_nodes) {
      // check if one path visible_cities[0] new_node visible_cities[1] is
      // deformable to other visible_cities[0] nb.first visible_cities[1]
      //   bool is_deformable = isDeformablePathBetween(
      //   visible_cities[0], visible_cities[1], new_node, nb.first);

      bool is_deformable = map_->isDeformablePathBetween(
           visible_cities[0], visible_cities[1], new_node, nb.first,
           min_clearance_, collision_distance_check_);
      if (is_deformable) {
        distinct = false;
        const double dist1 = distance(visible_cities[0]->data, new_node->data);
        const double dist2 = distance(new_node->data, visible_cities[1]->data);
        // const double distance_new = distance(visible_cities[0], new_node) +
        //                             distance(new_node, visible_cities[1]);
        if (dist1 + dist2 < nb.second) {
          nb.first->data = new_node->data;
          nb.first->visibility_node_ids[visible_cities[0]] = dist1;
          visible_cities[0]->visibility_node_ids[nb.first] = dist1;
          nb.first->visibility_node_ids[visible_cities[1]] = dist2;
          visible_cities[1]->visibility_node_ids[nb.first] = dist2;
          // // INFO("improved pos " << new_node->data)
        }
        break;
      }
    }

    if (distinct) {
      const double dist1 = distance(visible_cities[0]->data, new_node->data);
      new_node->visibility_node_ids[visible_cities[0]] = dist1;
      visible_cities[0]->visibility_node_ids[new_node] = dist1;
      const double dist2 = distance(new_node->data, visible_cities[1]->data);
      new_node->visibility_node_ids[visible_cities[1]] = dist2;
      visible_cities[1]->visibility_node_ids[new_node] = dist2;
      connection_nodes_.push_back(new_node);
      // // INFO("new pos " << new_node->data)
    }
  }
}

template<class T>
std::vector<HeapNode<T>*> Raptor<T>::getVisibleGuards(
  HeapNode<T>* node) {
  std::vector<HeapNode<T>*> visible_cities;
  // for (size_t i = 0; i < cities_nodes_.size(); i++) {
  //   if (isSimplePathFreeBetweenNodes(cities_nodes_[i], node)) {
  //     visible_cities.push_back(cities_nodes_[i]);
  //   }
  // }
  for (size_t i = 0; i < guard_nodes_.size(); i++) {
    if (map_->isSimplePathFreeBetweenGuards(guard_nodes_[i]->data, node->data, 
                            min_clearance_, collision_distance_check_)) {
      visible_cities.push_back(guard_nodes_[i]);
    }
  }
  return visible_cities;
}

template<class T>
std::map<HeapNode<T>*, double> Raptor<T>::nodesBetweenNodes(
  HeapNode<T>* node1, HeapNode<T>* node2) {

  std::map<HeapNode<T>*, double> between_nodes;
  typename std::unordered_map<HeapNode<T>*, double>::iterator it;
  // std::vector<HeapNode<T>*> between_nodes;
  for (const auto& vn1 : node1->visibility_node_ids) {
    it = vn1.first->visibility_node_ids.find(node2);
    if (it != vn1.first->visibility_node_ids.end()) {
      between_nodes.insert(
        std::pair<HeapNode<T>*, double>(vn1.first, vn1.second + it->second));
    }
  }
  return between_nodes;
}

template<class T>
void Raptor<T>::setBorders(Vector<3> min_position,
                                   Vector<3> max_position) {
  min_position_ = min_position;
  max_position_ = max_position;
  position_range_ = max_position - min_position;
}

template<class T>
void Raptor<T>::saveRoadmap(std::string filename) {
  // INFO("save Raptor map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t i = 0; i < guard_nodes_.size(); i++) {
      std::string city_node_str = to_string_raw(guard_nodes_[i]->data);
      myfile << city_node_str << std::endl;
      for (const auto& vn1 : guard_nodes_[i]->visibility_node_ids) {
        std::string neighbor_str = to_string_raw(vn1.first->data);
        ss_connections << city_node_str << "," << neighbor_str << std::endl;
      }
    }
    // for (size_t i = 0; i < connection_nodes_.size(); i++) {
    //   std::string city_node_str = to_string_raw(connection_nodes_[i]->data);
    //   myfile << city_node_str << std::endl;
    //   for (const auto& vn1 : connection_nodes_[i]->visibility_node_ids) {
    //     std::string neighbor_str = to_string_raw(vn1.first->data);
    //     ss_connections << city_node_str << "," << neighbor_str << std::endl;
    //   }
    // }
    myfile << ss_connections.str();
    myfile.close();
  }
}

template<class T>
void Raptor<T>::savePath(std::string filename,
                                 std::vector<HeapNode<T>*> path) {
  // INFO("save Raptor map to file " << filename);
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

template<class T>
std::vector<path_with_length<T>> Raptor<T>::findDistinctPaths() {
  // INFO("findDistinctPaths begin")
  std::vector<HeapNode<T>*> all_nodes = connection_nodes_;
  all_nodes.insert(all_nodes.end(), guard_nodes_.begin() + 2,
                   guard_nodes_.end());  // do not add start and end
  DistinctPathDFS<T> dpDFS(all_nodes, start_, end_);
  std::vector<path_with_length<T>> distinct_paths = dpDFS.findPaths();
  // INFO("findDistinctPaths end")
  return distinct_paths;
}

template <class T>
path_with_length<T>
Raptor<T>::shorten_path(path_with_length<T> path,
                                          bool forward) {
  path_with_length<T> shortened;
  shortened.length = 0;

  // return path;

  const double num_samples = ceil(path.length / collision_distance_check_) + 1;
  // INFO("path.path size " << path.path.size())
  // INFO("num_samples " << num_samples);
  // INFO("collision_distance_check_ " << collision_distance_check_);
  std::vector<T> sampled = map_->samplePath(path.plan, path.length, num_samples);
  // INFO("num samples " << sampled.size());

  // int dir = 1;
  int start_iter = 1;
  if (forward) {
    shortened.plan.push_back(path.plan.front());
  } else {
    shortened.plan.push_back(path.plan.back());
    std::reverse(sampled.begin(), sampled.end());
    // dir = -1;
    // start_iter = sampled.size() - 2;
  }


  // for (size_t i = sampled.size()-2; i < sampled.size(); i += dir) {
  for (size_t i = 1; i < sampled.size(); i++) {
    HeapNode<T>* start_node = shortened.plan.back();
    // INFO("from " << start_node->data.transpose() << " to "
    //              << sampled[i].transpose())


    std::pair<bool, Vector<3>> is_free =
      map_->isSimplePathFreeBetweenNodes(start_node->data, sampled[i], min_clearance_, collision_distance_check_);

    if (!is_free.first) {
      // INFO("not free")
      const Vector<3> collision_place = is_free.second;
      const auto [gradient_in_place, voxel_center] =
        map_->gradientInVoxelCenter(collision_place);
      // INFO("collision in pos " << collision_place.transpose())
      // INFO("gradient_in_place " << gradient_in_place.transpose())
      // go perpendicular to line shortened.path.back()->data, sampled[i]
      // and towards shortest distance to line shortened.path.back()->data,
      // sampled[i-1]
      // const Vector<3> old_point_vector = sampled[i - 1] - start_node->data;


      const Vector<3> new_point_vector = sampled[i] - start_node->data;
      // INFO_VAR(new_point_vector.transpose())
      const Vector<3> normal_gradient_new_point =
        gradient_in_place.cross(new_point_vector);
      // INFO_VAR(normal_gradient_new_point.transpose())

      // const Vector<3> normal_new_old_start =
      //   old_point_vector.cross(new_point_vector);
      Vector<3> vec_from_collision =
        new_point_vector.cross(normal_gradient_new_point);
      vec_from_collision.normalize();
      // INFO_VAR(vec_from_collision.transpose())

      // const double clearance_collision =
      // map_->getClearence(collision_place); INFO("clearance_collision " <<
      // clearance_collision)
      const double ds = collision_distance_check_;

      // double count_check =
      // std::min(collision_distance_check_ / clearance_collision, 1.0);
      // INFO("count_check " << count_check)
      bool added_after_collision = false;
      for (double tci = min_clearance_; tci <= min_clearance_ * 4; tci += ds) {
        const Vector<3> new_point = voxel_center + vec_from_collision * tci;
        // INFO("test collision in new place " << new_point.transpose())
        if (!map_->isInCollision(new_point, min_clearance_)) {
          HeapNode<T>* new_node = new HeapNode<T>(new_point);
          // HeapNode<T>* back_old = shortened.path.back();
          const double dist_new_node = distance(start_node->data, new_point);
          start_node->visibility_node_ids[new_node] = dist_new_node;
          new_node->visibility_node_ids[start_node] = dist_new_node;
          shortened.length += dist_new_node;
          shortened.plan.push_back(new_node);
          added_after_collision = true;
          break;
        } else {
          // INFO("in collision wiht value" << map_->getClearence(new_point))
        }
      }

      if (!added_after_collision) {
        ERROR_RED("no point added to shortened path after collision");
        return path;
        exit(1);
      }
    }
  }

  // INFO("shortened from " << path.length << " to " << shortened.length)
  // INFO("shortened.path.size() " << shortened.path.size())
  // INFO("shorten_path end")

  HeapNode<T>* last_node_original;
  if (forward) {
    last_node_original = path.plan.back();
  } else {
    last_node_original = path.plan.front();
  }

  HeapNode<T>* back_old = shortened.plan.back();
  const double dist_new_node =
    distance(back_old->data, last_node_original->data);
  back_old->visibility_node_ids[last_node_original] = dist_new_node;
  last_node_original->visibility_node_ids[back_old] = dist_new_node;
  shortened.length += dist_new_node;
  shortened.plan.push_back(last_node_original);

  if (!forward) {
    reverse(shortened.plan.begin(), shortened.plan.end());
  }


  // debuging begin
  // check distance
  double calc_distance = 0;
  for (size_t i = 1; i < shortened.plan.size(); i++) {
    calc_distance +=
      distance(shortened.plan[i - 1]->data, shortened.plan[i]->data);
  }
  if (fabs(calc_distance - shortened.length) > PRECISION || shortened.plan[shortened.plan.size() - 1]->data != path.plan[path.plan.size() - 1]->data) {
    INFO_RED("shortened length does not equal")
    INFO_VAR(shortened.length)
    INFO_VAR(calc_distance)
    exit(1);
  }
  // INFO("calc_distance " << calc_distance)
  // debuging end


  return shortened;
}

template<class T>
std::vector<path_with_length<T>> Raptor<T>::removeEquivalentPaths(
  std::vector<path_with_length<T>> paths) {
  // // INFO_GREEN("removeEquivalentPaths begin with " << paths.size() << " paths")
  // // INFO_VAR(min_clearance_)
  // // INFO_VAR(collision_distance_check_)
  // // INFO_VAR(map_->getResolution())
  std::vector<path_with_length<T>> paths_copy = paths;
  // std::vector<path_with_length<T>> proned;
  if (paths_copy.size() > 1) {
    // bool something_removed = true;
    // while (something_removed) {
    // something_removed = false;

    // for (size_t i = 0; i < paths_copy.size(); i++) {
    size_t i = 0;
    while(i < paths_copy.size()) {
      int shortest_path_i = i;
      double shortest_length = paths_copy[i].length;
      std::vector<int> to_remove_indexes;
      for (size_t j = i + 1; j < paths_copy.size(); j++) {
        const double larger_length =
          std::max(paths_copy[i].length, paths_copy[j].length);
        const double num_check_collision =
          ceil(larger_length / collision_distance_check_) + 1;
        bool deformable = map_->isDeformablePath(
          paths_copy[i].plan, paths_copy[i].length, paths_copy[j].plan,
          paths_copy[j].length, num_check_collision, min_clearance_, collision_distance_check_);
        if (deformable) {
          // // INFO("path " << i << " is deformable to " << j)
          // ith_unique = false;
          to_remove_indexes.push_back(j);
          if (paths_copy[j].length < shortest_length) {
            shortest_path_i = j;
            shortest_length = paths_copy[j].length;
          }
        } else {
          // // INFO("path " << i << " not deformable to " << j)
        }
      }
      if (shortest_path_i != i) {
        paths_copy[i] = paths_copy[shortest_path_i];
      }
      for (int tri = to_remove_indexes.size() - 1; tri >= 0; tri--) {
        // INFO("removing " << to_remove_indexes[tri])
        paths_copy.erase(paths_copy.begin() + to_remove_indexes[tri]);
        // // INFO("size " << to_remove_indexes[tri])
      }
      // // INFO("purged")
      //}
      i++;
    }
  }
  // // INFO_GREEN("removeEquivalentPaths end")
  return paths_copy;
}

template<class T>
std::vector<path_with_length<T>> Raptor<T>::removeTooLongPaths(
  std::vector<path_with_length<T>> paths) {
  std::vector<path_with_length<T>> proned = paths;
  // // INFO("removeTooLongPaths begin")

  // find shortest length
  double min_length = DBL_MAX;
  for (auto p : proned) {
    if (p.length < min_length) {
      min_length = p.length;
    }
  }

  // remove the one that are longer than cutoof_distance_ratio_to_shortest_ *
  // min_length
  for (int i = proned.size() - 1; i >= 0; i--) {
    if (proned[i].length > cutoof_distance_ratio_to_shortest_ * min_length) {
      proned.erase(proned.begin() + i);
    }
  }
  // // INFO("remove " << (paths.size() - proned.size())
  //                << " paths due to being too long")
  // // INFO("removeTooLongPaths end")
  return proned;
}


template <class T>
std::vector<std::vector<path_with_length<Vector<3>>>>
Raptor<T>::find_geometrical_paths(
    const YAML::Node &planner_config, std::shared_ptr<BaseMap> map,
    std::vector<Vector<3>> &gates_with_start_end_poses,
    std::string output_folder) {

  // INFO("find_geometrical_paths begin")

  auto begin_c = std::chrono::high_resolution_clock::now();
  // std::shared_ptr<PRM<Vector<3>>> prm;
  std::vector<std::vector<path_with_length<Vector<3>>>> paths_between_gates;
  std::vector<std::shared_ptr<Raptor<Vector<3>>>> topological_prms;

  paths_between_gates.resize(gates_with_start_end_poses.size() - 1);
  topological_prms.resize(gates_with_start_end_poses.size() - 1);
  for (size_t i = 0; i < topological_prms.size(); i++) {
    // // INFO("gates_with_start_end_poses[i] " << gates_with_start_end_poses[i])
    // // INFO("gates_with_start_end_poses[i+1] "
    //      << gates_with_start_end_poses[i + 1])
    topological_prms[i] = std::make_shared<Raptor<Vector<3>>>(
      planner_config, map, gates_with_start_end_poses[i],
      gates_with_start_end_poses[i + 1]);
    topological_prms[i]->setBorders(map->getMinPos(), map->getMaxPos());
  }
  // INFO("TopologicalPRM created");


  for (size_t i = 0; i < topological_prms.size(); i++) {
    for (size_t si = 0; si < num_samples; si++) {
      topological_prms[i]->sample_point();
    }

    // std::stringstream ss;
    // ss << "roadmap_all_rap" << i << ".csv";    
    // topological_prms[i]->saveRoadmap(ss.str());

    std::vector<path_with_length<Vector<3>>> diff_paths =
      topological_prms[i]->findDistinctPaths();
    
    auto end_c = std::chrono::high_resolution_clock::now();
    auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);
    INFO_GREEN("algorithm time raptor " << elapsed_c.count() * 1e-9)

    std::vector<path_with_length<Vector<3>>> shortened_paths = diff_paths;

    for (size_t pi = 0; pi < diff_paths.size(); pi++) {
      // // INFO("shortening path bef " << pi)
      // // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())
      shortened_paths[pi] =
        Raptor<Vector<3>>::shorten_path(shortened_paths[pi]);
      shortened_paths[pi] =
        Raptor<Vector<3>>::shorten_path(shortened_paths[pi], false);
      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // // INFO("shortening path aft " << pi)
    }

    shortened_paths =
     Raptor<Vector<3>>::removeTooLongPaths(shortened_paths);

    shortened_paths =
     Raptor<Vector<3>>::removeEquivalentPaths(shortened_paths);

      for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
      // // INFO("shortening path bef " << pi)
      // // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())
      shortened_paths[pi] =
        Raptor<Vector<3>>::shorten_path(shortened_paths[pi]);
      shortened_paths[pi] =
        Raptor<Vector<3>>::shorten_path(shortened_paths[pi], false);
      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // // INFO("shortening path aft " << pi)
    }

    shortened_paths =
     Raptor<Vector<3>>::removeEquivalentPaths(shortened_paths);

    std::sort(shortened_paths.begin(), shortened_paths.end(),
              comparator_path_with_length<Vector<3>>);

    paths_between_gates[i] = shortened_paths;
    // auto end_c = std::chrono::high_resolution_clock::now();
    // auto elapsed_c = std::chrono::duration_cast<std::chrono::nanoseconds>(end_c - begin_c);
    // INFO_GREEN("post processing time raptor " << elapsed_c.count() * 1e-9)

  //   for (size_t pi = 0; pi < diff_paths.size(); pi++) {
  //   //   std::stringstream path_ss;
  //   //   // INFO(diff_paths[pi].length)
  //   //   path_ss << "roadmap_path" << i << "_" << pi << ".csv";
  //   //   Raptor<Vector<3>>::savePath(path_ss.str(), diff_paths[pi].plan);
  //   }
  }
  // INFO("samples created");

  return paths_between_gates;
}