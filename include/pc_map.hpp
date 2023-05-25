#pragma once

#include "base_map.hpp"
#include "common.hpp"

class PCMap {
 public:
  PCMap();
  void load(std::string filename);
  // double getClearence(const Vector<3> pos);

  // std::pair<Vector<3>, Vector<3>> gradientInVoxelCenter(const Vector<3> pos);
  // Vector<3> getMinPos();
  // Vector<3> getMaxPos();

 private:
  template<typename T>
  void fill_data(T* data, cnpy::NpyArray& voxels_arr);
  std::vector<std::vector<float>> map_data;
  Vector<3> max_point_axis;
  Vector<3> min_point_axis;
};


template<typename T>
void PCMap::fill_data(T* data, cnpy::NpyArray& voxels_arr) {
  map_data.resize(voxels_arr.shape[0]);
  for (size_t xi = 0; xi < voxels_arr.shape[0]; xi++) {
    map_data[xi].resize(voxels_arr.shape[1]);
    for (size_t yi = 0; yi < voxels_arr.shape[1]; yi++) {
      map_data[xi][yi] = data[xi * voxels_arr.shape[0] + yi];
    }
  }
};