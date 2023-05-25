
#include "pc_map.hpp"

#include <float.h>


PCMap::PCMap() {}
void PCMap::load(std::string filename) {
  INFO("loading pc map from file " << filename);

  FILE* fp = fopen(filename.c_str(), "rb");

  if (!fp)
    throw std::runtime_error("npy_load: Unable to open file " + filename);


  cnpy::NpyArray pc_arr = BaseMap::getArray(fp);

  int num_points = pc_arr.shape[0];
  fclose(fp);
  int num_pc_values_in_rarray = 1;
  for (size_t i = 0; i < pc_arr.shape.size(); i++) {
    INFO("resolution_arr " << i << " shape " << pc_arr.shape[i])
    num_pc_values_in_rarray *= pc_arr.shape[i];
  }
  int num_bytes_per_voxel = (pc_arr.num_bytes() / num_pc_values_in_rarray);
  INFO("there should be " << num_pc_values_in_rarray << " voxels")
  INFO("num bytes per voxel is " << num_bytes_per_voxel)

  if (num_bytes_per_voxel == 4) {
    float* data = pc_arr.data<float>();
    fill_data<float>(data, pc_arr);
  } else if (num_bytes_per_voxel == 8) {
    double* data = pc_arr.data<double>();
    fill_data<double>(data, pc_arr);
  } else {
    INFO("bad number of bytes per voxel " << num_bytes_per_voxel)
    exit(1);
  }

  max_point_axis = Vector<3>::Constant(-DBL_MAX);
  min_point_axis = Vector<3>::Constant(DBL_MAX);

  for (size_t pi = 0; pi < map_data.size(); pi++) {
    for (size_t di = 0; di < map_data[pi].size(); di++) {
      if (map_data[pi][di] > max_point_axis(di)) {
        max_point_axis(di) = map_data[pi][di];
      }
      if (map_data[pi][di] < min_point_axis(di)) {
        min_point_axis(di) = map_data[pi][di];
      }
    }
  }


  INFO("min_point_axis " << min_point_axis.transpose())
  INFO("max_point_axis " << max_point_axis.transpose())


  INFO("filled pc map")
}

// Vector<3> PCMap::getMinPos() { return min_point_axis; }
// Vector<3> PCMap::getMaxPos() { return max_point_axis; }

// double PCMap::getClearence(const Vector<3> pos) { return 0; }

// std::pair<Vector<3>, Vector<3>> PCMap::gradientInVoxelCenter(
//   const Vector<3> pos) {
//   Vector<3> central_gradient = Vector<3>::Zero();
//   return {central_gradient.normalized(), pos};
// }
