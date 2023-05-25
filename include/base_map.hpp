#pragma once

#include "cnpy.h"
#include "common.hpp"
#include "dijkstra.hpp"
// #include "heap.hpp"

#define PRECISION_BASE_MAP (10e-6)

class BaseMap {
public:
  BaseMap(){};
  virtual void load(std::string filename){};
  virtual double getClearence(const Vector<3> pos) = 0;
  virtual Vector<3> getMinPos() { return Vector<3>::Zero(); };
  virtual Vector<3> getMaxPos() { return Vector<3>::Zero(); };
  //   Vector<3> getMinPos() { return centroid - extents / 2; }
  //   Vector<3> getMaxPos() { return centroid + extents / 2; }

  virtual std::pair<Vector<3>, Vector<3>>
  gradientInVoxelCenter(const Vector<3> pos) {
    return {Vector<3>::Zero(), Vector<3>::Zero()};
  };

  static cnpy::NpyArray getArray(FILE *fp) {
    std::vector<size_t> shape;
    size_t word_size;
    bool fortran_order;
    cnpy::parse_npy_header(fp, word_size, shape, fortran_order);

    cnpy::NpyArray arr(shape, word_size, fortran_order);
    size_t nread = fread(arr.data<char>(), 1, arr.num_bytes(), fp);
    if (nread != arr.num_bytes()) {
      // INFO("badly read array");
      exit(1);
    }
    // INFO("get_array with " << arr.num_bytes() << "bytes end with shape")
    for (size_t i = 0; i < arr.shape.size(); i++) {
      // INFO(i << " shape " << arr.shape[i])
    }
    return arr;
  }

  void fillRandomState(HeapNode<Vector<3>> *positionToFill,
                       Vector<3> min_position, Vector<3> position_range);

  void
  fillRandomStateInEllipse(HeapNode<Vector<3>> *positionToFill,
                           HeapNode<Vector<3>> *start,
                           HeapNode<Vector<3>> *goal,
                           const double ellipse_ratio_major_axis_focal_length,
                           bool planar);
  // bool isInCollision(Vector<3> object_position);
  bool isInCollision(Vector<3> object_position, const double clearance);
  // std::pair<bool, Vector<3>> isSimplePathFreeBetweenNodes(Vector<3> actual,
  //                                                         Vector<3>
  //                                                         neigbour);
  std::pair<bool, Vector<3>>
  isSimplePathFreeBetweenNodes(Vector<3> actual, Vector<3> neigbour,
                               const double clearance,
                               const double collision_distance_check);
  std::pair<bool, Vector<3>>
  isPathCollisionFree(path_with_length<Vector<3>> path, const double clearance,
                      const double collision_distance_check);

  bool
  isSimplePathFreeBetweenGuards(Vector<3> actual, Vector<3> neigbour,
                                      const double clearance,
                                      const double collision_distance_check);

  // bool isSimplePathFreeBetweenNodes(HeapNode<Vector<3>> *actual,
  //                                   HeapNode<Vector<3>> *neigbour);

  bool isDeformablePath(std::vector<HeapNode<Vector<3>> *> path1,
                        const double length_tot1,
                        std::vector<HeapNode<Vector<3>> *> path2,
                        const double length_tot2,
                        const double num_collision_check,
                        const double clearance,
                        const double collision_distance_check);

  bool isDeformablePathBetween(HeapNode<Vector<3>> *start,
                               HeapNode<Vector<3>> *end,
                               HeapNode<Vector<3>> *between1,
                               HeapNode<Vector<3>> *between2,
                               const double clearance,
                               const double collision_distance_check);

  std::vector<Vector<3>> samplePath(std::vector<HeapNode<Vector<3>> *> path,
                                    const double length_tot,
                                    const double num_samples);
};
