#include "base_map.hpp"

bool BaseMap::isInCollision(Vector<3> object_position, const double clearance) {
  const double clearance_val = getClearence(object_position);
  // // INFO("clearance " << clearance)
  return !std::isfinite(clearance_val) || clearance_val < clearance;
  // return MeshObject::collide(&obstacles, object, object_position);
}

// bool BaseMap::isInCollision(Vector<3> object_position) {
//   return isInCollision(object_position, min_clearance_);
// }

void BaseMap::fillRandomState(HeapNode<Vector<3>> *positionToFill,
                              Vector<3> min_position,
                              Vector<3> position_range) {
  // positionToFill->city_node = false;
  positionToFill->data = (0.5 * (Vector<3>::Ones() + Vector<3>::Random()))
                             .cwiseProduct(position_range) +
                         min_position;
}

void BaseMap::fillRandomStateInEllipse(
    HeapNode<Vector<3>> *new_node, HeapNode<Vector<3>> *start,
    HeapNode<Vector<3>> *goal,
    const double ellipse_ratio_major_axis_focal_length, bool planar) {

  const double dist = (goal->data - start->data).norm();
  Vector<3> rand_ellipse_point = random_ellipsoid_point(
      start->data, goal->data, dist * ellipse_ratio_major_axis_focal_length);
  // exit(0);
  new_node->data = rand_ellipse_point;

  if (planar)
    new_node->data(2) = start->data(2);

}

/*
check if the path between two vectors is collision free
*/
std::pair<bool, Vector<3>>
BaseMap::isSimplePathFreeBetweenNodes(Vector<3> actual, Vector<3> neigbour,
                                      const double clearance,
                                      const double collision_distance_check) {

  const Vector<3> vec_between = neigbour - actual;
  const double vec_between_norm = vec_between.norm();
  
  if (vec_between_norm < collision_distance_check) {
    return {true, Vector<3>::Constant(NAN)};
  }

  const double num_points_path =
      ceil((vec_between_norm / collision_distance_check) + 1.0);

  for (int index = 1; index < num_points_path; index += 1) {
    const double length_between_01 = ((double)index / (double)num_points_path);

    const Vector<3> object_position = actual + vec_between * length_between_01;

    if (isInCollision(object_position, clearance)) {
      return {false, object_position};
    }
  }
  return {true, Vector<3>::Constant(NAN)};
}

bool
BaseMap::isSimplePathFreeBetweenGuards(Vector<3> actual, Vector<3> neigbour,
                                      const double clearance,
                                      const double collision_distance_check) {

  const Vector<3> vec_between = actual - neigbour;
  const double vec_between_norm = vec_between.norm();
  const double num_points_path =
      ceil((vec_between_norm / collision_distance_check) + 1.0);

  for (int index = 1; index < num_points_path; index += 1) {
    const double length_between_01 = ((double)index / (double)num_points_path);

    const Vector<3> object_position = neigbour + vec_between * length_between_01;

    if (isInCollision(object_position, clearance)) {
      return false;
    }
  }
  return true;
}

// std::pair<bool, Vector<3>>
// BaseMap::isSimplePathFreeBetweenNodes(Vector<3> actual, Vector<3> neigbour) {
//   return isSimplePathFreeBetweenNodes(actual, neigbour, min_clearance_);
// }

std::vector<Vector<3>>
BaseMap::samplePath(std::vector<HeapNode<Vector<3>> *> path,
                    const double length_tot, const double num_samples) {
  // // INFO("sample path begin")
  // // INFO("path num nodes " << path.size())
  // // INFO("pnum_samples " << num_samples)

  std::vector<Vector<3>> samples;
  samples.resize(num_samples);

  int path_current_id = 0;
  double current_path_length =
      (path[path_current_id + 1]->data - path[path_current_id]->data).norm();
  // distance(path[path_current_id]->data,
  //                  path[path_current_id + 1]->data);
  double path_current_path_start = 0;

  for (double i = 0; i < num_samples; ++i) {
    double at_length = length_tot * (i / (num_samples - 1));
    if (at_length > path_current_path_start + current_path_length) {
      path_current_id += 1;
      if (path_current_id + 1 >= path.size()) {
        if (at_length - length_tot > PRECISION_BASE_MAP) {
          INFO(at_length - length_tot);
          ERROR("baaaaaad path_current_id");
          // INFO_VAR(at_length)
          // INFO_VAR(length_tot)
          exit(1);
        } else {
          // precision metter
          path_current_id -= 1;
          at_length = path_current_path_start + current_path_length;
        }
      } else {
        path_current_path_start += current_path_length;
        current_path_length =
            (path[path_current_id + 1]->data - path[path_current_id]->data)
                .norm();

        // distance(path[path_current_id]->data,
        //                                path[path_current_id + 1]->data);
      }
    }
    const Vector<3> pos =
        path[path_current_id]->data +
        (path[path_current_id + 1]->data - path[path_current_id]->data) *
            ((at_length - path_current_path_start) / current_path_length);
    samples[i] = pos;
  }

  // // INFO("sample path end")
  return samples;
}

bool BaseMap::isDeformablePath(std::vector<HeapNode<Vector<3>> *> path1,
                               const double length_tot1,
                               std::vector<HeapNode<Vector<3>> *> path2,
                               const double length_tot2,
                               const double num_collision_check,
                               const double clearance,
                               const double collision_distance_check) {
  // // INFO("isDeformablePath clearance " << clearance)
  std::vector<Vector<3>> samples1 =
      samplePath(path1, length_tot1, num_collision_check);
  std::vector<Vector<3>> samples2 =
      samplePath(path2, length_tot2, num_collision_check);

  for (double i = 0; i < num_collision_check; ++i) {
    if (!isSimplePathFreeBetweenNodes(samples1[i], samples2[i], clearance,
                                      collision_distance_check)
             .first) {
      // // INFO("not deformable between " << samples1[i].transpose() << " "
      //                             << samples2[i].transpose())

      // const Vector<3> pos =
      //     isSimplePathFreeBetweenNodes(samples1[i], samples2[i], clearance,
      //                                  collision_distance_check)
      //         .second;

      // // INFO_VAR(map_->getClearence(pos))
      // std::cout << "samples1 " << samples1 << std::endl;
      // std::cout << "samples2 " << samples2 << std::endl;
      return false;
    }
  }
  return true;
}

// bool BaseMap::isSimplePathFreeBetweenNodes(HeapNode<Vector<3>> *actual,
//                                            HeapNode<Vector<3>> *neigbour) {
//   return isSimplePathFreeBetweenNodes(actual->data, neigbour->data).first;
// }

std::pair<bool, Vector<3>>
BaseMap::isPathCollisionFree(path_with_length<Vector<3>> path,
                             const double clearance,
                             const double collision_distance_check) {
  std::pair<bool, Vector<3>> cp;
  for (size_t i = 1; i < path.plan.size(); i++) {
    cp =
        isSimplePathFreeBetweenNodes(path.plan[i - 1]->data, path.plan[i]->data,
                                     clearance, collision_distance_check);
    if (!cp.first) {
      return cp;
    }
  }
  return cp;
}

bool BaseMap::isDeformablePathBetween(HeapNode<Vector<3>> *start,
                                      HeapNode<Vector<3>> *end,
                                      HeapNode<Vector<3>> *between1,
                                      HeapNode<Vector<3>> *between2,
                                      const double clearance,
                                      const double collision_distance_check) {
  const double length_b1_1 = (start->data - between1->data).norm();
  const double length_b1_2 = (between1->data - end->data).norm();
  const double length_b1_tot = length_b1_1 + length_b1_2;
  const double length_b2_1 = (start->data - between2->data).norm();
  const double length_b2_2 = (between2->data - end->data).norm();
  const double length_b2_tot = length_b2_1 + length_b2_2;
  //
  const double larger_length = std::max(length_b1_tot, length_b2_tot);
  const double num_check_collision =
      ceil(larger_length / collision_distance_check) + 1;

  bool deformable = isDeformablePath(
      {start, between1, end}, length_b1_tot, {start, between2, end},
      length_b2_tot, num_check_collision, clearance, collision_distance_check);
  // std::vector<Vector<3>> second = interpolatePath(, num_check_collision);

  return deformable;
}