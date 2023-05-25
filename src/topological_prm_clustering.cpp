#include "topological_prm_clustering.hpp"

template <>
std::string TopologicalPRMClustering<Vector<3>>::to_string_raw(Vector<3> data) {
  std::stringstream ss;
  ss << data(0) << "," << data(1) << "," << data(2);
  return ss.str();
}

template <>
double TopologicalPRMClustering<Vector<3>>::distance(Vector<3> from, Vector<3> to) {
  const Vector<3> vec_between = to - from;
  const double vec_between_norm = vec_between.norm();
  return vec_between_norm;
}
