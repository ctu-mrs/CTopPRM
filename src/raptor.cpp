#include "raptor.hpp"

template <>
std::string Raptor<Vector<3>>::to_string_raw(Vector<3> data) {
  std::stringstream ss;
  ss << data(0) << "," << data(1) << "," << data(2);
  return ss.str();
}