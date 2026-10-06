#pragma once
#include <vector>

namespace ocs2 {
namespace multi_robot {

struct SolverLogger {
  void reset() {
    binary_variables = 0;
    continuous_variables = 0;
    num_terrain_polygons = 0;
    solving_time = 0.0;
  }
  int binary_variables = 0;
  int continuous_variables = 0;
  int num_terrain_polygons = 0;
  double solving_time = 0.0;
};

} // namespace multi_robot
} // namespace ocs2