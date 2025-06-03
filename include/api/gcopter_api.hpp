#include "gcopter/flatness.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

class GCopterAPI {
public:
  void configure_map();
  void set_endpoints();
  void run_inference();
  void get_visualization_data();

private:
};