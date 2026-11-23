// Apache-2.0. Humble cannot deserialize an empty YAML string-array parameter.
// This no-input layer supplies Nav2's required global costmap lifecycle context.
// It does not subscribe, mark obstacles, inflate costs, or feed HeightOnlyPlanner.
#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace legged_navigation {
class HeightContextLayer : public nav2_costmap_2d::Layer {
 public:
  void onInitialize() override { current_=true; enabled_=true; }
  void reset() override { current_=true; }
  bool isClearable() override { return false; }
  void updateBounds(double, double, double, double*, double*, double*, double*) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D&, int, int, int, int) override {}
};
}
PLUGINLIB_EXPORT_CLASS(legged_navigation::HeightContextLayer, nav2_costmap_2d::Layer)
