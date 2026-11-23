#pragma once
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"
namespace legged_navigation {
class PlannerParameters {
 public:
  PlannerParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string name)
    : node_(std::move(node)), name_(std::move(name)) {}
  template<class T> T get(const std::string &key, T fallback) const {
    nav2_util::declare_parameter_if_not_declared(node_,name_+"."+key,rclcpp::ParameterValue(fallback));
    return node_->get_parameter(name_+"."+key).get_value<T>();
  }
 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
};
inline rubi_heightmap_step_wavefront_planner::StepEvaluatorParameters
readHeightEvaluation(const PlannerParameters &reader) {
  rubi_heightmap_step_wavefront_planner::StepEvaluatorParameters p;
    p.edge_check_spacing_m=reader.get("evaluation.edge_check_spacing_m",0.025);
    p.max_crossable_height_jump_m=reader.get("evaluation.max_crossable_height_jump_m",0.08);
    p.height_noise_floor_m=reader.get("evaluation.height_noise_floor_m",0.01);
    p.height_cost_exponent=reader.get("evaluation.height_cost_exponent",2.0);
    p.distance_weight=reader.get("evaluation.distance_weight",1.0);
    p.height_cost_weight=reader.get("evaluation.height_cost_weight",5.0);
    p.node_evidence_radius_m=reader.get("node_evidence_radius_m",0.10);
    p.node_min_observed_cells=reader.get<int64_t>("node_min_observed_cells",3);
    p.node_max_nearest_evidence_distance_m=reader.get("node_max_nearest_evidence_distance_m",0.075);
    p.node_height_outlier_threshold_m=reader.get("node_height_outlier_threshold_m",0.08);
    p.node_max_height_outlier_ratio=reader.get("node_max_height_outlier_ratio",0.30);
    p.edge_height_query_radius_m=reader.get("edge_height_query_radius_m",0.075);
    p.edge_max_height_evidence_gap_m=reader.get("edge_max_height_evidence_gap_m",0.10);
    p.sobel_hard_reject_enabled=reader.get("evaluation.sobel_hard_reject_enabled",false);
    p.sobel_equivalent_step_height_m=reader.get("evaluation.sobel_equivalent_step_height_m",0.10);
    p.sobel_cost_weight=reader.get("evaluation.sobel_cost_weight",0.0);
    p.sobel_cost_exponent=reader.get("evaluation.sobel_cost_exponent",2.0);
    p.grid_sobel_hard_reject_enabled=reader.get("evaluation.grid_sobel_hard_reject_enabled",false);
    p.grid_sobel_kernel_size=reader.get<int64_t>("evaluation.grid_sobel_kernel_size",3);
    p.grid_sobel_gradient_cost_weight=reader.get("evaluation.grid_sobel_gradient_cost_weight",0.0);
    p.grid_terrain_clearance_enabled=reader.get("evaluation.grid_terrain_clearance_enabled",false);
    p.grid_terrain_clearance_distance_m=reader.get("evaluation.grid_terrain_clearance_distance_m",0.20);
    p.local_relief_hard_reject_enabled=reader.get("evaluation.local_relief_hard_reject_enabled",false);
    p.local_relief_threshold_m=reader.get("evaluation.local_relief_threshold_m",0.10);
    p.local_relief_first_window_radius_m=reader.get("evaluation.local_relief_first_window_radius_m",0.10);
    p.local_relief_second_window_radius_m=reader.get("evaluation.local_relief_second_window_radius_m",0.10);
    p.local_relief_lower_quantile=reader.get("evaluation.local_relief_lower_quantile",0.10);
    p.local_relief_upper_quantile=reader.get("evaluation.local_relief_upper_quantile",0.90);
    p.local_relief_min_observed_ratio=reader.get("evaluation.local_relief_min_observed_ratio",0.70);
    p.local_relief_critical_cell_count=reader.get<int64_t>("evaluation.local_relief_critical_cell_count",3);
    p.grid_cot_height_cost_enabled=reader.get("evaluation.grid_cot_height_cost_enabled",true);
    p.grid_cot_step_heights_m=reader.get<std::vector<double>>("evaluation.grid_cot_step_heights_m",{});
    p.grid_cot_equivalent_penalties_m=reader.get<std::vector<double>>("evaluation.grid_cot_equivalent_penalties_m",{});
  return p;
}
}  // namespace legged_navigation
