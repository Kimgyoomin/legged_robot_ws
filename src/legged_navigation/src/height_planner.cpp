// Apache-2.0. Adapter around the existing RUBI planner; no duplicate search algorithm.
#include <mutex>
#include <memory>
#include <cmath>
#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/ros/pointcloud2_heightmap_adapter.hpp"

namespace legged_navigation {
namespace core = rubi_heightmap_step_wavefront_planner;
class HeightPlanner : public nav2_core::GlobalPlanner {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::string name_, frame_;
  bool static_height_{true};
  double max_age_{2.5};
  double costmap_clearance_m_{0.0};
  core::StepEvaluatorParameters evaluation_;
  core::HeightmapAdapterParameters adapter_;
  core::GridAStarParameters grid_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::shared_ptr<const core::HeightmapSnapshot> height_;
  builtin_interfaces::msg::Time stamp_;
  std::mutex mutex_;
  template<class T> T parameter(const std::string &key, T fallback) {
    nav2_util::declare_parameter_if_not_declared(node_,name_+"."+key,rclcpp::ParameterValue(fallback));
    return node_->get_parameter(name_+"."+key).get_value<T>();
  }
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock(); name_=name; costmap_=costmap;
    frame_=costmap_->getGlobalFrameID();
    static_height_=parameter("static_heightmap",true);
    max_age_=parameter("max_heightmap_age_s",2.5);
    costmap_clearance_m_=parameter("costmap_clearance_m",0.0);
    if (!std::isfinite(costmap_clearance_m_) || costmap_clearance_m_<0.)
      throw std::invalid_argument("costmap_clearance_m must be finite and nonnegative");
    adapter_.resolution_m=parameter("map_resolution_m",.05);
    adapter_.lattice_tolerance_m=parameter("lattice_tolerance_m",.01);
    grid_.max_planning_time_ms=parameter<int64_t>("grid_max_planning_time_ms",500);
    grid_.max_expanded_states=parameter<int64_t>("grid_max_expanded_states",500000);
    grid_.allow_diagonal=parameter("grid_allow_diagonal",true);
    evaluation_.edge_check_spacing_m=parameter("evaluation.edge_check_spacing_m",0.025);
    evaluation_.max_crossable_height_jump_m=parameter("evaluation.max_crossable_height_jump_m",0.08);
    evaluation_.height_noise_floor_m=parameter("evaluation.height_noise_floor_m",0.01);
    evaluation_.height_cost_exponent=parameter("evaluation.height_cost_exponent",2.0);
    evaluation_.distance_weight=parameter("evaluation.distance_weight",1.0);
    evaluation_.height_cost_weight=parameter("evaluation.height_cost_weight",5.0);
    evaluation_.inflation_cost_weight=parameter("evaluation.inflation_cost_weight",5.0);
    evaluation_.inflation_cost_exponent=parameter("evaluation.inflation_cost_exponent",1.0);
    evaluation_.node_evidence_radius_m=parameter("node_evidence_radius_m",0.10);
    evaluation_.node_min_observed_cells=parameter<int64_t>("node_min_observed_cells",3);
    evaluation_.node_max_nearest_evidence_distance_m=parameter("node_max_nearest_evidence_distance_m",0.075);
    evaluation_.node_height_outlier_threshold_m=parameter("node_height_outlier_threshold_m",0.08);
    evaluation_.node_max_height_outlier_ratio=parameter("node_max_height_outlier_ratio",0.30);
    evaluation_.edge_height_query_radius_m=parameter("edge_height_query_radius_m",0.075);
    evaluation_.edge_max_height_evidence_gap_m=parameter("edge_max_height_evidence_gap_m",0.10);
    evaluation_.sobel_hard_reject_enabled=parameter("evaluation.sobel_hard_reject_enabled",false);
    evaluation_.sobel_equivalent_step_height_m=parameter("evaluation.sobel_equivalent_step_height_m",0.10);
    evaluation_.sobel_cost_weight=parameter("evaluation.sobel_cost_weight",0.0);
    evaluation_.sobel_cost_exponent=parameter("evaluation.sobel_cost_exponent",2.0);
    evaluation_.grid_sobel_hard_reject_enabled=parameter("evaluation.grid_sobel_hard_reject_enabled",false);
    evaluation_.grid_sobel_kernel_size=parameter<int64_t>("evaluation.grid_sobel_kernel_size",3);
    evaluation_.grid_sobel_gradient_cost_weight=parameter("evaluation.grid_sobel_gradient_cost_weight",0.0);
    evaluation_.grid_terrain_clearance_enabled=parameter("evaluation.grid_terrain_clearance_enabled",false);
    evaluation_.grid_terrain_clearance_distance_m=parameter("evaluation.grid_terrain_clearance_distance_m",0.20);
    evaluation_.local_relief_hard_reject_enabled=parameter("evaluation.local_relief_hard_reject_enabled",false);
    evaluation_.local_relief_threshold_m=parameter("evaluation.local_relief_threshold_m",0.10);
    evaluation_.local_relief_first_window_radius_m=parameter("evaluation.local_relief_first_window_radius_m",0.10);
    evaluation_.local_relief_second_window_radius_m=parameter("evaluation.local_relief_second_window_radius_m",0.10);
    evaluation_.local_relief_lower_quantile=parameter("evaluation.local_relief_lower_quantile",0.10);
    evaluation_.local_relief_upper_quantile=parameter("evaluation.local_relief_upper_quantile",0.90);
    evaluation_.local_relief_min_observed_ratio=parameter("evaluation.local_relief_min_observed_ratio",0.70);
    evaluation_.local_relief_critical_cell_count=parameter<int64_t>("evaluation.local_relief_critical_cell_count",3);
    evaluation_.grid_cot_height_cost_enabled=parameter("evaluation.grid_cot_height_cost_enabled",true);
    evaluation_.grid_cot_step_heights_m=parameter<std::vector<double>>("evaluation.grid_cot_step_heights_m",{});
    evaluation_.grid_cot_equivalent_penalties_m=parameter<std::vector<double>>("evaluation.grid_cot_equivalent_penalties_m",{});
    const auto topic=parameter<std::string>("input_heightmap_topic","/legged/height");
    subscription_=node_->create_subscription<sensor_msgs::msg::PointCloud2>(topic,rclcpp::QoS(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        try {
          if(cloud->header.frame_id!=frame_) throw std::runtime_error("height frame mismatch");
          auto snapshot=std::make_shared<const core::HeightmapSnapshot>(
            core::PointCloud2HeightmapAdapter{}.makeSnapshot(*cloud,adapter_));
          std::lock_guard<std::mutex> lock(mutex_);height_=snapshot;stamp_=cloud->header.stamp;
        } catch(const std::exception &e) {
          std::lock_guard<std::mutex> lock(mutex_);height_.reset();
          RCLCPP_ERROR(node_->get_logger(),"Height rejected: %s",e.what());
        }
      });
  }
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {subscription_.reset();height_.reset();costmap_.reset();node_.reset();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) override {
    std::shared_ptr<const core::HeightmapSnapshot> height;
    builtin_interfaces::msg::Time stamp;
    {std::lock_guard<std::mutex> lock(mutex_);height=height_;stamp=stamp_;}
    if(!height) throw std::runtime_error("No valid height map");
    if(start.header.frame_id!=frame_ || goal.header.frame_id!=frame_)
      throw std::runtime_error("Planning pose frame mismatch");
    const double age=(node_->now()-rclcpp::Time(stamp)).seconds();
    if(!static_height_ && (age<0. || age>max_age_)) throw std::runtime_error("Stale live height map");
    auto *map=costmap_->getCostmap();
    auto cost=[&] {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      return core::CostmapSnapshot::fromData(map->getSizeInCellsX(),map->getSizeInCellsY(),
        map->getResolution(),map->getOriginX(),map->getOriginY(),
        std::vector<uint8_t>(map->getCharMap(),map->getCharMap()+map->getSizeInCellsX()*map->getSizeInCellsY()));
    }();
    cost=cost.withObstacleClearance(costmap_clearance_m_);
    core::StepEvaluator evaluator(*height,cost,evaluation_);
    std::unique_ptr<core::LocalReliefHazardSnapshot> hazard;
    if(evaluation_.grid_terrain_clearance_enabled) hazard.reset(new core::LocalReliefHazardSnapshot(
      core::LocalReliefHazardSnapshot::build(evaluator)));
    const auto result=core::StepGridAStarPlanner(grid_).plan(evaluator,
      {start.pose.position.x,start.pose.position.y},{goal.pose.position.x,goal.pose.position.y},hazard.get());
    if(!result.success) throw std::runtime_error("Height planner: "+result.message);
    nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=node_->now();
    for(auto id:result.path_node_ids) {
      geometry_msgs::msg::PoseStamped p;p.header=path.header;
      p.pose.position.x=result.nodes.at(id).point.x;p.pose.position.y=result.nodes.at(id).point.y;
      p.pose.position.z=result.nodes.at(id).elevation_m;
      p.pose.orientation.w=1.;path.poses.push_back(p);
    }
    for(size_t i=0;i+1<path.poses.size();++i) {
      const auto &a=path.poses[i].pose.position;const auto &b=path.poses[i+1].pose.position;
      const auto yaw=std::atan2(b.y-a.y,b.x-a.x);
      path.poses[i].pose.orientation.z=std::sin(yaw/2);path.poses[i].pose.orientation.w=std::cos(yaw/2);
    }
    if(!path.poses.empty()) path.poses.back().pose.orientation=goal.pose.orientation;
    return path;
  }
};
}  // namespace legged_navigation
PLUGINLIB_EXPORT_CLASS(legged_navigation::HeightPlanner, nav2_core::GlobalPlanner)
