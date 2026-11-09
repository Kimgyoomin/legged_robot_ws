// Apache-2.0. Adapter around the existing RUBI planner; no duplicate search algorithm.
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
  bool cache_frozen_{true};
  std::shared_ptr<core::FrozenGridCache> prepared_;
  std::shared_ptr<const core::HeightmapSnapshot> requested_height_;
  std::array<double,5> requested_geometry_{};
  std::thread worker_;
  std::atomic<bool> cancel_{false};
  rclcpp::TimerBase::SharedPtr prepare_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  void stopWorker() {cancel_=true;if(worker_.joinable())worker_.join();}
  void prepare() {
    std::shared_ptr<const core::HeightmapSnapshot> height;
    {std::lock_guard<std::mutex> l(mutex_);height=height_;}
    if(!height)return;
    auto *m=costmap_->getCostmap();std::array<double,5> geometry;
    {std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> l(*m->getMutex());
     geometry={double(m->getSizeInCellsX()),double(m->getSizeInCellsY()),m->getResolution(),m->getOriginX(),m->getOriginY()};}
    if(geometry[0]==0 || geometry[1]==0)return;
    if(height==requested_height_ && geometry==requested_geometry_)return;
    stopWorker();cancel_=false;requested_height_=height;requested_geometry_=geometry;
    {std::lock_guard<std::mutex> l(mutex_);prepared_.reset();}
    RCLCPP_INFO(node_->get_logger(),"Frozen terrain preparing (height/geometry changed); wait for READY before requesting paths");
    worker_=std::thread([this,height,geometry] {
      const auto started=std::chrono::steady_clock::now();
      try {
        auto g=core::CostmapSnapshot::fromData(size_t(geometry[0]),size_t(geometry[1]),geometry[2],geometry[3],geometry[4],
          std::vector<uint8_t>(size_t(geometry[0])*size_t(geometry[1]),0));
        auto next=std::make_shared<core::FrozenGridCache>(height,g,evaluation_);
        if(!next->warm(&cancel_) || cancel_)return;
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
        {std::lock_guard<std::mutex> l(mutex_);if(height_!=height)return;prepared_=next;}
        RCLCPP_INFO(node_->get_logger(),"Frozen terrain READY: %.1f ms startup, %zu cached nodes, %.1f MiB indexed terrain (auxiliary evaluator storage excluded)",
          ms,next->cachedNodes(),next->estimatedBytes()/1048576.0);
      }catch(const std::exception &e){RCLCPP_ERROR(node_->get_logger(),"Frozen terrain preparation failed: %s",e.what());}
    });
  }
  template<class T> T parameter(const std::string &key, T fallback) {
    nav2_util::declare_parameter_if_not_declared(node_,name_+"."+key,rclcpp::ParameterValue(fallback));
    return node_->get_parameter(name_+"."+key).get_value<T>();
  }
 public:
  ~HeightPlanner() override {prepare_timer_.reset();stopWorker();}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock(); name_=name; costmap_=costmap;
    frame_=costmap_->getGlobalFrameID();
    static_height_=parameter("static_heightmap",true);
    cache_frozen_=parameter("cache_frozen_terrain",true) && static_height_;
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
    subscription_=node_->create_subscription<sensor_msgs::msg::PointCloud2>(topic,static_height_ ? rclcpp::QoS(1).transient_local() : rclcpp::QoS(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        try {
          if(cloud->header.frame_id!=frame_) throw std::runtime_error("height frame mismatch");
          auto snapshot=std::make_shared<const core::HeightmapSnapshot>(
            core::PointCloud2HeightmapAdapter{}.makeSnapshot(*cloud,adapter_));
          std::lock_guard<std::mutex> lock(mutex_);
          if(!height_ || height_->contentHash()!=snapshot->contentHash()) {height_=snapshot;prepared_.reset();}
          stamp_=cloud->header.stamp;
        } catch(const std::exception &e) {
          std::lock_guard<std::mutex> lock(mutex_);height_.reset();prepared_.reset();
          RCLCPP_ERROR(node_->get_logger(),"Height rejected: %s",e.what());
        }
      });
    // Parameters are a configured snapshot. Explicitly reject silent live edits;
    // lifecycle cleanup/configure reloads them and rebuilds the terrain cache.
    parameter_guard_=node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &ps) {
      rcl_interfaces::msg::SetParametersResult r;r.successful=true;
      for(const auto &p:ps)if(p.get_name().rfind(name_+".",0)==0) {
        r.successful=false;r.reason="HeightPlanner parameters require lifecycle cleanup/configure (terrain cache rebuild)";break;}
      return r;
    });
    if(cache_frozen_)prepare_timer_=node_->create_wall_timer(std::chrono::milliseconds(500),[this]{prepare();});
  }
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {prepare_timer_.reset();stopWorker();parameter_guard_.reset();subscription_.reset();prepared_.reset();requested_height_.reset();height_.reset();costmap_.reset();node_.reset();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) override {
    const auto plan_started=std::chrono::steady_clock::now();
    std::shared_ptr<core::FrozenGridCache> cache;
    std::shared_ptr<const core::HeightmapSnapshot> height;
    builtin_interfaces::msg::Time stamp;
    {std::lock_guard<std::mutex> lock(mutex_);height=height_;stamp=stamp_;cache=prepared_;}
    if(cache_frozen_ && !cache)throw std::runtime_error("Frozen terrain cache is preparing; wait for READY");
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
    const auto copied=std::chrono::steady_clock::now();
    cost=cost.withObstacleClearance(costmap_clearance_m_);
    const auto cleared=std::chrono::steady_clock::now();
    core::StepEvaluator evaluator(*height,cost,evaluation_);
    std::unique_ptr<core::LocalReliefHazardSnapshot> hazard;
    if(!cache && evaluation_.grid_terrain_clearance_enabled) hazard.reset(new core::LocalReliefHazardSnapshot(
      core::LocalReliefHazardSnapshot::build(evaluator)));
    const auto result=core::StepGridAStarPlanner(grid_).plan(evaluator,
      {start.pose.position.x,start.pose.position.y},{goal.pose.position.x,goal.pose.position.y},hazard.get(),cache.get());
    const auto searched=std::chrono::steady_clock::now();
    const auto elapsed=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
    RCLCPP_INFO(node_->get_logger(),"Height plan: cached=%s snapshot=%.1f ms clearance=%.1f ms search=%.1f ms total=%.1f ms expanded=%zu poses=%zu success=%s",
      cache?"true":"false",elapsed(plan_started,copied),elapsed(copied,cleared),elapsed(cleared,searched),elapsed(plan_started,searched),
      result.expansions,result.path_node_ids.size(),result.success?"true":"false");
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
