// Apache-2.0. Height-only Nav2 adapter; shares terrain evaluation and A* with HeightPlanner.
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "legged_navigation/planner_parameters.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/height_only_grid_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/ros/pointcloud2_heightmap_adapter.hpp"

namespace legged_navigation {
namespace core = rubi_heightmap_step_wavefront_planner;
class HeightOnlyPlanner : public nav2_core::GlobalPlanner {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_, frame_;
  core::HeightmapAdapterParameters adapter_;
  core::StepEvaluatorParameters evaluation_;
  core::GridAStarParameters search_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
  std::shared_ptr<const core::HeightmapSnapshot> height_, requested_;
  std::shared_ptr<core::HeightOnlyGridPlanner> prepared_;
  std::mutex mutex_, planning_mutex_;
  std::thread worker_;
  std::atomic<bool> cancel_{false};
  void stopWorker() { cancel_=true; if(worker_.joinable()) worker_.join(); }
  void prepare() {
    std::shared_ptr<const core::HeightmapSnapshot> height;
    { std::lock_guard<std::mutex> lock(mutex_); height=height_; }
    if (!height || height==requested_) return;
    stopWorker(); cancel_=false; requested_=height;
    RCLCPP_INFO(node_->get_logger(), "HeightOnly terrain preparing; wait for READY");
    worker_=std::thread([this,height] {
      try {
        auto started=std::chrono::steady_clock::now();
        auto next=std::make_shared<core::HeightOnlyGridPlanner>(height,evaluation_,search_);
        if (!next->warm(&cancel_) || cancel_) return;
        { std::lock_guard<std::mutex> lock(mutex_); if(height_!=height) return; prepared_=next; }
        RCLCPP_INFO(node_->get_logger(), "HeightOnly terrain READY: %.1f ms, %zu cached nodes; no 2D obstacle/inflation costs",
          std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count(),next->cachedNodes());
      } catch(const std::exception &e) { RCLCPP_ERROR(node_->get_logger(), "HeightOnly preparation failed: %s", e.what()); }
    });
  }
 public:
  ~HeightOnlyPlanner() override { timer_.reset(); stopWorker(); }
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock(); name_=name;
    if (!node_) throw std::runtime_error("HeightOnly parent node expired");
    // Only the frame is used. The Nav2 global costmap's geometry and costs are never read.
    frame_=costmap->getGlobalFrameID();
    PlannerParameters p(node_,name_);
    adapter_.resolution_m=p.get("map_resolution_m",.05);
    adapter_.lattice_tolerance_m=p.get("lattice_tolerance_m",.01);
    const auto budget=p.get<int64_t>("grid_max_planning_time_ms",800);
    const auto expanded=p.get<int64_t>("grid_max_expanded_states",500000);
    const auto cells=p.get<int64_t>("max_grid_cells",5000000);
    if(budget<1 || expanded<1 || cells<1) throw std::invalid_argument("HeightOnly search/map limits must be positive");
    adapter_.max_grid_cells=cells;
    search_.max_planning_time_ms=budget; search_.max_expanded_states=expanded;
    search_.allow_diagonal=p.get("grid_allow_diagonal",true);
    evaluation_=readHeightEvaluation(p);
    evaluation_.inflation_cost_weight=0.;
    const auto topic=p.get<std::string>("input_heightmap_topic","/legged/height");
    subscription_=node_->create_subscription<sensor_msgs::msg::PointCloud2>(topic,rclcpp::QoS(1).transient_local(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        try {
          if(cloud->header.frame_id!=frame_) throw std::runtime_error("height frame mismatch");
          auto next=std::make_shared<const core::HeightmapSnapshot>(core::PointCloud2HeightmapAdapter{}.makeSnapshot(*cloud,adapter_));
          std::lock_guard<std::mutex> lock(mutex_);
          if(!height_ || height_->contentHash()!=next->contentHash()) { height_=next; prepared_.reset(); }
        } catch(const std::exception &e) {
          std::lock_guard<std::mutex> lock(mutex_); height_.reset(); prepared_.reset();
          RCLCPP_ERROR(node_->get_logger(),"HeightOnly map rejected: %s",e.what());
        }
      });
    guard_=node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &ps) {
      rcl_interfaces::msg::SetParametersResult r; r.successful=true;
      for(const auto &p:ps) if(p.get_name().rfind(name_+".",0)==0) {
        r.successful=false; r.reason="HeightOnly parameters require lifecycle cleanup/configure"; break;
      }
      return r;
    });
    timer_=node_->create_wall_timer(std::chrono::milliseconds(500),[this]{prepare();});
  }
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {
    timer_.reset(); stopWorker(); guard_.reset(); subscription_.reset();
    prepared_.reset(); requested_.reset(); height_.reset(); node_.reset();
  }
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) override {
    std::lock_guard<std::mutex> planning_lock(planning_mutex_);
    if(start.header.frame_id!=frame_ || goal.header.frame_id!=frame_)
      throw std::runtime_error("HeightOnly planning pose frame mismatch");
    std::shared_ptr<core::HeightOnlyGridPlanner> planner;
    { std::lock_guard<std::mutex> lock(mutex_); planner=prepared_; }
    if(!planner) throw std::runtime_error("HeightOnly has no prepared map; wait for terrain READY");
    auto started=std::chrono::steady_clock::now();
    const auto result=planner->plan({start.pose.position.x,start.pose.position.y}, {goal.pose.position.x,goal.pose.position.y});
    RCLCPP_INFO(node_->get_logger(),"HeightOnly plan: search=%.1f ms expanded=%zu poses=%zu success=%s",
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count(),
      result.expansions,result.path_node_ids.size(),result.success?"true":"false");
    if(!result.success) throw std::runtime_error("HeightOnly: "+result.message);
    nav_msgs::msg::Path path; path.header.frame_id=frame_; path.header.stamp=node_->now();
    for(auto id:result.path_node_ids) {
      geometry_msgs::msg::PoseStamped p; p.header=path.header;
      const auto &n=result.nodes.at(id); p.pose.position.x=n.point.x; p.pose.position.y=n.point.y;
      p.pose.position.z=n.elevation_m; p.pose.orientation.w=1.; path.poses.push_back(p);
    }
    for(size_t i=0;i+1<path.poses.size();++i) {
      const auto &a=path.poses[i].pose.position; const auto &b=path.poses[i+1].pose.position;
      const auto yaw=std::atan2(b.y-a.y,b.x-a.x);
      path.poses[i].pose.orientation.z=std::sin(yaw/2); path.poses[i].pose.orientation.w=std::cos(yaw/2);
    }
    if(!path.poses.empty()) path.poses.back().pose.orientation=goal.pose.orientation;
    return path;
  }
};
}  // namespace legged_navigation
PLUGINLIB_EXPORT_CLASS(legged_navigation::HeightOnlyPlanner, nav2_core::GlobalPlanner)
