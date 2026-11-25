#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/planning/height_only_grid_planner.hpp"
namespace c=rubi_heightmap_step_wavefront_planner;
namespace {
std::shared_ptr<const c::HeightmapSnapshot> height(double wall=0., bool hole=false, bool opening=false) {
  std::vector<c::HeightPoint> p;
  for(int y=0;y<15;++y) for(int x=0;x<25;++x) {
    if(hole && x==12) continue;
    p.push_back({-.95+.1*x,-.45+.1*y,x==12 && (!opening || y<10)?wall:0.});
  }
  return std::make_shared<const c::HeightmapSnapshot>(c::HeightmapSnapshot::fromPoints(p,.1,.001,10000));
}
c::StepEvaluatorParameters params() {
  c::StepEvaluatorParameters p;p.node_evidence_radius_m=.01;p.node_min_observed_cells=1;
  p.node_max_nearest_evidence_distance_m=.01;p.max_crossable_height_jump_m=.11;return p;
}
}
TEST(HeightOnlyGrid, FlatCostAndCoordinatesWithoutInflation) {
  auto p=params();p.inflation_cost_weight=999.;
  c::HeightOnlyGridPlanner planner(height(),p,{});ASSERT_TRUE(planner.warm());
  auto r=planner.plan({-.75,.25},{1.25,.25});ASSERT_TRUE(r.success)<<r.message;
  EXPECT_NEAR(r.path_metrics.total_cost,2.,1e-8);
  EXPECT_NEAR(r.nodes[r.path_node_ids.front()].point.x,-.75,1e-8);
}
TEST(HeightOnlyGrid, RejectsUnknownStripeAndExcessiveStep) {
  for(bool hole:{false,true}) {
    c::HeightOnlyGridPlanner planner(height(.3,hole),params(),{});ASSERT_TRUE(planner.warm());
    EXPECT_FALSE(planner.plan({-.75,.25},{1.25,.25}).success);
  }
}
TEST(HeightOnlyGrid, FindsHeightDetourAndRetainsElevations) {
  c::HeightOnlyGridPlanner planner(height(.3,false,true),params(),{});ASSERT_TRUE(planner.warm());
  auto r=planner.plan({-.75,.25},{1.25,.25});ASSERT_TRUE(r.success)<<r.message;
  EXPECT_GT(r.path_metrics.total_cost,2.);
  for(auto id:r.path_node_ids) EXPECT_NEAR(r.nodes[id].elevation_m,0.,1e-8);
}
TEST(HeightOnlyGrid, LowStepUsesHeightCostAndCancelledWarmFails) {
  c::HeightOnlyGridPlanner planner(height(.08),params(),{});ASSERT_TRUE(planner.warm());
  auto r=planner.plan({-.75,.25},{1.25,.25});ASSERT_TRUE(r.success)<<r.message;
  EXPECT_GT(r.path_metrics.total_cost,2.);
  std::atomic<bool> cancel{true};c::HeightOnlyGridPlanner cancelled(height(),params(),{});
  EXPECT_FALSE(cancelled.warm(&cancel));
}

namespace {
c::StepEvaluatorParameters preference() {
  c::StepEvaluatorParameters p;
  p.max_crossable_height_jump_m=.11;
  p.height_cost_weight=1.;
  p.node_height_outlier_threshold_m=.11;
  p.local_relief_hard_reject_enabled=true;
  p.local_relief_threshold_m=.11;
  p.grid_terrain_clearance_enabled=true;
  p.grid_terrain_clearance_distance_m=.2;
  p.grid_cot_height_cost_enabled=true;
  p.grid_cot_apply_to_descents=true;
  p.grid_cot_step_heights_m={0.,.01,.06,.08,.10,.11};
  p.grid_cot_equivalent_penalties_m={0.,0.,.02,.30,2.,3.};
  return p;
}
std::shared_ptr<const c::HeightmapSnapshot> platform(double step, bool barrier=false) {
  std::vector<c::HeightPoint> points;
  for(int y=0;y<81;++y) for(int x=0;x<101;++x) {
    double px=-2.+.05*x,py=-2.+.05*y;
    bool raised=barrier ? px>=0. : px>=0. && px<=1. && std::abs(py)<=.5;
    points.push_back({px,py,raised?step:0.});
  }
  return std::make_shared<const c::HeightmapSnapshot>(
      c::HeightmapSnapshot::fromPoints(points,.05,.001,20000));
}
}
TEST(HeightOnlyGrid, PreferenceCrossesSixAndDetoursTenAndTwentyNineBothDirections) {
  for(double step:{.06,.10,.29}) {
    c::HeightOnlyGridPlanner planner(platform(step),preference(),{});
    ASSERT_TRUE(planner.warm());
    for(bool reverse:{false,true}) {
      auto r=planner.plan(reverse?c::Point2D{2.,0.}:c::Point2D{-1.,0.},
                          reverse?c::Point2D{-1.,0.}:c::Point2D{2.,0.});
      ASSERT_TRUE(r.success)<<step<<" "<<r.message;
      double peak=0.;
      for(auto id:r.path_node_ids) peak=std::max(peak,r.nodes[id].elevation_m);
      if(step==.06) {
        EXPECT_NEAR(peak,.06,1e-9);
        EXPECT_NEAR(r.path_metrics.height_cost,.04,1e-8);
      } else EXPECT_NEAR(peak,0.,1e-9)<<step;
    }
  }
}
TEST(HeightOnlyGrid, TenRemainsTraversableWithoutDetourAndTwentyNineIsForbidden) {
  for(double step:{.10,.29}) {
    c::HeightOnlyGridPlanner planner(platform(step,true),preference(),{});
    ASSERT_TRUE(planner.warm());
    for(bool reverse:{false,true}) {
      auto r=planner.plan(reverse?c::Point2D{2.,0.}:c::Point2D{-1.,0.},
                          reverse?c::Point2D{-1.,0.}:c::Point2D{2.,0.});
      EXPECT_EQ(r.success,step==.10)<<r.message;
      if(r.success) { EXPECT_NEAR(r.path_metrics.height_cost,2.,1e-8); }
    }
  }
}
TEST(HeightOnlyGrid, DescentPreferenceIsOptInAndInvalidatesFrozenCache) {
  auto h=platform(.10,true);auto p=preference();p.grid_cot_apply_to_descents=false;
  c::HeightOnlyGridPlanner legacy(h,p,{});ASSERT_TRUE(legacy.warm());
  auto r=legacy.plan({2.,0.},{-1.,0.});ASSERT_TRUE(r.success)<<r.message;
  EXPECT_NEAR(r.path_metrics.height_cost,0.,1e-8);
  auto g=c::CostmapSnapshot::fromData(101,81,.05,-2.025,-2.025,std::vector<uint8_t>(8181,0));
  c::FrozenGridCache cache(h,g,p);
  p.grid_cot_apply_to_descents=true;
  EXPECT_FALSE(cache.matches(c::StepEvaluator(*h,g,p)));
}
