#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
namespace c=rubi_heightmap_step_wavefront_planner;
namespace {
std::shared_ptr<const c::HeightmapSnapshot> height(double step=0.,bool hole=false) {
 std::vector<c::HeightPoint> p;
 for(int y=0;y<15;++y)for(int x=0;x<25;++x){if(hole && x==12)continue;p.push_back({.05+.1*x,.05+.1*y,x==12?step:0.});}
 return std::make_shared<const c::HeightmapSnapshot>(c::HeightmapSnapshot::fromPoints(p,.1,.001,10000));
}
c::StepEvaluatorParameters params(){c::StepEvaluatorParameters p;p.node_evidence_radius_m=.01;p.node_min_observed_cells=1;p.node_max_nearest_evidence_distance_m=.01;p.max_crossable_height_jump_m=.11;return p;}
c::CostmapSnapshot cost(std::vector<uint8_t> data=std::vector<uint8_t>(375,0),double ox=0.){return c::CostmapSnapshot::fromData(25,15,.1,ox,0,std::move(data));}
void equal(const c::PlanResult&a,const c::PlanResult&b){ASSERT_EQ(a.success,b.success);ASSERT_EQ(a.path_node_ids.size(),b.path_node_ids.size());EXPECT_NEAR(a.path_metrics.total_cost,b.path_metrics.total_cost,1e-10);for(size_t i=0;i<a.path_node_ids.size();++i){auto x=a.nodes[a.path_node_ids[i]],y=b.nodes[b.path_node_ids[i]];EXPECT_DOUBLE_EQ(x.point.x,y.point.x);EXPECT_DOUBLE_EQ(x.point.y,y.point.y);EXPECT_DOUBLE_EQ(x.elevation_m,y.elevation_m);}}
void compare(const c::StepEvaluator&e,c::FrozenGridCache&f){auto h=c::LocalReliefHazardSnapshot::build(e);auto a=c::StepGridAStarPlanner({}).plan(e,{.25,.75},{2.25,.75},&h);auto b=c::StepGridAStarPlanner({}).plan(e,{.25,.75},{2.25,.75},nullptr,&f);equal(a,b);}
}
TEST(FrozenGridCache, DynamicObstaclesInflationAndReopeningUseCurrentCostmap){auto h=height();auto g=cost();auto p=params();c::FrozenGridCache f(h,g,p);ASSERT_TRUE(f.warm());
 compare(c::StepEvaluator(*h,g,p),f);
 for(uint8_t v:{uint8_t(180),uint8_t(252),uint8_t(253),uint8_t(254),uint8_t(255),uint8_t(0)}){
 std::vector<uint8_t>d(375,0);for(int x=5;x<20;++x)d[7*25+x]=v;auto changed=cost(d);c::StepEvaluator e(*h,changed,p);ASSERT_TRUE(f.matches(e));compare(e,f);}
 std::vector<uint8_t>d(375,0);for(int y=0;y<15;++y)d[y*25+12]=254;auto blocked=cost(d);c::StepEvaluator e(*h,blocked,p);compare(e,f);EXPECT_FALSE(c::StepGridAStarPlanner({}).plan(e,{.25,.75},{2.25,.75},nullptr,&f).success);
}
TEST(FrozenGridCache, StepsUnknownAndTerrainClearanceMatchUncached){for(double s:{.06,.10,.29})for(bool hole:{false,true}){auto h=height(s,hole);auto g=cost();auto p=params();p.local_relief_min_observed_ratio=.5;p.local_relief_critical_cell_count=1;p.local_relief_threshold_m=.11;p.local_relief_hard_reject_enabled=true;p.grid_terrain_clearance_enabled=true;p.grid_terrain_clearance_distance_m=.2;c::FrozenGridCache f(h,g,p);f.warm();compare(c::StepEvaluator(*h,g,p),f);}}
TEST(FrozenGridCache, DirectedCotAscentAndGeometryCostsMatch){for(bool cot:{false,true}){auto h=height(.1);auto g=cost();auto p=params();p.grid_cot_height_cost_enabled=cot;p.grid_cot_step_heights_m={0.,.05,.15};p.grid_cot_equivalent_penalties_m={0.,.3,1.6};c::FrozenGridCache f(h,g,p);f.warm();c::StepEvaluator e(*h,g,p);compare(e,f);auto a=c::StepGridAStarPlanner({}).plan(e,{1.25,.75},{2.25,.75});auto b=c::StepGridAStarPlanner({}).plan(e,{1.25,.75},{2.25,.75},nullptr,&f);equal(a,b);}}
TEST(FrozenGridCache, HeightGeometryAndParametersInvalidate){auto h=height();auto g=cost();auto p=params();c::FrozenGridCache f(h,g,p);auto h2=height(.1);auto g2=cost(std::vector<uint8_t>(375,0),.01);EXPECT_FALSE(f.matches(c::StepEvaluator(*h2,g,p)));EXPECT_FALSE(f.matches(c::StepEvaluator(*h,g2,p)));p.max_crossable_height_jump_m=.12;EXPECT_FALSE(f.matches(c::StepEvaluator(*h,g,p)));auto r=c::StepGridAStarPlanner({}).plan(c::StepEvaluator(*h,g,p),{.25,.75},{2.25,.75},nullptr,&f);EXPECT_FALSE(r.success);EXPECT_EQ(r.message,"frozen grid context mismatch");}
TEST(FrozenGridCache, WarmCanBeCancelled){auto h=height();auto g=cost();c::FrozenGridCache f(h,g,params());std::atomic<bool>cancel{true};EXPECT_FALSE(f.warm(&cancel));}
