#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
namespace c=rubi_heightmap_step_wavefront_planner;
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
int main(){
 std::ifstream hf("/tmp/rubi_planner_profile/height.xyz64",std::ios::binary|std::ios::ate);
 auto bytes=hf.tellg(); hf.seekg(0); std::vector<c::HeightPoint> points(static_cast<size_t>(bytes)/sizeof(c::HeightPoint)); hf.read(reinterpret_cast<char*>(points.data()),bytes);
 auto h0=Clock::now(); auto height=c::HeightmapSnapshot::fromPoints(points,.05,.01,5000000);auto h1=Clock::now();
 std::ifstream cf("/home/rclab/legged_robot_ws/validation/rubi_replay_20261109/full_route/costmap.u8",std::ios::binary);
 std::vector<uint8_t> raw((std::istreambuf_iterator<char>(cf)),std::istreambuf_iterator<char>());
 c::StepEvaluatorParameters p;
#include "parameters.inc"
 c::GridAStarParameters gp;gp.max_planning_time_ms=5000;gp.max_expanded_states=500000;gp.allow_diagonal=true;
 std::cout<<std::fixed<<std::setprecision(3)<<"height_snapshot_ms="<<ms(h0,h1)<<" observed="<<height.observedCount()<<" cells="<<height.cellCount()<<"\n";
 for(int i=0;i<3;++i){
  auto t0=Clock::now();auto cost=c::CostmapSnapshot::fromData(1300,2986,0.05000000074505806,-6.15,-54.1,raw);auto t1=Clock::now();
  cost=cost.withObstacleClearance(.3);auto t2=Clock::now();
  c::StepEvaluator e(height,cost,p);auto t3=Clock::now();
  auto hazard=c::LocalReliefHazardSnapshot::build(e);auto t4=Clock::now();
  auto r=c::StepGridAStarPlanner(gp).plan(e,{2.85944,.169367},{48.4538,41.392},&hazard);auto t5=Clock::now();
  std::cout<<"cold="<<i<<" cost_snapshot_ms="<<ms(t0,t1)<<" clearance_ms="<<ms(t1,t2)<<" evaluator_ms="<<ms(t2,t3)<<" terrain_ms="<<ms(t3,t4)<<" plan_ms="<<ms(t4,t5)<<" total_ms="<<ms(t0,t5)<<" success="<<r.success<<" poses="<<r.path_node_ids.size()<<" expanded="<<r.expansions<<" nodes="<<r.statistics.node_evaluation_calls<<" edges="<<r.statistics.edge_evaluation_calls<<" astar_ms="<<r.astar_time_ms<<" finalize_ms="<<r.path_finalize_time_ms<<" validate_ms="<<r.path_validation_time_ms<<" cost="<<r.path_metrics.total_cost<<std::endl;
  if(i==2){
   auto q0=Clock::now();c::StepEvaluator fresh(height,cost,p);
   auto q=c::StepGridAStarPlanner(gp).plan(fresh,{2.85944,.169367},{48.4538,41.392},&hazard);auto q1=Clock::now();
   std::cout<<"reused_hazard_only_ms="<<ms(q0,q1)<<" success="<<q.success<<" poses="<<q.path_node_ids.size()<<" cost="<<q.path_metrics.total_cost<<std::endl;
   for(int j=0;j<3;++j){auto a=Clock::now();auto z=c::StepGridAStarPlanner(gp).plan(e,{2.85944,.169367},{48.4538,41.392},&hazard);auto b=Clock::now();std::cout<<"unchanged_inputs_reuse="<<j<<" plan_ms="<<ms(a,b)<<" success="<<z.success<<" poses="<<z.path_node_ids.size()<<" cost="<<z.path_metrics.total_cost<<std::endl;}
  }
 }
}
