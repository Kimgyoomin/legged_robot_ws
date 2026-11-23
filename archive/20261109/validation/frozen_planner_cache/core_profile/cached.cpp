#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
namespace c=rubi_heightmap_step_wavefront_planner;
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
int main(int argc,char **argv){
 if(argc!=3){std::cerr<<"usage: cached HEIGHT_XYZ64 COSTMAP_U8\n";return 2;}
 std::ifstream hf(argv[1],std::ios::binary|std::ios::ate);
 if(!hf){std::cerr<<"height open failed\n";return 2;} auto bytes=hf.tellg(); hf.seekg(0); std::vector<c::HeightPoint> points(static_cast<size_t>(bytes)/sizeof(c::HeightPoint)); hf.read(reinterpret_cast<char*>(points.data()),bytes);
 auto h0=Clock::now(); auto height=c::HeightmapSnapshot::fromPoints(points,.05,.01,5000000);auto h1=Clock::now();
 std::ifstream cf(argv[2],std::ios::binary);
 if(!cf){std::cerr<<"costmap open failed\n";return 2;} std::vector<uint8_t> raw((std::istreambuf_iterator<char>(cf)),std::istreambuf_iterator<char>());
 c::StepEvaluatorParameters p;
#include "parameters.inc"
 c::GridAStarParameters gp;gp.max_planning_time_ms=5000;gp.max_expanded_states=500000;gp.allow_diagonal=true;
 std::cout<<std::fixed<<std::setprecision(3)<<"height_snapshot_ms="<<ms(h0,h1)<<" observed="<<height.observedCount()<<" cells="<<height.cellCount()<<"\n";
 auto cost=c::CostmapSnapshot::fromData(1300,2986,0.05000000074505806,-6.15,-54.1,raw).withObstacleClearance(.3);
 c::StepEvaluator e(height,cost,p);
 auto t0=Clock::now(); c::FrozenGridCache cache(std::make_shared<const c::HeightmapSnapshot>(height),cost,p);cache.warm();auto t1=Clock::now();
 std::cout<<"warm_ms="<<ms(t0,t1)<<" nodes="<<cache.cachedNodes()<<" cache_bytes="<<cache.estimatedBytes()<<std::endl;
 for(int i=0;i<5;++i){auto a=Clock::now();auto r=c::StepGridAStarPlanner(gp).plan(e,{2.85944,.169367},{48.4538,41.392},nullptr,&cache);auto b=Clock::now();
 std::cout<<"cached="<<i<<" ms="<<ms(a,b)<<" success="<<r.success<<" poses="<<r.path_node_ids.size()<<" expanded="<<r.expansions<<" cost="<<r.path_metrics.total_cost<<std::endl;}
}
