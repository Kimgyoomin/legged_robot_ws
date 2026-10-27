#include <fstream>
#include <iostream>
#include <vector>
#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"
namespace c = rubi_heightmap_step_wavefront_planner;
int main(int argc, char**argv) {
 std::ifstream h(argv[1]);std::vector<c::HeightPoint> points;c::HeightPoint p;
 while(h>>p.x>>p.y>>p.z) points.push_back(p);
 auto hm=c::HeightmapSnapshot::fromPoints(points,.05,.01,10000000);
 std::cout<<"file,clearance,valid,reason,cost,elevation\n";
 for(int n=2;n<argc;n++) {
  std::ifstream f(argv[n]);int w,hh;double res,ox,oy,x,y;f>>w>>hh>>res>>ox>>oy>>x>>y;
  std::vector<uint8_t> data;int v;while(f>>v)data.push_back(v);
  auto map=c::CostmapSnapshot::fromData(w,hh,res,ox,oy,std::move(data));
  for(double clearance:{0.,.45}) {
   auto padded=map.withObstacleClearance(clearance);
   c::StepEvaluatorParameters params;
   auto r=c::StepEvaluator(hm,padded,params).evaluateNode(padded.cellCenter(*padded.worldToCell({x,y})));
   std::cout<<argv[n]<<","<<clearance<<","<<r.valid<<","<<c::toString(r.reason)<<","<<int(r.raw_cost)<<","<<r.elevation_m<<"\n";
  }
 }
}
