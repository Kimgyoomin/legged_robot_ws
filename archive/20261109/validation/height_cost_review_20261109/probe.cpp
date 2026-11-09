#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
namespace p=rubi_heightmap_step_wavefront_planner;
void print_path(const p::PlanResult &a){
 std::cout<<"{\"success\":"<<a.success<<",\"message\":\""<<a.message<<"\",\"cost\":"<<a.path_metrics.total_cost<<",\"length\":"<<a.path_metrics.length_xy_m<<",\"height_cost\":"<<a.path_metrics.height_cost<<",\"inflation_cost\":"<<a.path_metrics.inflation_cost<<",\"path\":[";bool first=true;
 for(auto id:a.path_node_ids){auto &n=a.nodes.at(id);if(!first)std::cout<<",";first=false;std::cout<<"["<<n.point.x<<","<<n.point.y<<","<<n.elevation_m<<"]";}std::cout<<"]}";
}
int main(int argc,char**argv){
 std::ifstream f(argv[1]);std::string line,frame;std::getline(f,line);double r,ox,oy,cut;size_t nx,ny,count;long long stamp;f>>std::quoted(frame)>>r>>ox>>oy>>nx>>ny>>cut>>stamp>>count;
 std::vector<p::HeightPoint> points;
 for(size_t j=0;j<count;++j){size_t idx;double z,v;long long t;int usable;f>>idx>>z>>v>>t>>usable;if(usable)points.push_back({float(ox+(idx%nx+.5)*r),float(oy+(idx/nx+.5)*r),float(z)});}
 auto hm=p::HeightmapSnapshot::fromPoints(points,r,.01,5000000);
 std::vector<uint8_t> costs(1300*2986);std::ifstream cf(argv[2],std::ios::binary);cf.read(reinterpret_cast<char*>(costs.data()),costs.size());if(!cf)throw std::runtime_error("costmap dimensions");
 auto cm=p::CostmapSnapshot::fromData(1300,2986,r,-6.15,-54.1,costs).withObstacleClearance(.45);
 p::StepEvaluatorParameters q;
#include "params.inc"
 q.max_crossable_height_jump_m=std::stod(argv[3]); q.local_relief_threshold_m=q.max_crossable_height_jump_m; q.grid_cot_height_cost_enabled=(std::string(argv[4])=="cot");
 p::StepEvaluator e(hm,cm,q);auto h=p::LocalReliefHazardSnapshot::build(e);
 auto q0=q;q0.height_cost_weight=0;p::StepEvaluator e0(hm,cm,q0);auto h0=p::LocalReliefHazardSnapshot::build(e0);
 p::GridAStarParameters gp;gp.max_planning_time_ms=5000;gp.max_expanded_states=500000;
 std::vector<p::Point2D> centers={{19.855,43.15},{29.64,47.27},{32.475,43.885},{38.885,44.405},{43.44,43.30}};std::vector<std::string> ids={"S5","S3","S4","S1","S2"};
 std::cout<<std::setprecision(9)<<"[";
 for(size_t k=0;k<centers.size();++k){if(k)std::cout<<",";auto c=*cm.worldToCell(centers[k]);auto pt=cm.cellCenter(c);auto ne=e.evaluateNode(pt);auto tq=h.queryPoint(pt,.2);auto rel=e.supportedLocalReliefAt(hm.worldToCell(pt));
 std::cout<<"{\"id\":\""<<ids[k]<<"\",\"center\":["<<pt.x<<","<<pt.y<<"],\"center_node_valid\":"<<ne.valid<<",\"center_reason\":\""<<p::toString(ne.reason)<<"\",\"center_hazard_violation\":"<<tq.violation<<",\"center_relief\":"<<rel.value_or(-1)<<",\"grid\":[";
 bool first=true;
 for(int dy=-16;dy<=16;++dy)for(int dx=-16;dx<=16;++dx){p::GridCell b{c.x+dx,c.y+dy};auto xy=cm.cellCenter(b);auto n=e.evaluateNode(xy);auto t=h.queryPoint(xy,.2);auto rr=e.supportedLocalReliefAt(hm.worldToCell(xy));int reason=int(n.reason);if(n.valid){if(!t.evidence_valid)reason=15;else if(t.violation)reason=14;}
 if(!first)std::cout<<",";first=false;std::cout<<"["<<xy.x<<","<<xy.y<<","<<reason<<","<<rr.value_or(-1)<<"]";
 }
 std::cout<<"],\"straight_edges\":[";first=true;
 double length=0,cost=0,hcost=0,icost=0;int invalid=0;int span=(k==1?15:20);
 for(int dx=-span;dx<span;++dx){p::GridCell a{c.x+dx,c.y},b{c.x+dx+1,c.y};auto x=cm.cellCenter(a),y=cm.cellCenter(b);auto n=e.evaluateNode(x),m=e.evaluateNode(y);auto edge=e.evaluateGridTransition(a,b,n,m);auto t=h.querySegment(x,y,.2);
 bool ok=edge.valid&&t.evidence_valid&&!t.violation;invalid+=!ok;length+=edge.length_xy_m;cost+=edge.cost;hcost+=q.height_cost_weight*edge.height_jump_score_m;icost+=q.inflation_cost_weight*edge.inflation_score_m;
 if(!first)std::cout<<",";first=false;std::cout<<"["<<x.x<<","<<x.y<<","<<ok<<","<<int(edge.reason)<<","<<t.violation<<","<<edge.max_local_relief_m<<","<<edge.max_height_jump_m<<","<<edge.height_jump_score_m<<"]";
 }
 std::cout<<"],\"straight_invalid_edges\":"<<invalid<<",\"straight_cost_valid_only\":"<<cost<<",\"straight_height_score\":"<<hcost<<",\"straight_inflation_score\":"<<icost<<",\"default\":";
 auto start=cm.cellCenter({c.x-span,c.y}),goal=cm.cellCenter({c.x+span,c.y});print_path(p::StepGridAStarPlanner(gp).plan(e,start,goal,&h));std::cout<<",\"no_height_cost_diagnostic\":";print_path(p::StepGridAStarPlanner(gp).plan(e0,start,goal,&h0));std::cout<<"}";
 }
 std::cout<<"]\n";
}
