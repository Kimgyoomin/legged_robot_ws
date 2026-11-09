#include "rubi_heightmap_step_wavefront_planner/planning/frozen_grid_cache.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace rubi_heightmap_step_wavefront_planner {
namespace {
constexpr int dx[8]={1,-1,0,0,1,1,-1,-1};
constexpr int dy[8]={0,0,1,-1,1,-1,1,-1};
bool sameParameters(const StepEvaluatorParameters &a, const StepEvaluatorParameters &b) {
  return a.hard_clearance_radius_m == b.hard_clearance_radius_m &&
    a.edge_check_spacing_m == b.edge_check_spacing_m &&
    a.max_crossable_height_jump_m == b.max_crossable_height_jump_m &&
    a.height_noise_floor_m == b.height_noise_floor_m &&
    a.height_cost_exponent == b.height_cost_exponent &&
    a.distance_weight == b.distance_weight &&
    a.height_cost_weight == b.height_cost_weight &&
    a.grid_cot_height_cost_enabled == b.grid_cot_height_cost_enabled &&
    a.grid_cot_step_heights_m == b.grid_cot_step_heights_m &&
    a.grid_cot_equivalent_penalties_m == b.grid_cot_equivalent_penalties_m &&
    a.preferred_clearance_radius_m == b.preferred_clearance_radius_m &&
    a.clearance_cost_weight == b.clearance_cost_weight &&
    a.clearance_cost_exponent == b.clearance_cost_exponent &&
    a.inflation_cost_weight == b.inflation_cost_weight &&
    a.inflation_cost_exponent == b.inflation_cost_exponent &&
    a.node_evidence_radius_m == b.node_evidence_radius_m &&
    a.node_min_observed_cells == b.node_min_observed_cells &&
    a.node_max_nearest_evidence_distance_m == b.node_max_nearest_evidence_distance_m &&
    a.node_height_outlier_threshold_m == b.node_height_outlier_threshold_m &&
    a.node_max_height_outlier_ratio == b.node_max_height_outlier_ratio &&
    a.edge_height_query_radius_m == b.edge_height_query_radius_m &&
    a.edge_max_height_evidence_gap_m == b.edge_max_height_evidence_gap_m &&
    a.sobel_hard_reject_enabled == b.sobel_hard_reject_enabled &&
    a.sobel_equivalent_step_height_m == b.sobel_equivalent_step_height_m &&
    a.sobel_cost_weight == b.sobel_cost_weight &&
    a.sobel_cost_exponent == b.sobel_cost_exponent &&
    a.grid_sobel_hard_reject_enabled == b.grid_sobel_hard_reject_enabled &&
    a.grid_sobel_kernel_size == b.grid_sobel_kernel_size &&
    a.grid_sobel_gradient_cost_weight == b.grid_sobel_gradient_cost_weight &&
    a.grid_terrain_clearance_enabled == b.grid_terrain_clearance_enabled &&
    a.grid_terrain_clearance_distance_m == b.grid_terrain_clearance_distance_m &&
    a.local_relief_hard_reject_enabled == b.local_relief_hard_reject_enabled &&
    a.local_relief_threshold_m == b.local_relief_threshold_m &&
    a.local_relief_first_window_radius_m == b.local_relief_first_window_radius_m &&
    a.local_relief_second_window_radius_m == b.local_relief_second_window_radius_m &&
    a.local_relief_lower_quantile == b.local_relief_lower_quantile &&
    a.local_relief_upper_quantile == b.local_relief_upper_quantile &&
    a.local_relief_min_observed_ratio == b.local_relief_min_observed_ratio &&
    a.local_relief_critical_cell_count == b.local_relief_critical_cell_count;
}
}
FrozenGridCache::FrozenGridCache(std::shared_ptr<const HeightmapSnapshot> h,
    const CostmapSnapshot &g, StepEvaluatorParameters p)
: height_(std::move(h)), geometry_(CostmapSnapshot::fromData(g.sizeX(),g.sizeY(),
    g.resolution(),g.originX(),g.originY(),std::vector<uint8_t>(g.cellCount(),0))),
  evaluator_(*height_,geometry_,std::move(p)), slots_(g.cellCount(),-1) {
  if(evaluator_.parameters().grid_terrain_clearance_enabled)
    hazards_=std::make_unique<LocalReliefHazardSnapshot>(LocalReliefHazardSnapshot::build(evaluator_));
  for(std::size_t i=0;i<inflation_.size();++i)
    inflation_[i]=std::pow(double(i)/252.0,evaluator_.parameters().inflation_cost_exponent);
}
bool FrozenGridCache::matches(const StepEvaluator & e) const {
  const auto *g=e.costmap();
  return g && e.snapshot().contentHash()==height_->contentHash() &&
    g->sizeX()==geometry_.sizeX() && g->sizeY()==geometry_.sizeY() &&
    g->originX()==geometry_.originX() && g->originY()==geometry_.originY() &&
    g->resolution()==geometry_.resolution() && sameParameters(e.parameters(),evaluator_.parameters());
}
std::size_t FrozenGridCache::slot(GridCell c) {
  auto &id=slots_.at(*geometry_.index(c));
  if(id<0) {
    id=static_cast<int32_t>(nodes_.size());
    nodes_.push_back(evaluator_.evaluateNode(geometry_.cellCenter(c)));
    transitions_.emplace_back();
  }
  return static_cast<std::size_t>(id);
}
NodeEvaluation FrozenGridCache::node(GridCell c,const CostmapSnapshot & live) {
  NodeEvaluation n;
  auto cost=live.cost(c);
  if(!cost){n.reason=StepInvalidReason::kCostmapOutOfBounds;return n;}
  if(*cost>=253){n.raw_cost=*cost;n.reason=*cost==255 ? StepInvalidReason::kCostmapUnknown : StepInvalidReason::kCostmapCollision;return n;}
  n=nodes_[slot(c)];n.raw_cost=*cost;return n;
}
EdgeEvaluation FrozenGridCache::edge(GridCell a,GridCell b,const NodeEvaluation &na,const NodeEvaluation &nb) {
  EdgeEvaluation out;
  if(!na.valid || !nb.valid){out.reason=!na.valid?na.reason:nb.reason;return out;}
  int d=0;for(;d<8;++d) if(b.x-a.x==dx[d] && b.y-a.y==dy[d])break;
  if(d==8)return out;
  const auto ia=slot(a),ib=slot(b);
  auto &t=transitions_[ia][d];
  if(!t.ready) {
    auto e=evaluator_.evaluateGridTransition(a,b,nodes_[ia],nodes_[ib]);
    t.length=e.length_xy_m;
    t.height_cost=evaluator_.parameters().height_cost_weight*e.height_jump_score_m;
    t.sobel_cost=evaluator_.parameters().grid_sobel_gradient_cost_weight*e.sobel_gradient_exposure_m;
    t.reason=e.reason;
    if(e.valid && hazards_) {
      const auto q=hazards_->querySegment(geometry_.cellCenter(a),geometry_.cellCenter(b),
        evaluator_.parameters().grid_terrain_clearance_distance_m);
      if(!q.evidence_valid)t.reason=StepInvalidReason::kTerrainClearanceEvidenceMissing;
      else if(q.violation)t.reason=StepInvalidReason::kTerrainClearanceViolation;
    }
    t.ready=true;
  }
  out.valid=t.reason==StepInvalidReason::kNone;out.reason=t.reason;
  // Same floating-point addition order as the uncached evaluator.
  out.cost=evaluator_.parameters().distance_weight*t.length +
    evaluator_.parameters().inflation_cost_weight*(inflation_[nb.raw_cost]*t.length)+t.height_cost+t.sobel_cost;
  return out;
}
bool FrozenGridCache::warm(const std::atomic<bool> *cancel) {
  // Only costmap cells near observed height need evidence tests; all other cells
  // remain lazy-invalid. The extra cell accounts for differing lattice origins.
  std::vector<uint8_t> candidates(geometry_.cellCount(),0);
  std::vector<GridCell> cells;
  const int radius=static_cast<int>(std::ceil(evaluator_.parameters().node_evidence_radius_m/geometry_.resolution()))+1;
  for(int y=0;y<static_cast<int>(height_->sizeY());++y) {
    if(cancel && cancel->load())return false;
    for(int x=0;x<static_cast<int>(height_->sizeX());++x) {
      if(!height_->observed({x,y}))continue;
      auto c=geometry_.worldToCell(height_->cellCenter({x,y}));if(!c)continue;
      for(int j=-radius;j<=radius;++j)for(int i=-radius;i<=radius;++i) {
        GridCell n{c->x+i,c->y+j};auto idx=geometry_.index(n);
        if(idx && !candidates[*idx]){candidates[*idx]=1;cells.push_back(n);}
      }
    }
  }
  for(auto c:cells){if(cancel && cancel->load())return false;slot(c);}
  for(auto c:cells) {
    if(cancel && cancel->load())return false;
    const auto a=nodes_[slot(c)];if(!a.valid)continue;
    for(int d=0;d<8;++d){GridCell n{c.x+dx[d],c.y+dy[d]};
      if(geometry_.inBounds(n))edge(c,n,a,nodes_[slot(n)]);}
  }
  return true;
}
std::size_t FrozenGridCache::estimatedBytes() const {
  return slots_.capacity()*sizeof(int32_t)+geometry_.cellCount()+
    nodes_.size()*(sizeof(NodeEvaluation)+sizeof(std::array<Transition,8>));
}
}  // namespace rubi_heightmap_step_wavefront_planner
