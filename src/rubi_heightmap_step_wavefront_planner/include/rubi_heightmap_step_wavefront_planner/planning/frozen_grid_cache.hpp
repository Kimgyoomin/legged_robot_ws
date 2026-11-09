#pragma once
#include <array>
#include <atomic>
#include <deque>
#include <memory>
#include "rubi_heightmap_step_wavefront_planner/terrain/local_relief_hazard_snapshot.hpp"

namespace rubi_heightmap_step_wavefront_planner {
// One immutable terrain context. Costmap *values* are deliberately not cached.
// Thread confinement: construct/warm in one worker, then use on the planning thread.
class FrozenGridCache {
public:
  FrozenGridCache(std::shared_ptr<const HeightmapSnapshot> height,
                  const CostmapSnapshot & geometry, StepEvaluatorParameters parameters);
  FrozenGridCache(const FrozenGridCache &) = delete;
  FrozenGridCache & operator=(const FrozenGridCache &) = delete;
  bool matches(const StepEvaluator & evaluator) const;
  NodeEvaluation node(GridCell cell, const CostmapSnapshot & live);
  EdgeEvaluation edge(GridCell from, GridCell to, const NodeEvaluation & a,
                      const NodeEvaluation & b);
  const LocalReliefHazardSnapshot * hazards() const {return hazards_.get();}
  bool warm(const std::atomic<bool> * cancel = nullptr);
  std::size_t cachedNodes() const {return nodes_.size();}
  std::size_t estimatedBytes() const;
private:
  struct Transition {
    double length{0}, height_cost{0}, sobel_cost{0};
    StepInvalidReason reason{StepInvalidReason::kInvalidInput};
    bool ready{false};
  };
  std::size_t slot(GridCell cell);
  std::shared_ptr<const HeightmapSnapshot> height_;
  CostmapSnapshot geometry_;
  StepEvaluator evaluator_;
  std::unique_ptr<LocalReliefHazardSnapshot> hazards_;
  std::vector<int32_t> slots_;
  std::deque<NodeEvaluation> nodes_;
  std::deque<std::array<Transition,8>> transitions_;
  std::array<double,253> inflation_;
};
}  // namespace rubi_heightmap_step_wavefront_planner
