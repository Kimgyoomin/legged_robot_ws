#pragma once
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"

namespace rubi_heightmap_step_wavefront_planner {
// The grid is an observed/unknown mask derived exclusively from height evidence.
// There is deliberately no API accepting a Nav2/OccupancyGrid obstacle costmap.
class HeightOnlyGridPlanner {
 public:
  HeightOnlyGridPlanner(std::shared_ptr<const HeightmapSnapshot> height,
                       StepEvaluatorParameters evaluation, GridAStarParameters search);
  bool warm(const std::atomic<bool> *cancel = nullptr);
  PlanResult plan(Point2D start, Point2D goal);
  std::size_t cachedNodes() const { return cache_.cachedNodes(); }
  std::size_t estimatedBytes() const { return cache_.estimatedBytes(); }
 private:
  std::shared_ptr<const HeightmapSnapshot> height_;
  StepEvaluatorParameters evaluation_;
  CostmapSnapshot observed_mask_;
  GridAStarParameters search_;
  FrozenGridCache cache_;
};
}  // namespace rubi_heightmap_step_wavefront_planner
