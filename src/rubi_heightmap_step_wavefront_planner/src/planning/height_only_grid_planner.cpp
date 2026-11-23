#include "rubi_heightmap_step_wavefront_planner/planning/height_only_grid_planner.hpp"
#include <stdexcept>

namespace rubi_heightmap_step_wavefront_planner {
namespace {
StepEvaluatorParameters heightParameters(StepEvaluatorParameters p) {
  p.inflation_cost_weight = 0.;
  p.clearance_cost_weight = 0.;
  return p;
}
CostmapSnapshot observedMask(const std::shared_ptr<const HeightmapSnapshot> &height) {
  if (!height) throw std::invalid_argument("HeightOnlyGridPlanner needs a height map");
  std::vector<std::uint8_t> mask(height->cellCount(), 255);
  for (std::size_t y=0; y<height->sizeY(); ++y)
    for (std::size_t x=0; x<height->sizeX(); ++x)
      if (height->observed({int(x), int(y)})) mask[y*height->sizeX()+x] = 0;
  // Heightmap origins are cell centers; CostmapSnapshot origins are boundaries.
  return CostmapSnapshot::fromData(height->sizeX(), height->sizeY(), height->resolution(),
    height->originX()-.5*height->resolution(), height->originY()-.5*height->resolution(), std::move(mask));
}
}
HeightOnlyGridPlanner::HeightOnlyGridPlanner(std::shared_ptr<const HeightmapSnapshot> height,
    StepEvaluatorParameters evaluation, GridAStarParameters search)
  : height_(std::move(height)), evaluation_(heightParameters(evaluation)),
    observed_mask_(observedMask(height_)), search_(search), cache_(height_, observed_mask_, evaluation_) {}
bool HeightOnlyGridPlanner::warm(const std::atomic<bool> *cancel) { return cache_.warm(cancel); }
PlanResult HeightOnlyGridPlanner::plan(Point2D start, Point2D goal) {
  const StepEvaluator evaluator(*height_, observed_mask_, evaluation_);
  return StepGridAStarPlanner(search_).plan(evaluator, start, goal, nullptr, &cache_);
}
}  // namespace rubi_heightmap_step_wavefront_planner
