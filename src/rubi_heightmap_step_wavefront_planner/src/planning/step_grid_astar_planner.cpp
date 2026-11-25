#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr double kTolerance = 1.0e-12;

// Search touches a fraction of a large global costmap. Initialize state in
// contiguous blocks on first use, and keep the blocks local to this request.
struct SearchCell
{
  double g{std::numeric_limits<double>::infinity()};
  std::size_t parent{std::numeric_limits<std::size_t>::max()};
  bool closed{false};
};

class SearchCells
{
public:
  explicit SearchCells(const std::size_t count)
  : blocks_(1U + (count - 1U) / kBlockSize) {}

  SearchCell & operator[](const std::size_t index)
  {
    auto & block = blocks_[index / kBlockSize];
    if (!block) {block = std::make_unique<Block>();}
    return (*block)[index % kBlockSize];
  }

private:
  static constexpr std::size_t kBlockSize = 128U;
  using Block = std::array<SearchCell, kBlockSize>;
  std::vector<std::unique_ptr<Block>> blocks_;
};

bool hardBlocked(const CostmapSnapshot & costmap, const GridCell cell)
{
  const auto cost = costmap.cost(cell);
  return !cost || *cost >= 253U;
}

void recordStepRejection(
  const EdgeEvaluation & edge, const double adjacent_threshold_m,
  PlanningStatistics & statistics)
{
  if (edge.reason != StepInvalidReason::kStepLimit) {return;}
  ++statistics.rejected_step_limit;
  const bool adjacent = edge.max_height_jump_m > adjacent_threshold_m;
  const bool sobel = edge.sobel_hard_rejection;
  statistics.adjacent_step_rejects += adjacent ? 1U : 0U;
  statistics.sobel_step_rejects += sobel ? 1U : 0U;
  statistics.both_step_rejects += adjacent && sobel ? 1U : 0U;
  statistics.local_relief_step_rejects += edge.local_relief_hard_rejection ? 1U : 0U;
}
}  // namespace

StepGridAStarPlanner::StepGridAStarPlanner(const GridAStarParameters parameters)
: parameters_(parameters)
{
  if (parameters_.max_expanded_states == 0U) {
    throw std::invalid_argument("grid max_expanded_states must be > 0");
  }
}

double StepGridAStarPlanner::octileDistance(const int dx, const int dy) noexcept
{
  const double x = std::abs(dx);
  const double y = std::abs(dy);
  return x + y + (std::sqrt(2.0) - 2.0) * std::min(x, y);
}

PlanResult StepGridAStarPlanner::plan(
  const StepEvaluator & evaluator, const Point2D start, const Point2D goal,
  const LocalReliefHazardSnapshot * terrain_hazards, FrozenGridCache * frozen) const
{
  const auto started = Clock::now();
  PlanResult result;
  const CostmapSnapshot * costmap = evaluator.costmap();
  if (!costmap || evaluator.mode() != StepEvaluationMode::kCostmapHeightHybrid) {
    result.message = "grid planner requires hybrid evaluator";
    return result;
  }
  if(frozen && !frozen->matches(evaluator)) {
    result.message="frozen grid context mismatch";return result;
  }
  if(frozen) terrain_hazards=frozen->hazards();
  const auto evaluate_node=[&](GridCell c) {return frozen ? frozen->node(c,*costmap) : evaluator.evaluateNode(costmap->cellCenter(c));};
  const bool clearance_enabled = evaluator.parameters().grid_terrain_clearance_enabled;
  if (clearance_enabled &&
    (!terrain_hazards || !terrain_hazards->matches(evaluator)))
  {
    result.message = "grid terrain clearance context unavailable";
    result.rejected.push_back({RejectionKind::kNode,
      StepInvalidReason::kTerrainClearanceContextUnavailable, start, start});
    return result;
  }
  if (terrain_hazards) {
    result.statistics.terrain_seed_count = terrain_hazards->seedCount();
    result.statistics.terrain_evidence_invalid_count =
      terrain_hazards->evidenceInvalidCount();
  }
  const auto start_cell = costmap->worldToCell(start);
  const auto goal_cell = costmap->worldToCell(goal);
  if (!start_cell || !goal_cell || !costmap->inBounds(*start_cell) ||
    !costmap->inBounds(*goal_cell))
  {
    result.message = "grid endpoint outside costmap";
    return result;
  }
  ++result.statistics.node_evaluation_calls;
  const NodeEvaluation start_evaluation = evaluate_node(*start_cell);
  ++result.statistics.node_evaluation_calls;
  const NodeEvaluation goal_evaluation = evaluate_node(*goal_cell);
  if (!start_evaluation.valid || !goal_evaluation.valid) {
    result.message = !start_evaluation.valid ? "grid start invalid" : "grid goal invalid";
    result.rejected.push_back({RejectionKind::kNode,
      !start_evaluation.valid ? start_evaluation.reason : goal_evaluation.reason,
      !start_evaluation.valid ? start : goal, !start_evaluation.valid ? start : goal});
    return result;
  }
  if (clearance_enabled) {
    const auto reject_endpoint = [&](const Point2D point) {
        const TerrainClearanceQuery query = terrain_hazards->queryPoint(
          point, evaluator.parameters().grid_terrain_clearance_distance_m);
        result.path_metrics.minimum_terrain_clearance_m = std::min(
          result.path_metrics.minimum_terrain_clearance_m, query.minimum_distance_m);
        result.path_metrics.minimum_terrain_clearance_exact =
          result.path_metrics.minimum_terrain_clearance_exact &&
          query.minimum_distance_exact;
        if (!query.evidence_valid) {
          ++result.statistics.grid_terrain_clearance_evidence_rejects;
          result.message = "grid endpoint terrain clearance evidence missing";
          if(!frozen) result.rejected.push_back({RejectionKind::kNode,
            StepInvalidReason::kTerrainClearanceEvidenceMissing, point, point});
          return true;
        }
        if (query.violation) {
          ++result.statistics.grid_terrain_clearance_rejects;
          result.message = "grid endpoint violates terrain clearance";
          if(!frozen) result.rejected.push_back({RejectionKind::kNode,
            StepInvalidReason::kTerrainClearanceViolation, point, point});
          return true;
        }
        return false;
      };
    if (reject_endpoint(costmap->cellCenter(*start_cell)) ||
      reject_endpoint(costmap->cellCenter(*goal_cell)))
    {
      return result;
    }
  }

  const std::size_t width = costmap->sizeX();
  const std::size_t count = costmap->cellCount();
  const auto toIndex = [width](const GridCell cell) {
      return static_cast<std::size_t>(cell.y) * width + static_cast<std::size_t>(cell.x);
    };
  const auto toCell = [width](const std::size_t index) {
      return GridCell{static_cast<int>(index % width), static_cast<int>(index / width)};
    };
  const std::size_t start_index = toIndex(*start_cell);
  const std::size_t goal_index = toIndex(*goal_cell);
  struct Entry {double f; double g; std::size_t index;};
  struct Greater {bool operator()(const Entry & left, const Entry & right) const noexcept {
    return std::tie(left.f, left.g, left.index) > std::tie(right.f, right.g, right.index);
  }};
  SearchCells state(count);
  std::vector<std::optional<NodeEvaluation>> node_evaluations(frozen ? 0 : count);
  std::vector<NodeId> graph_node_id(frozen ? 0U : count, count);
  if(!frozen){node_evaluations[start_index]=start_evaluation;node_evaluations[goal_index]=goal_evaluation;}
  const auto get_node=[&](std::size_t i) -> NodeEvaluation {
    if(frozen)return evaluate_node(toCell(i));
    if(!node_evaluations[i]){++result.statistics.node_evaluation_calls;node_evaluations[i]=evaluate_node(toCell(i));}
    return *node_evaluations[i];
  };
  const auto materialize_expanded_cell = [&](const std::size_t index) {
      if (graph_node_id[index] != count) {
        return;
      }
      const Point2D point = costmap->cellCenter(toCell(index));

      graph_node_id[index] = result.nodes.size();
      result.nodes.push_back({
        graph_node_id[index], point, get_node(index).elevation_m});
      const std::size_t parent = state[index].parent;
      if (parent < count && graph_node_id[parent] != count) {
        const EdgeEvaluation edge = evaluator.evaluateGridTransition(
          toCell(parent), toCell(index),
          get_node(parent), get_node(index));
        result.edges.push_back({graph_node_id[parent], graph_node_id[index], edge});
      }
    };
  std::priority_queue<Entry, std::vector<Entry>, Greater> open;
  const double weighted_resolution =
    evaluator.parameters().distance_weight * costmap->resolution();
  const auto priority = [&](const GridCell cell, const double path_cost) {
      return std::fma(weighted_resolution,
        octileDistance(cell.x - goal_cell->x, cell.y - goal_cell->y), path_cost);
    };
  state[start_index].g = 0.0;
  open.push({priority(*start_cell, 0.0), 0.0, start_index});
  ++result.statistics.astar_open_pushes;
  constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double resolution = costmap->resolution();
  const double minimum_step[] = {evaluator.parameters().distance_weight * resolution,
    evaluator.parameters().distance_weight * (resolution * std::hypot(1, 1))};
  bool found = false;
  while (!open.empty()) {
    if (parameters_.max_planning_time_ms > 0U &&
      std::chrono::duration<double, std::milli>(Clock::now() - started).count() >=
      static_cast<double>(parameters_.max_planning_time_ms))
    {
      result.termination = PlanTermination::kMaxGraphBuildTime;
      result.message = "grid planning timeout";
      break;
    }
    const Entry current = open.top();
    open.pop();
    auto & current_state = state[current.index];
    if (current_state.closed || std::abs(current.g - current_state.g) > kTolerance) {continue;}
    current_state.closed = true;
    if(!frozen) materialize_expanded_cell(current.index);
    ++result.expansions;
    if (result.expansions > parameters_.max_expanded_states) {
      result.termination = PlanTermination::kMaxExpansions;
      result.message = "grid expansion limit";
      break;
    }
    if (current.index == goal_index) {found = true; break;}
    const GridCell current_cell = toCell(current.index);
    const auto current_node = get_node(current.index);
    for (int direction = 0; direction < (parameters_.allow_diagonal ? 8 : 4); ++direction) {
      ++result.statistics.neighbor_candidates;
      const GridCell neighbor{current_cell.x + kDx[direction], current_cell.y + kDy[direction]};
      if (!costmap->inBounds(neighbor)) {continue;}
      const std::size_t neighbor_index = toIndex(neighbor);
      // Octile distance is consistent: each edge costs at least its weighted
      // grid distance, with nonnegative terrain and inflation penalties.
      // Closed neighbors cannot improve; skip their node/edge evaluation.
      auto & neighbor_state = state[neighbor_index];
      if (neighbor_state.closed) {continue;}
      // Every terrain and inflation penalty is nonnegative. Even a free
      // edge cannot improve this neighbor, including the parent tie-break.
      if (frozen && current_state.g + minimum_step[direction >= 4] >
        neighbor_state.g + kTolerance) {continue;}
      if (direction >= 4 && (hardBlocked(
          *costmap, {current_cell.x + kDx[direction], current_cell.y}) ||
        hardBlocked(*costmap, {current_cell.x, current_cell.y + kDy[direction]})))
      {
        continue;
      }
      const Point2D neighbor_point = costmap->cellCenter(neighbor);
      const NodeEvaluation node=get_node(neighbor_index);
      if (!node.valid) {
        if(!frozen) result.rejected.push_back({RejectionKind::kNode, node.reason,
          costmap->cellCenter(current_cell), neighbor_point});
        continue;
      }
      ++result.statistics.edge_evaluation_calls;
      const EdgeEvaluation edge = frozen ?
        frozen->edge(current_cell, neighbor, current_node, node) :
        evaluator.evaluateGridTransition(current_cell, neighbor, current_node, node);
      if (!edge.valid) {
        recordStepRejection(
          edge, evaluator.parameters().max_crossable_height_jump_m, result.statistics);
        if(!frozen) result.rejected.push_back({RejectionKind::kEdge, edge.reason,
          costmap->cellCenter(current_cell), neighbor_point});
        continue;
      }
      if (clearance_enabled && !frozen) {
        const TerrainClearanceQuery clearance = terrain_hazards->querySegment(
          costmap->cellCenter(current_cell), neighbor_point,
          evaluator.parameters().grid_terrain_clearance_distance_m);
        if (!clearance.evidence_valid || clearance.violation) {
          const StepInvalidReason reason = clearance.evidence_valid ?
            StepInvalidReason::kTerrainClearanceViolation :
            StepInvalidReason::kTerrainClearanceEvidenceMissing;
          result.statistics.grid_terrain_clearance_rejects +=
            clearance.violation ? 1U : 0U;
          result.statistics.grid_terrain_clearance_evidence_rejects +=
            clearance.evidence_valid ? 0U : 1U;
          if(!frozen) result.rejected.push_back({RejectionKind::kEdge, reason,
            costmap->cellCenter(current_cell), neighbor_point});
          continue;
        }
      }
      const double tentative = current_state.g + edge.cost;
      if (tentative + kTolerance < neighbor_state.g ||
        (std::abs(tentative - neighbor_state.g) <= kTolerance &&
        current.index < neighbor_state.parent))
      {
        neighbor_state.g = tentative;
        neighbor_state.parent = current.index;
        open.push({priority(neighbor, tentative), tentative, neighbor_index});
        ++result.statistics.astar_open_pushes;
      }
    }
  }
  result.astar_time_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - started).count();
  result.statistics.expanded_states = result.expansions;
  result.statistics.accepted_nodes = result.nodes.size();
  result.statistics.accepted_edges = result.edges.size();
  const auto copy_instrumentation = [&]() {
      const auto & instrumentation = evaluator.instrumentation();
      result.statistics.edge_samples_total = instrumentation.edge_samples_total;
      result.statistics.height_evidence_queries = instrumentation.height_evidence_queries;
      result.statistics.costmap_queries = instrumentation.costmap_queries;
      result.statistics.local_relief_queries = instrumentation.local_relief_queries;
      result.statistics.local_relief_cache_hits = instrumentation.local_relief_cache_hits;
      result.statistics.local_relief_missing_neighborhoods =
        instrumentation.local_relief_missing_neighborhoods;
      result.statistics.supported_relief_queries = instrumentation.supported_relief_queries;
      result.statistics.grid_transition_evaluations =
        instrumentation.grid_transition_evaluations;
      result.statistics.grid_sobel_queries = instrumentation.grid_sobel_queries;
      result.statistics.grid_sobel_cache_hits = instrumentation.grid_sobel_cache_hits;
      result.statistics.grid_sobel_5x5_valid = instrumentation.grid_sobel_5x5_valid;
      result.statistics.grid_sobel_5x5_fallback_to_3x3 =
        instrumentation.grid_sobel_5x5_fallback_to_3x3;
      result.statistics.grid_sobel_missing = instrumentation.grid_sobel_missing;
    };
  copy_instrumentation();
  if (!found) {
    if (result.message.empty()) {
      result.termination = PlanTermination::kFrontierExhausted;
      result.message = "grid path not found";
    }
    result.core_total_time_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - started).count();
    return result;
  }
  std::vector<std::size_t> path_cells;
  for (std::size_t trace = goal_index; ; trace = state[trace].parent) {
    path_cells.push_back(trace);
    if (trace == start_index) {break;}
    if (state[trace].parent >= count || path_cells.size() > count) {
      path_cells.clear();
      break;
    }
  }
  std::reverse(path_cells.begin(), path_cells.end());
  if (path_cells.empty()) {return result;}
  const auto finalize_started = Clock::now();
  result.path_node_ids.reserve(path_cells.size());
  if (frozen) {
    result.nodes.reserve(path_cells.size());
    result.edges.reserve(path_cells.size() - 1U);
  }
  for (std::size_t path_index = 0U; path_index < path_cells.size(); ++path_index) {
    const std::size_t cell_index = path_cells[path_index];
    if (frozen) {
      const GridCell cell = toCell(cell_index);
      result.nodes.push_back({path_index, costmap->cellCenter(cell), get_node(cell_index).elevation_m});
      result.path_node_ids.push_back(path_index);
    } else {
      result.path_node_ids.push_back(graph_node_id[cell_index]);
    }
    if (path_index == 0U) {continue;}
    const std::size_t previous_index = path_cells[path_index - 1U];
    const EdgeEvaluation edge = evaluator.evaluateGridTransition(
      toCell(previous_index), toCell(cell_index),
      get_node(previous_index), get_node(cell_index));
    if (frozen) {result.edges.push_back({path_index - 1U, path_index, edge});}
    result.path_metrics.length_xy_m += edge.length_xy_m;
    result.path_metrics.height_event_count += edge.height_jump_event_count;
    result.path_metrics.max_height_jump_m = std::max(
      result.path_metrics.max_height_jump_m, edge.max_height_jump_m);
    result.path_metrics.max_sobel_equivalent_step_height_m = std::max(
      result.path_metrics.max_sobel_equivalent_step_height_m,
      edge.max_sobel_equivalent_step_height_m);
    result.path_metrics.max_sobel_gradient = std::max(
      result.path_metrics.max_sobel_gradient, edge.max_sobel_gradient);
    result.path_metrics.sobel_gradient_exposure_m += edge.sobel_gradient_exposure_m;
    result.path_metrics.max_grid_sobel_gradient = std::max(
      result.path_metrics.max_grid_sobel_gradient, edge.max_grid_sobel_gradient);
    result.path_metrics.max_local_relief_m = std::max(
      result.path_metrics.max_local_relief_m, edge.max_local_relief_m);
    result.path_metrics.max_supported_local_relief_m = std::max(
      result.path_metrics.max_supported_local_relief_m,
      edge.max_supported_local_relief_m);
    result.path_metrics.height_score_m += edge.height_jump_score_m;
    result.path_metrics.inflation_score_m += edge.inflation_score_m;
    result.path_metrics.maximum_raw_cost = std::max(
      result.path_metrics.maximum_raw_cost, edge.maximum_raw_cost);
    if (clearance_enabled) {
      const TerrainClearanceQuery clearance = terrain_hazards->querySegment(
        costmap->cellCenter(toCell(previous_index)),
        costmap->cellCenter(toCell(cell_index)),
        evaluator.parameters().grid_terrain_clearance_distance_m);
      result.path_metrics.minimum_terrain_clearance_m = std::min(
        result.path_metrics.minimum_terrain_clearance_m,
        clearance.minimum_distance_m);
      result.path_metrics.minimum_terrain_clearance_exact =
        result.path_metrics.minimum_terrain_clearance_exact &&
        clearance.minimum_distance_exact;
    }
  }
  result.path_metrics.height_cost = evaluator.parameters().height_cost_weight *
    result.path_metrics.height_score_m;
  result.path_metrics.inflation_cost = evaluator.parameters().inflation_cost_weight *
    result.path_metrics.inflation_score_m;
  result.path_metrics.sobel_cost = evaluator.parameters().grid_sobel_gradient_cost_weight *
    result.path_metrics.sobel_gradient_exposure_m;
  result.path_metrics.total_cost = state[goal_index].g;
  result.path_metrics.grid_search_cost = state[goal_index].g;
  result.path_metrics.grid_validation_cost =
    evaluator.parameters().distance_weight * result.path_metrics.length_xy_m +
    result.path_metrics.height_cost + result.path_metrics.inflation_cost +
    result.path_metrics.sobel_cost;
  result.success = true;
  result.termination = PlanTermination::kPostGoalComplete;
  result.message = "grid path found";
  result.statistics.accepted_nodes = result.nodes.size();
  result.statistics.accepted_edges = result.edges.size();
  copy_instrumentation();
  result.path_finalize_time_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - finalize_started).count();
  result.core_total_time_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - started).count();
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
