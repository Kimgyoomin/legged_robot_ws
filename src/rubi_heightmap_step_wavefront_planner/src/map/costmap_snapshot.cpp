#include "rubi_heightmap_step_wavefront_planner/map/costmap_snapshot.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void hashBytes(std::uint64_t & hash, const void * data, const std::size_t size) noexcept
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0U; index < size; ++index) {
    hash ^= bytes[index];
    hash *= kFnvPrime;
  }
}
}  // namespace

CostmapSnapshot CostmapSnapshot::fromData(
  const std::size_t size_x, const std::size_t size_y, const double resolution_m,
  const double origin_x, const double origin_y, std::vector<std::uint8_t> costs)
{
  if (size_x == 0U || size_y == 0U ||
    size_x > std::numeric_limits<std::size_t>::max() / size_y ||
    costs.size() != size_x * size_y || !std::isfinite(resolution_m) ||
    resolution_m <= 0.0 || !std::isfinite(origin_x) || !std::isfinite(origin_y))
  {
    throw std::invalid_argument("invalid costmap snapshot geometry or data size");
  }
  CostmapSnapshot snapshot;
  snapshot.size_x_ = size_x;
  snapshot.size_y_ = size_y;
  snapshot.resolution_m_ = resolution_m;
  snapshot.origin_x_ = origin_x;
  snapshot.origin_y_ = origin_y;
  snapshot.costs_ = std::move(costs);
  std::uint64_t hash = kFnvOffset;
  hashBytes(hash, &snapshot.size_x_, sizeof(snapshot.size_x_));
  hashBytes(hash, &snapshot.size_y_, sizeof(snapshot.size_y_));
  hashBytes(hash, &snapshot.resolution_m_, sizeof(snapshot.resolution_m_));
  hashBytes(hash, &snapshot.origin_x_, sizeof(snapshot.origin_x_));
  hashBytes(hash, &snapshot.origin_y_, sizeof(snapshot.origin_y_));
  hashBytes(hash, snapshot.costs_.data(), snapshot.costs_.size());
  snapshot.content_hash_ = hash;
  return snapshot;
}

bool CostmapSnapshot::inBounds(const GridCell cell) const noexcept
{
  return cell.x >= 0 && cell.y >= 0 &&
         static_cast<std::size_t>(cell.x) < size_x_ &&
         static_cast<std::size_t>(cell.y) < size_y_;
}

std::optional<std::size_t> CostmapSnapshot::index(const GridCell cell) const noexcept
{
  if (!inBounds(cell)) {return std::nullopt;}
  return static_cast<std::size_t>(cell.y) * size_x_ + static_cast<std::size_t>(cell.x);
}

std::optional<GridCell> CostmapSnapshot::worldToCell(const Point2D point) const noexcept
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {return std::nullopt;}
  const double x = std::floor((point.x - origin_x_) / resolution_m_);
  const double y = std::floor((point.y - origin_y_) / resolution_m_);
  if (x < static_cast<double>(std::numeric_limits<int>::min()) ||
    x > static_cast<double>(std::numeric_limits<int>::max()) ||
    y < static_cast<double>(std::numeric_limits<int>::min()) ||
    y > static_cast<double>(std::numeric_limits<int>::max()))
  {
    return std::nullopt;
  }
  return GridCell{static_cast<int>(x), static_cast<int>(y)};
}

Point2D CostmapSnapshot::cellCenter(const GridCell cell) const noexcept
{
  return {origin_x_ + (static_cast<double>(cell.x) + 0.5) * resolution_m_,
    origin_y_ + (static_cast<double>(cell.y) + 0.5) * resolution_m_};
}

std::optional<std::uint8_t> CostmapSnapshot::cost(const GridCell cell) const noexcept
{
  const auto cell_index = index(cell);
  return cell_index ? std::optional<std::uint8_t>{costs_[*cell_index]} : std::nullopt;
}

std::optional<std::uint8_t> CostmapSnapshot::costAt(const Point2D point) const noexcept
{
  const auto cell = worldToCell(point);
  return cell ? cost(*cell) : std::nullopt;
}

CostmapSnapshot CostmapSnapshot::withObstacleClearance(const double radius_m) const
{
  if (!std::isfinite(radius_m) || radius_m < 0.0) {
    throw std::invalid_argument("costmap clearance must be finite and nonnegative");
  }
  if (radius_m == 0.0) {return *this;}
  auto padded = costs_;
  // Cover source-cell and query-cell areas, not only their centers. Paths can
  // be smoothed anywhere inside a cell. 253 is planner exclusion, never a new
  // physical obstacle (254); the final guard still receives the original map.
  const double reach = radius_m + std::sqrt(2.0)*resolution_m_;
  const double border = radius_m + std::sqrt(0.5)*resolution_m_;
  const int width = static_cast<int>(size_x_), height = static_cast<int>(size_y_);
  auto block = [&](int x, int y) {
      auto & value = padded[static_cast<std::size_t>(y)*size_x_+x];
      value = std::max<std::uint8_t>(value, 253U);
    };
  if (2.0*border >= std::min(size_x_, size_y_)*resolution_m_) {
    for (auto & value : padded) {value = std::max<std::uint8_t>(value, 253U);}
  } else {
    const int cells = static_cast<int>(std::ceil(reach/resolution_m_));
    std::vector<int> spans(2*cells+1, -1);
    for (int dy=-cells; dy<=cells; ++dy) {
      const double remaining = reach*reach-std::pow(dy*resolution_m_, 2);
      if (remaining >= 0.) {spans[dy+cells] = static_cast<int>(std::floor(
          std::sqrt(remaining)/resolution_m_));}
    }
    for (int y=0; y<height; ++y) {
      for (int x=0; x<width; ++x) {
        if (std::min({x+.5, width-x-.5, y+.5, height-y-.5})*resolution_m_ <= border) {
          block(x,y);
        }
        if (costs_[static_cast<std::size_t>(y)*size_x_+x] < 254U) {continue;}
        // Only boundary obstacle cells can be nearest to free space. Skipping
        // interiors keeps a large unknown background cheap to process.
        bool boundary = false;
        for (int dy=-1; dy<=1 && !boundary; ++dy) {
          for (int dx=-1; dx<=1; ++dx) {
            const int nx=x+dx, ny=y+dy;
            if (nx>=0 && nx<width && ny>=0 && ny<height &&
              costs_[static_cast<std::size_t>(ny)*size_x_+nx] < 254U)
            {boundary=true; break;}
          }
        }
        if (!boundary) {continue;}
        for (int dy=-cells; dy<=cells; ++dy) {
          const int ny=y+dy, span=spans[dy+cells];
          if (ny<0 || ny>=height || span<0) {continue;}
          for (int nx=std::max(0,x-span); nx<=std::min(width-1,x+span); ++nx) {
            block(nx,ny);
          }
        }
      }
    }
  }
  return fromData(size_x_, size_y_, resolution_m_, origin_x_, origin_y_, std::move(padded));
}

}  // namespace rubi_heightmap_step_wavefront_planner
