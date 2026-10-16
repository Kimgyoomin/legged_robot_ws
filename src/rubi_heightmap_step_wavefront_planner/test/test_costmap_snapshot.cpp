#include <cstdint>
#include <vector>
#include <cmath>
#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/map/costmap_snapshot.hpp"
namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(CostmapSnapshot, GeometryIndexRoundTripAndRawSemantics)
{
  const auto map = planner::CostmapSnapshot::fromData(
    5U, 1U, 0.05, -1.0, 2.0, {0U, 252U, 253U, 254U, 255U});
  EXPECT_EQ(map.cellCount(), 5U);
  EXPECT_DOUBLE_EQ(map.originX(), -1.0);
  for (int x = 0; x < 5; ++x) {
    const planner::GridCell cell{x, 0};
    const auto world = map.cellCenter(cell);
    ASSERT_TRUE(map.worldToCell(world));
    EXPECT_EQ(*map.worldToCell(world), cell);
    EXPECT_EQ(*map.index(cell), static_cast<std::size_t>(x));
    EXPECT_EQ(*map.cost(cell), static_cast<std::uint8_t>(x == 0 ? 0 : 251 + x));
  }
  EXPECT_FALSE(map.cost({5, 0}));
  EXPECT_FALSE(map.worldToCell({std::numeric_limits<double>::infinity(), 0.0}));
}

TEST(CostmapSnapshot, RejectsMalformedGeometryAndData)
{
  EXPECT_THROW(
    planner::CostmapSnapshot::fromData(2U, 2U, 0.05, 0.0, 0.0, {0U}),
    std::invalid_argument);
  EXPECT_THROW(
    planner::CostmapSnapshot::fromData(1U, 1U, 0.0, 0.0, 0.0, {0U}),
    std::invalid_argument);
}

TEST(CostmapSnapshot, ContentHashDeduplicatesAndDetectsGeometryOrCellChanges)
{
  const auto first = planner::CostmapSnapshot::fromData(
    3U, 2U, 0.05, -1.0, 2.0, {0U, 1U, 2U, 3U, 4U, 5U});
  const auto same = planner::CostmapSnapshot::fromData(
    3U, 2U, 0.05, -1.0, 2.0, {0U, 1U, 2U, 3U, 4U, 5U});
  const auto changed_cell = planner::CostmapSnapshot::fromData(
    3U, 2U, 0.05, -1.0, 2.0, {0U, 1U, 254U, 3U, 4U, 5U});
  const auto changed_origin = planner::CostmapSnapshot::fromData(
    3U, 2U, 0.05, -0.95, 2.0, {0U, 1U, 2U, 3U, 4U, 5U});
  EXPECT_EQ(first.contentHash(), same.contentHash());
  EXPECT_NE(first.contentHash(), changed_cell.contentHash());
  EXPECT_NE(first.contentHash(), changed_origin.contentHash());
}

TEST(CostmapSnapshot, StopEnvelopeClearanceDoesNotModifyPhysicalMapOrDoubleInflate)
{
  std::vector<std::uint8_t> data(101*101, 200U);
  data[50*101+50]=254U;
  const auto original=planner::CostmapSnapshot::fromData(101,101,.05,0.,0.,data);
  const auto padded=original.withObstacleClearance(std::hypot(.5,.45));
  EXPECT_EQ(*padded.cost({62,50}),253U);  // 0.60 m is inside the rotated envelope.
  EXPECT_EQ(*padded.cost({66,50}),200U);  // 0.80 m remains soft, not inflated twice.
  EXPECT_EQ(*padded.cost({50,50}),254U);
  EXPECT_EQ(*original.cost({62,50}),200U);
  EXPECT_EQ(original.withObstacleClearance(0.).contentHash(),original.contentHash());
  EXPECT_NE(padded.contentHash(),original.contentHash());
  EXPECT_THROW(original.withObstacleClearance(-1.),std::invalid_argument);
  EXPECT_THROW(original.withObstacleClearance(NAN),std::invalid_argument);
}

TEST(CostmapSnapshot, ClearanceBlocksUnknownAndMapBoundary)
{
  std::vector<std::uint8_t> data(101*101,0U);data[50*101+50]=255U;
  const auto padded=planner::CostmapSnapshot::fromData(101,101,.05,0.,0.,data)
    .withObstacleClearance(std::hypot(.5,.45));
  EXPECT_EQ(*padded.cost({50,50}),255U);
  EXPECT_EQ(*padded.cost({62,50}),253U);
  EXPECT_EQ(*padded.cost({10,30}),253U);
  EXPECT_EQ(*padded.cost({20,30}),0U);
}

TEST(CostmapSnapshot, BoundarySeedOptimizationMatchesBruteForceDiskExclusion)
{
  constexpr int n=40;
  constexpr double resolution=.1, radius=.35;
  std::vector<std::uint8_t> data(n*n,100U);
  for (int y=14;y<26;++y) for(int x=14;x<26;++x) data[y*n+x]=255U;
  data[8*n+8]=254U;data[32*n+30]=254U;
  const auto map=planner::CostmapSnapshot::fromData(n,n,resolution,0.,0.,data);
  const auto padded=map.withObstacleClearance(radius);
  for (int y=0;y<n;++y) for(int x=0;x<n;++x) {
    bool blocked=std::min({x+.5,n-x-.5,y+.5,n-y-.5})*resolution<=
      radius+std::sqrt(.5)*resolution;
    for (int sy=0;sy<n && !blocked;++sy) for(int sx=0;sx<n;++sx) {
      if(data[sy*n+sx]>=254 && std::hypot(x-sx,y-sy)*resolution<=
        radius+std::sqrt(2.)*resolution) {blocked=true;break;}
    }
    EXPECT_EQ(*padded.cost({x,y}),blocked ? std::max<int>(253,data[y*n+x]):data[y*n+x])
      << x << ',' << y;
  }
}

TEST(CostmapSnapshot, ATooNarrowMapHasNoClearPlanningCells)
{
  const auto map=planner::CostmapSnapshot::fromData(12,50,.1,0.,0.,std::vector<std::uint8_t>(600,0));
  const auto padded=map.withObstacleClearance(std::hypot(.5,.45));
  for(int y=0;y<50;++y) for(int x=0;x<12;++x) EXPECT_EQ(*padded.cost({x,y}),253U);
}
