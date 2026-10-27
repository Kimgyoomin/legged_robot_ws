#include <gtest/gtest.h>
#include <limits>
#include "legged_navigation/sector_filter.hpp"

using legged_navigation::SectorFilter;
const std::array<double,6> identity{1,0,0,0,1,0};

TEST(SectorFilter, NearFrontOnlyAndRangeBounds) {
  SectorFilter f(.6,5,.3,60);
  EXPECT_TRUE(f.keep(.3,0,0,identity));
  EXPECT_FALSE(f.keep(.299,0,0,identity));
  EXPECT_FALSE(f.keep(-.4,0,0,identity));
  EXPECT_FALSE(f.keep(0,.4,0,identity));
  EXPECT_TRUE(f.keep(-.6,0,0,identity));
  EXPECT_TRUE(f.keep(5,0,0,identity));
  EXPECT_FALSE(f.keep(5.001,0,0,identity));
  for (double sign : {-1.,1.}) {
    const double inside=59.999*std::acos(-1.)/180.;
    const double outside=60.001*std::acos(-1.)/180.;
    EXPECT_TRUE(f.keep(.4*std::cos(inside),sign*.4*std::sin(inside),0,identity));
    EXPECT_FALSE(f.keep(.4*std::cos(outside),sign*.4*std::sin(outside),0,identity));
  }
}

TEST(SectorFilter, RangeIs3DAndBearingIsNotVertical) {
  SectorFilter f(.6,5,.3,60);
  EXPECT_TRUE(f.keep(.1,0,.3,identity));
  EXPECT_FALSE(f.keep(0,0,.4,identity));
  EXPECT_FALSE(f.keep(0,0,0,identity));
  EXPECT_FALSE(f.keep(std::numeric_limits<double>::quiet_NaN(),0,0,identity));
  EXPECT_FALSE(f.keep(std::numeric_limits<double>::infinity(),0,0,identity));
}

TEST(SectorFilter, UsesMountRotationNotSensorXAxis) {
  SectorFilter f(.6,5,.3,60);
  const std::array<double,6> yaw90{0,-1,0,1,0,0};
  EXPECT_TRUE(f.keep(0,-.4,0,yaw90));  // Sensor -Y is robot +X.
  EXPECT_FALSE(f.keep(.4,0,0,yaw90));
  const std::array<double,6> inverted{1,0,0,0,-1,0};
  EXPECT_TRUE(f.keep(.4,.1,0,inverted));
}

TEST(SectorFilter, DisableAndInvalidSettings) {
  SectorFilter legacy(.6,5,.3,0);
  EXPECT_FALSE(legacy.keep(.4,0,0,identity));
  EXPECT_TRUE(legacy.keep(.7,0,0,identity));
  EXPECT_THROW(SectorFilter(.6,5,.7,60), std::invalid_argument);
  EXPECT_THROW(SectorFilter(.6,5,.3,180), std::invalid_argument);
  EXPECT_THROW(SectorFilter(.6,5,-.1,60), std::invalid_argument);
}
