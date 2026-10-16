#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "pongbot_navigation/pure_pursuit_core.hpp"

namespace pp = pongbot_navigation::pure_pursuit;

TEST(PurePursuitCore, StraightPathProducesZeroAngularVelocity)
{
  const auto curvature = pp::computeCurvature({0.35, 0.0});
  ASSERT_TRUE(curvature);
  EXPECT_NEAR(*curvature, 0.0, 1.0e-12);
  EXPECT_NEAR(pp::computeDesiredCommand(*curvature, {}).angular, 0.0, 1.0e-12);
}

TEST(PurePursuitCore, CurveDirectionControlsAngularSign)
{
  ASSERT_TRUE(pp::computeCurvature({0.35, 0.1}));
  EXPECT_GT(pp::computeDesiredCommand(*pp::computeCurvature({0.35, 0.1}), {}).angular, 0.0);
  EXPECT_LT(pp::computeDesiredCommand(*pp::computeCurvature({0.35, -0.1}), {}).angular, 0.0);
}

TEST(PurePursuitCore, CommandsRespectHardSaturation)
{
  pp::Limits limits;
  limits.nominal_linear_velocity = 5.0;
  const auto straight = pp::computeDesiredCommand(0.0, limits);
  const auto sharp = pp::computeDesiredCommand(1000.0, limits);
  EXPECT_LE(straight.linear, 0.70);
  EXPECT_LE(std::abs(sharp.angular), 1.50);
  EXPECT_GE(sharp.linear, 0.0);
}

TEST(PurePursuitCore, LookaheadInterpolatesAlongArcLength)
{
  const std::vector<pp::Point2D> path{{0.0, 0.0}, {0.2, 0.0}, {0.2, 0.4}};
  const auto target = pp::selectLookaheadTarget(path, 0, 0.35);
  ASSERT_TRUE(target);
  EXPECT_NEAR(target->point.x, 0.2, 1.0e-12);
  EXPECT_NEAR(target->point.y, 0.15, 1.0e-12);
  EXPECT_EQ(target->path_index, 2U);
}

TEST(PurePursuitCore, NearestProgressIsMonotonic)
{
  const std::vector<pp::Point2D> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  EXPECT_EQ(pp::findNearestPathIndex(path, {0.1, 0.0}, 2, 2), 2U);
  EXPECT_EQ(pp::findNearestPathIndex(path, {2.8, 0.0}, 2, 2), 3U);
}

TEST(PurePursuitCore, RejectsTargetsBehindRobot)
{
  const std::vector<pp::Point2D> behind{{-2.0, 0.0}, {-1.0, 0.0}};
  EXPECT_FALSE(pp::selectForwardTarget(behind, {}, 0, 0.35));
  const std::vector<pp::Point2D> crossing{{-0.2, 0.0}, {0.4, 0.0}};
  const auto target = pp::selectForwardTarget(crossing, {}, 0, 0.1);
  ASSERT_TRUE(target);
  EXPECT_GT(target->point.x, 0.0);
}

TEST(PurePursuitCore, DetectsGoalTolerance)
{
  EXPECT_TRUE(pp::withinGoalTolerance({0.0, 0.0}, {0.1, 0.0}, 0.2));
  EXPECT_FALSE(pp::withinGoalTolerance({0.0, 0.0}, {0.3, 0.0}, 0.2));
}

TEST(PurePursuitCore, EmptyAndNonfinitePathsAreInvalid)
{
  EXPECT_FALSE(pp::isFinitePath({}));
  EXPECT_FALSE(pp::isFinitePath({{0.0, std::numeric_limits<double>::quiet_NaN()}}));
  EXPECT_TRUE(pp::isFinitePath({{0.0, 0.0}, {1.0, 0.0}}));
}

TEST(PurePursuitCore, CrossTrackDistanceSupportsSafetyThreshold)
{
  EXPECT_GT(pp::distance({0.0, 0.5}, {0.0, 0.0}), 0.40);
  EXPECT_LT(pp::distance({0.0, 0.1}, {0.0, 0.0}), 0.40);
}

TEST(PurePursuitCore, RateLimiterBoundsChanges)
{
  const auto command = pp::rateLimit({}, {0.7, 1.5}, 0.05, 0.5, 1.5);
  EXPECT_NEAR(command.linear, 0.025, 1.0e-12);
  EXPECT_NEAR(command.angular, 0.075, 1.0e-12);
}

TEST(PurePursuitCore, BehindTargetRequestsOnlyRotation)
{
  const auto a=pp::headingAlignment({-1.,.01},false,.7,.25,1.,.4);
  EXPECT_TRUE(a.active);EXPECT_DOUBLE_EQ(a.command.linear,0.);
  EXPECT_DOUBLE_EQ(a.command.angular,.4);
  const auto b=pp::headingAlignment({-1.,-.01},false,.7,.25,1.,.4);
  EXPECT_DOUBLE_EQ(b.command.angular,-.4);
}

TEST(PurePursuitCore, HeadingHysteresisAvoidsModeChatter)
{
  const pp::Point2D target{std::cos(.4),std::sin(.4)};
  EXPECT_FALSE(pp::headingAlignment(target,false,.7,.25,1.,.4).active);
  EXPECT_TRUE(pp::headingAlignment(target,true,.7,.25,1.,.4).active);
  EXPECT_FALSE(pp::headingAlignment({1.,.1},true,.7,.25,1.,.4).active);
}

TEST(PurePursuitCore, AlignmentBrakesBeforeTurningAndBoundsYawAcceleration)
{
  pp::Command prev{.5,0.};
  for(int i=0;i<10;++i) {
    const auto next=pp::limitTrackingCommand(prev,{0.,.4},true,.1,.5,1.);
    EXPECT_NEAR(next.angular,0.,1.e-12);EXPECT_LE(next.linear,prev.linear);
    prev=next;
  }
  const auto next=pp::limitTrackingCommand(prev,{0.,.4},true,.1,.5,1.);
  EXPECT_DOUBLE_EQ(next.linear,0.);EXPECT_NEAR(next.angular,.1,1.e-12);
}

TEST(PurePursuitCore, NonfiniteAlignmentTargetDoesNotRequestMotion)
{
  const auto a=pp::headingAlignment({NAN,0.},false,.7,.25,1.,.4);
  EXPECT_TRUE(a.active);EXPECT_DOUBLE_EQ(a.command.linear,0.);EXPECT_DOUBLE_EQ(a.command.angular,0.);
}
