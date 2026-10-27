#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <chrono>
#include <fstream>
#include <iterator>
#include <thread>

namespace {
using Status = BT::NodeStatus;
using Clock = std::chrono::steady_clock;

struct Scenario {
  int plan_calls{0}, follow_calls{0}, halts{0};
  bool plan_blocked{false}, follow_blocked{false}, arrive{false};
  std::string observed_goal;
};

class Follow : public BT::StatefulActionNode {
 public:
  Follow(const std::string &name, const BT::NodeConfiguration &cfg, Scenario &state)
      : BT::StatefulActionNode(name, cfg), s(state) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("path"), BT::InputPort<std::string>("controller_id")};
  }
  Status onStart() override { return step(); }
  Status onRunning() override { return step(); }
  void onHalted() override { ++s.halts; }
 private:
  Status step() {
    ++s.follow_calls;
    if (s.follow_blocked) return Status::FAILURE;
    return s.arrive ? Status::SUCCESS : Status::RUNNING;
  }
  Scenario &s;
};

class NavigationRecovery : public ::testing::TestWithParam<const char *> {
 protected:
  Scenario s;
  BT::BehaviorTreeFactory factory;
  void SetUp() override {
    for (const auto &plugin : {"nav2_recovery_node_bt_node",
                              "nav2_pipeline_sequence_bt_node",
                              "nav2_rate_controller_bt_node"}) {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }
    for (const auto &name : {"ComputePathToPose", "ComputePathThroughPoses"}) {
      factory.registerSimpleAction(name, [this](BT::TreeNode &node) {
        ++s.plan_calls;
        std::string value;
        if (node.getInput("goal", value) || node.getInput("goals", value))
          s.observed_goal = value;
        if (s.plan_blocked) return Status::FAILURE;
        node.setOutput("path", std::string("fresh_valid_path"));
        return Status::SUCCESS;
      }, {BT::InputPort<std::string>("goal"), BT::InputPort<std::string>("goals"),
          BT::OutputPort<std::string>("path"), BT::InputPort<std::string>("planner_id")});
    }
    factory.registerBuilder<Follow>("FollowPath", [this](
        const std::string &name, const BT::NodeConfiguration &cfg) {
      return std::make_unique<Follow>(name, cfg, s);
    });
  }
  BT::Tree tree(bool fast = false) {
    std::ifstream file(std::string(NAVIGATION_CONFIG_DIR) + "/" + GetParam());
    std::string xml((std::istreambuf_iterator<char>(file)), {});
    EXPECT_NE(xml.find("number_of_retries=\"30\""), std::string::npos);
    EXPECT_NE(xml.find("delay_msec=\"1000\""), std::string::npos);
    if (fast) {
      // Keep production structure and retry count; shorten time only for tests.
      xml.replace(xml.find("delay_msec=\"1000\""), 17, "delay_msec=\"20\"");
    }
    auto board = BT::Blackboard::create();
    board->set("goal", std::string("original_goal"));
    board->set("goals", std::string("original_goal"));
    return factory.createTreeFromText(xml, board);
  }
  Status tickUntilDone(BT::Tree &t, double seconds) {
    auto end = Clock::now() + std::chrono::duration<double>(seconds);
    Status result = Status::RUNNING;
    while (Clock::now() < end && result == Status::RUNNING) {
      result = t.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return result;
  }
};

TEST_P(NavigationRecovery, RetainsGoalAndWaitsBeforeRetryingPlanner) {
  auto t = tree();
  s.plan_blocked = true;
  EXPECT_EQ(t.tickRoot(), Status::RUNNING);
  EXPECT_EQ(s.plan_calls, 1);
  EXPECT_EQ(s.follow_calls, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(t.tickRoot(), Status::RUNNING);
  EXPECT_EQ(s.plan_calls, 1);  // No busy retry and no tracking of stale path.
  s.plan_blocked = false;
  s.arrive = true;
  EXPECT_EQ(tickUntilDone(t, 2.), Status::SUCCESS);
  EXPECT_EQ(s.plan_calls, 2);
  EXPECT_EQ(s.observed_goal, "original_goal");
}

TEST_P(NavigationRecovery, ControllerFailureTriggersFreshPlan) {
  auto t = tree(true);
  s.follow_blocked = true;
  EXPECT_EQ(t.tickRoot(), Status::RUNNING);
  s.follow_blocked = false;
  s.arrive = true;
  EXPECT_EQ(tickUntilDone(t, 1.), Status::SUCCESS);
  EXPECT_EQ(s.plan_calls, 2);
}

TEST_P(NavigationRecovery, ReplanFailureHaltsRunningController) {
  auto t = tree(true);
  EXPECT_EQ(t.tickRoot(), Status::RUNNING);
  s.plan_blocked = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  EXPECT_EQ(t.tickRoot(), Status::RUNNING);
  EXPECT_EQ(s.halts, 1);
  const auto calls = s.follow_calls;
  EXPECT_EQ(t.tickRoot(), Status::RUNNING);
  EXPECT_EQ(s.follow_calls, calls);
}

TEST_P(NavigationRecovery, PermanentFailureHasBoundedRetries) {
  auto t = tree(true);
  s.plan_blocked = true;
  EXPECT_EQ(tickUntilDone(t, 3.), Status::FAILURE);
  EXPECT_EQ(s.plan_calls, 31);  // Initial attempt plus 30 retries.
  EXPECT_EQ(s.follow_calls, 0);
}

TEST_P(NavigationRecovery, CancelDuringWaitDoesNotRestartWork) {
  auto t = tree(true);
  s.plan_blocked = true;
  EXPECT_EQ(t.tickRoot(), Status::RUNNING);
  t.haltTree();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(s.plan_calls, 1);
  EXPECT_EQ(s.follow_calls, 0);
  EXPECT_EQ(t.rootNode()->status(), Status::IDLE);
}

INSTANTIATE_TEST_SUITE_P(SingleAndMultipleGoals, NavigationRecovery,
                        ::testing::Values("navigate.xml", "navigate_many.xml"));
}  // namespace
