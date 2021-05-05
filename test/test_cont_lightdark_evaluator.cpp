#include <gtest/gtest.h>

#include <solver_ipft/problems/cont_lightdark_planner.hpp>
#include <vector>

namespace solver_ipft {
namespace test {

TEST(ProblemContLightDark, TestEvaluator) { // NOLINT
  Evaluator evaluator(std::make_unique<cld::MyPlanner>());
  evaluator.runEvaluation(0, nullptr);
  EXPECT_TRUE(true);
}

} // namespace test
} // namespace solver_ipft

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // INIT GLOG
  google::InitGoogleLogging(argv[0]); // NOLINT
  google::InstallFailureSignalHandler();
  // ::testing::InitGoogleTest(&argc, argv);

  FLAGS_colorlogtostderr = true;
  FLAGS_logtostderr = true;
  return RUN_ALL_TESTS();
}