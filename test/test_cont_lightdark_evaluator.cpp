#include <gtest/gtest.h>

#include <vector>
#include <solver_ipft/problems/cont_lightdark_planner.hpp>

namespace solver_ipft {
namespace test {

TEST(ProblemContLightDark, TestEvaluator) { // NOLINT
    Evaluator evaluator(std::make_unique<cld::MyPlanner>());
#ifdef NDEBUG
#else
    // set this parameter to avoid long test runtime, when compiled in debug mode
    // release build reproduces the benchmark results in IPFT paper.
    Globals::config.sim_len = 2;
    Globals::config.eval_len = 1;
#endif
    LOG(WARNING) << "Evaluation length: " << Globals::config.eval_len;
    evaluator.runEvaluation(0, nullptr);
    EXPECT_TRUE(true);
}
} // namespace test
} // namespace solver_ipft

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // INIT GLOG
    google::InitGoogleLogging(argv[0]); // NOLINT
    google::InstallFailureSignalHandler();
    // ::testing::InitGoogleTest(&argc, argv);

    FLAGS_colorlogtostderr = true;
    FLAGS_logtostderr = true;
    return RUN_ALL_TESTS();
}