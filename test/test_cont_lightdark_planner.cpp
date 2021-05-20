// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include <gtest/gtest.h>

#include <vector>
#include <solver_ipft/problems/cont_lightdark_planner.hpp>

namespace solver_ipft {
namespace test {

TEST(ProblemContLightDark, TestPlanner) { // NOLINT
    cld::MyPlanner planner;
#ifdef NDEBUG
#else
    Globals::config.sim_len = 2; // set this parameter to avoid long test runtime,
                                 // when compiled in debug mode
#endif
    LOG(WARNING) << "Simulation length:" << Globals::config.sim_len;
    planner.runPlanning(0, nullptr);
    EXPECT_TRUE(true);
}

} // namespace test
} // namespace solver_ipft

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    google::InitGoogleLogging(argv[0]); // NOLINT
    google::InstallFailureSignalHandler();
    // ::testing::InitGoogleTest(&argc, argv);

    FLAGS_colorlogtostderr = true;
    FLAGS_logtostderr = true;
    return RUN_ALL_TESTS();
}