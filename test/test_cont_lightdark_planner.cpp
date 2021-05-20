// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include <gtest/gtest.h>

#include <solver_ipft/problems/cont_lightdark_planner.hpp>
#include <vector>

// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
// TEST(${pkgname}, TestName)
// {
//     EXPECT_TRUE(true);
//     // TODO:    Add your test code here
// }
// Infos:
// Remember, when they fail, ASSERT_* yields a fatal failure and returns from
// the current function, while EXPECT_* yields a nonfatal failure, allowing the
// function to continue running. However, when possible, ASSERT_EQ(actual,
// expected) is preferred to ASSERT_TRUE(actual == expected), since it tells you
// actual and expected's values on failure.

// testfixtures:
// The first thing to remember is that googletest does not reuse the same test
// fixture object across multiple tests. For each TEST_F, googletest will create
// a fresh test fixture object, immediately call SetUp(), run the test body,
// call TearDown(), and then delete the test fixture object. When you need to
// write per-test set-up and tear-down logic, you have the choice between using
// the test fixture constructor/destructor or SetUp()/TearDown(). The former is
// usually preferred, as it has the following benefits: Like TEST(), the first
// argument is the test suite name, but for TEST_F() this must be the name of
// the test fixture class. You've probably guessed: _F is for fixture.

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