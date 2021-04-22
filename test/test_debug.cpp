// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include <gtest/gtest.h>

#include <solver_ipft/interface/spaces.hpp>
#include <solver_ipft/problems/cont_lightdark.hpp>
#include <solver_ipft/util/debug.hpp>
#include <vector>

// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
// TEST(${pkgname}, TestName)
// {
//     EXPECT_TRUE(true);
//     // TODO:    Add your test code here
// }
// Infos:
// Remember, when they fail, ASSERT_* yields a fatal failure and returns from the current function, while EXPECT_* yields a nonfatal failure, allowing the function to continue running.
// However, when possible, ASSERT_EQ(actual, expected) is preferred to ASSERT_TRUE(actual == expected), since it tells you actual and expected's values on failure.

// testfixtures:
// The first thing to remember is that googletest does not reuse the same test fixture object across multiple tests. For each TEST_F, googletest will create a fresh test fixture object, immediately call SetUp(), run the test body, call TearDown(), and then delete the test fixture object.
// When you need to write per-test set-up and tear-down logic, you have the choice between using the test fixture constructor/destructor or SetUp()/TearDown(). The former is usually preferred, as it has the following benefits:
// Like TEST(), the first argument is the test suite name, but for TEST_F() this must be the name of the test fixture class. You've probably guessed: _F is for fixture.

namespace solver_ipft {

namespace test {

TEST(TestDebugFunctions, doubleVec2StateVec) {
    std::vector<double> numbers = {4.253895,
                                   3.168835,
                                   3.300457,
                                   1.848377,
                                   -3.348563,
                                   -0.461945,
                                   2.089914,
                                   5.221298,
                                   0.026519,
                                   4.701948,
                                   1.938396,
                                   3.295580,
                                   -1.122583,
                                   3.541429,
                                   3.684907,
                                   1.776786,
                                   -2.282374,
                                   4.339850,
                                   3.168835,
                                   3.134980};

    std::vector<State*> states = debug::doubleVec2StateVec<cld::CLDState>(numbers);

    for (int i = 0; i < states.size(); i++) {
        EXPECT_DOUBLE_EQ(numbers[i], states[i]->get(0));
    }
    EXPECT_DOUBLE_EQ(1.0, State::weightSum(states));
    debug::freeStateVec(states);
}

TEST(TestPoint, TestPointMean) {
    std::vector<double> numbers = {4.253895,
                                   3.168835,
                                   3.300457,
                                   1.848377,
                                   -3.348563,
                                   -0.461945,
                                   2.089914,
                                   5.221298,
                                   0.026519,
                                   4.701948,
                                   1.938396,
                                   3.295580,
                                   -1.122583,
                                   3.541429,
                                   3.684907,
                                   1.776786,
                                   -2.282374,
                                   4.339850,
                                   3.168835,
                                   3.134980};

    std::vector<Point*> points = debug::doubleVec2PointVec<cld::CLDObs>(numbers);

    std::vector<double> meanVec = Point::mean(points);
    std::vector<double> varVec = Point::variance(points);
    EXPECT_DOUBLE_EQ(2.11382705, meanVec[0]);
    EXPECT_DOUBLE_EQ(5.365756272242247, varVec[0]);

    debug::freePointVec(points);
}

TEST(TestPoint, TestDoubleEpsilon) {
    double eps = std::numeric_limits<double>::epsilon();
    LOG(INFO) << "eps: " << eps;
    double sqrteps = sqrt(std::numeric_limits<double>::epsilon());
    LOG(INFO) << "sqrteps: " << sqrteps;
}

}  // namespace test

}  // namespace solver_ipft

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // INIT GLOG
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    // ::testing::InitGoogleTest(&argc, argv);

    FLAGS_colorlogtostderr = true;
    FLAGS_logtostderr = true;
    return RUN_ALL_TESTS();
}