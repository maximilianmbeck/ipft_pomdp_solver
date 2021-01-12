// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <interpolated_distance/interpolated_distance_function.hpp>
#include <vector>

namespace ipft {

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

// A google test function (uncomment the next function, add code and
// change the names TestGroupName and TestName)
TEST(InterpolatedDistance, SignedDistance) {
    using namespace interpolated_distance;

    std::vector<double> line{-2.0, 1.0, 0.0, 1.0, 2.0, 1.0};
    InterpolatedDistanceFunction interpolated_distance(line);

    // test the distance at a MIDDLE-segment
    auto d_middle = interpolated_distance.signedDistance(0.2, 2.0);
    ASSERT_DOUBLE_EQ(d_middle, 1.0);

    // test the distance at a line-to-line connection point
    auto d_line2line = interpolated_distance.signedDistance(0.0, 2.0);
    ASSERT_DOUBLE_EQ(d_line2line, 1.0);

    // test the distance at the FIRST-segment
    auto d_first = interpolated_distance.signedDistance(-4.0, -1.0);
    ASSERT_DOUBLE_EQ(d_first, -2.0 * pow(2, 0.5));

    // test the distance at the LAST-segment
    auto d_last = interpolated_distance.signedDistance(4.0, 3.0);
    ASSERT_DOUBLE_EQ(d_last, 2.0 * pow(2, 0.5));
}

TEST(InterpolatedDistance, Match) {
    using namespace interpolated_distance;

    std::vector<double> line{-2.0, 1.0, 0.0, 1.0, 2.0, 1.0};
    InterpolatedDistanceFunction interpolated_distance(line);

    auto match = interpolated_distance.match(0.0, 2.0);
    ASSERT_DOUBLE_EQ(std::get<0>(match), 2.0);  // arc_length
    ASSERT_DOUBLE_EQ(std::get<1>(match), 1.0);  // normal_distance
}

TEST(InterpolatedDistance, Reconstruct) {
    using namespace interpolated_distance;

    std::vector<double> line{-2.0, 1.0, 0.0, 1.0, 2.0, 1.0};
    InterpolatedDistanceFunction interpolated_distance(line);

    auto reconstruct = interpolated_distance.reconstruct(2.0, 1.0);
    ASSERT_DOUBLE_EQ(std::get<0>(reconstruct), 0.0);  // arc_length
    ASSERT_DOUBLE_EQ(std::get<1>(reconstruct), 2.0);  // normal_distance
}

TEST(InterpolatedDistance, Arclengths) {
    using namespace interpolated_distance;

    std::vector<double> line{-2.0, 0.0, 0.0, 1.5, 2.0, 3.0};
    InterpolatedDistanceFunction interpolated_distance(line);
    auto length = interpolated_distance.maxArclength();
    ASSERT_DOUBLE_EQ(length, 5.0);  // arc_length
}

}  // namespace ipft

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