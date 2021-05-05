#include <gtest/gtest.h>

#include <solver_ipft/interface/spaces.hpp>
#include <solver_ipft/problems/cont_lightdark.hpp>
#include <solver_ipft/util/debug.hpp>
#include <vector>
namespace solver_ipft {

namespace test {

TEST(TestDebugFunctions, doubleVec2StateVec) { // NOLINT
  std::vector<double> numbers = {
      4.253895, 3.168835, 3.300457,  1.848377, -3.348563, -0.461945, 2.089914,
      5.221298, 0.026519, 4.701948,  1.938396, 3.295580,  -1.122583, 3.541429,
      3.684907, 1.776786, -2.282374, 4.339850, 3.168835,  3.134980};

  std::vector<State *> states =
      debug::doubleVec2StateVec<cld::CLDState>(numbers);

  for (int i = 0; i < states.size(); i++) {
    EXPECT_DOUBLE_EQ(numbers[i], states[i]->get(0));
  }
  EXPECT_DOUBLE_EQ(1.0, State::weightSum(states));
  debug::freeStateVec(states);
}

TEST(TestPoint, TestPointMean) { // NOLINT
  std::vector<double> numbers = {
      4.253895, 3.168835, 3.300457,  1.848377, -3.348563, -0.461945, 2.089914,
      5.221298, 0.026519, 4.701948,  1.938396, 3.295580,  -1.122583, 3.541429,
      3.684907, 1.776786, -2.282374, 4.339850, 3.168835,  3.134980};

  std::vector<Point *> points = debug::doubleVec2PointVec<cld::CLDObs>(numbers);

  std::vector<double> meanVec = Point::mean(points);
  std::vector<double> varVec = Point::variance(points);
  EXPECT_DOUBLE_EQ(2.11382705, meanVec[0]);
  EXPECT_DOUBLE_EQ(5.365756272242247, varVec[0]);

  debug::freePointVec(points);
}

TEST(TestPoint, TestDoubleEpsilon) { // NOLINT
  double eps = std::numeric_limits<double>::epsilon();
  LOG(INFO) << "eps: " << eps;
  double sqrteps = sqrt(std::numeric_limits<double>::epsilon());
  LOG(INFO) << "sqrteps: " << sqrteps;
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