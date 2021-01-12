#include <gtest/gtest.h>

#include <ipft/core/density_estimation.hpp>

#include "test_particle_data.hpp"

namespace solver_ipft {
namespace test {
// The fixture for testing density estimation classes.
class DensityEstimationTest : public ParticleBeliefData {
   protected:
    // You can remove any or all of the following functions if their bodies would
    // be empty.

    DensityEstimationTest() {
    }

    virtual ~DensityEstimationTest() override {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    // Class members declared here can be used by all tests in the test suite
};

/* ---------------------------- Univariate tests ---------------------------- */

// Tests if the univariate RoT-Bandwidth selector works properly
TEST_F(DensityEstimationTest, RotBandwidthUnivariate) {
    RotBandwidthSelector bws;
    double bw = bws.computeBandwidth(belief1);
    EXPECT_DOUBLE_EQ(1.347712384201964, bw);  // value from julia code bandwidth() function
}

// Tests if the univariate RoT-Bandwidth selector works properly
TEST_F(DensityEstimationTest, RotBandwidthUnivariateAlmostZero) {
    RotBandwidthSelector bws;
    double bw = bws.computeBandwidth(belief2);
    EXPECT_DOUBLE_EQ(std::sqrt(std::numeric_limits<double>::epsilon()), bw);  // same principle as julia code
}

// Tests if the univariate NormalKernel works properly
TEST_F(DensityEstimationTest, NormalKernelUnivariate) {
    RotBandwidthSelector bws;
    NormalKernel nk;
    double bw = bws.computeBandwidth(belief1);
    const State* s = belief1->particle(0);
    double val0 = nk.univariateValue(s, belief1, bw);
    EXPECT_DOUBLE_EQ(0.1496858882334352, val0);  // value from julia code
    // value from python scipy.stats.gaussian_kde = 0.14806727
    s = belief1->particle(1);
    double val1 = nk.univariateValue(s, belief1, bw);
    EXPECT_DOUBLE_EQ(0.17491154726955283, val1);  // value from julia code
    // value from python scipy.stats.gaussian_kde = 0.1725006
}

// // Tests if the univariate Kernel Density Estimation works properly
// TEST_F(DensityEstimationTest, KDEUnivariate) {
//     EXPECT_EQ(0, 0);
// }

// /* --------------------------- Multivariate tests --------------------------- */
// //! currently not supported, since no multivariate model implemented

// // Tests if the multivariate RoT-Bandwidth selector works properly
// TEST_F(DensityEstimationTest, RotBandwidthMultivariate) {
//     EXPECT_EQ(0, 0);
// }

// // Tests if the multivariate NormalKernel works properly
// TEST_F(DensityEstimationTest, NormalKernelMultivariate) {
//     EXPECT_EQ(0, 0);
// }

// // Tests if the multivariate Kernel Density Estimation works properly
// TEST_F(DensityEstimationTest, KDEMultivariate) {
//     EXPECT_EQ(0, 0);
// }
}  // namespace test
}  // namespace solver_ipft

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // INIT GLOG
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    // ::testing::InitGoogleTest(&argc, argv);

    FLAGS_colorlogtostderr = true;
    FLAGS_logtostderr = true;
    return RUN_ALL_TESTS();
}