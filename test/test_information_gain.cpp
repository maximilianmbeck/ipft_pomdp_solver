#include <gtest/gtest.h>

#include <solver_ipft/core/information_gain.hpp>

#include "test_particle_data.hpp"

namespace solver_ipft {
namespace test {

// The fixture for testing class ParticleBelief.
class InformationGainTest : public ParticleBeliefData {
   protected:
    // You can remove any or all of the following functions if their bodies would
    // be empty.

    ParticleBelief *belErr1StepNext;

    InformationGainTest() {
        // You can do set-up work for each test here.
        std::vector<double> numbers = {3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643,
                                       3.0643};

        std::vector<State *> states = debug::doubleVec2StateVec(numbers, this->model_);
        belErr1StepNext = new ParticleBelief(states, false, this->model_, this->rand_);
    }

    ~InformationGainTest() override {
        // You can do clean-up work that doesn't throw exceptions here.
        delete belErr1StepNext;
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

TEST_F(InformationGainTest, EntropyTest) {
    EntropyInfGain ifm;
    double belief1entropy = ifm.computeEntropyEstimate(belief1);

    EXPECT_DOUBLE_EQ(-2.162244658288974, belief1entropy);  // value from julia function information()
}

TEST_F(InformationGainTest, kdeSilvermansRuleAllPartEqual) {
    KernelDensityEstimator kde;
    RotBandwidthSelector rotbws;
    double rotBandwidth = rotbws.computeBandwidth(belief2);
    LOG(INFO) << "Silvermans Bandwidth all particles equal: " << rotBandwidth;
    double kdeAllPEqual = kde.computeDensityValue(belief2->particle(0), belief2);
    LOG(INFO) << "KDE at a state pos in particle set all particles equal: " << kdeAllPEqual;
}

TEST_F(InformationGainTest, EntropyTestAllParticlesEqual) {
    EntropyInfGain ifm;
    double belief2entropy = ifm.computeEntropyEstimate(belief2);
    LOG(INFO) << "Entropy all particles equal: " << belief2entropy;
    EXPECT_DOUBLE_EQ(17.102888161353903, belief2entropy);  // value from julia function information()
}

TEST_F(InformationGainTest, ComputeDiscInfGainErr1Step) {
    EntropyInfGain ifm;
    double entrBelErr1Step = ifm.computeEntropyEstimate(belErr1Step);
    double entrBelErr1StepNext = ifm.computeEntropyEstimate(belErr1StepNext);
    double infrew = ifm.computeDiscInfGain(1.0, belErr1StepNext, belErr1Step);
    LOG(INFO) << "entr val belErr1Step: " << entrBelErr1Step;
    LOG(INFO) << "entr val all particles equal: " << entrBelErr1StepNext;
    LOG(INFO) << "inf reward: " << infrew * Globals::config.inf_gather_constant_lambda;
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