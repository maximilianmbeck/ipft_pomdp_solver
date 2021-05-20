#include <gtest/gtest.h>

#include <solver_ipft/core/information_gain.hpp>

#include "test_particle_data.hpp"

namespace solver_ipft {
namespace test {

// The fixture for testing class ParticleBelief.
class InformationGainTest : public ParticleBeliefData {
protected:
    std::unique_ptr<ParticleBelief> belErr1StepNext;

    InformationGainTest() {
        std::vector<double> numbers = {3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643,
                                       3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643, 3.0643};

        std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
        belErr1StepNext = std::make_unique<ParticleBelief>(states, false, this->model_, this->rand_);
    }
};

TEST_F(InformationGainTest, EntropyTest) { // NOLINT
    EntropyInfGain ifm;
    double belief1entropy = ifm.computeEntropyEstimate(belief1.get());

    EXPECT_DOUBLE_EQ(-2.162244658288974,
                     belief1entropy); // value from julia function information()
}

TEST_F(InformationGainTest, kdeSilvermansRuleAllPartEqual) { // NOLINT
    KernelDensityEstimator kde;
    RotBandwidthSelector rotbws;
    double rotBandwidth = rotbws.computeBandwidth(belief2.get());
    LOG(INFO) << "Silvermans Bandwidth all particles equal: " << rotBandwidth;
    double kdeAllPEqual = kde.computeDensityValue(belief2->particle(0), belief2.get());
    LOG(INFO) << "KDE at a state pos in particle set all particles equal : " << kdeAllPEqual;
}

TEST_F(InformationGainTest, EntropyTestAllParticlesEqual) { // NOLINT
    EntropyInfGain ifm;
    double belief2entropy = ifm.computeEntropyEstimate(belief2.get());
    LOG(INFO) << "Entropy all particles equal: " << belief2entropy;
    EXPECT_DOUBLE_EQ(17.102888161353903,
                     belief2entropy); // value from julia function information()
}

TEST_F(InformationGainTest, ComputeDiscInfGainErr1Step) { // NOLINT
    EntropyInfGain ifm;
    double entrBelErr1Step = ifm.computeEntropyEstimate(belErr1Step.get());
    double entrBelErr1StepNext = ifm.computeEntropyEstimate(belErr1StepNext.get());
    double infrew = ifm.computeDiscInfGain(1.0, belErr1StepNext.get(), belErr1Step.get());
    LOG(INFO) << "entr val belErr1Step: " << entrBelErr1Step;
    LOG(INFO) << "entr val all particles equal: " << entrBelErr1StepNext;
    LOG(INFO) << "inf reward: " << infrew * Globals::config.inf_gather_constant_lambda;
    EXPECT_DOUBLE_EQ(18.489005591100099, infrew);
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