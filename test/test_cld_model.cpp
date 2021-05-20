#include <gtest/gtest.h>

#include <solver_ipft/interface/spaces.hpp>
#include <solver_ipft/util/util.hpp>

#include "test_ipft_objects.hpp"

namespace solver_ipft {
namespace test {

// The fixture for testing class CldModel.
class CldModelTest : public IpftObjects {
protected:
    const int N_s = 100000; // number of samples drawn from distribution
    double deltaTransitionModel = 1e-3;
    double deltaObservationModel = 5e-3;

    CldModelTest() {
        this->model_ = std::make_shared<cld::ContLightDark>(this->rand_);
    }
};

TEST_F(CldModelTest, TestRewardModel) { // NOLINT
    auto act = static_cast<Action>(cld::CLDAction::ZERO);
    State* s = this->model_->allocateState();
    double reward;
    s->set(0.5, 0);
    reward = this->model_->reward(*s, act);
    EXPECT_DOUBLE_EQ(100.0, reward);
    s->set(1.0, 0);
    reward = this->model_->reward(*s, act);
    EXPECT_DOUBLE_EQ(100.0, reward);
    s->set(1.5, 0);
    reward = this->model_->reward(*s, act);
    EXPECT_DOUBLE_EQ(50.0, reward);
    s->set(2.0, 0);
    reward = this->model_->reward(*s, act);
    EXPECT_DOUBLE_EQ(-100.0, reward);
    s->set(-0.5, 0);
    reward = this->model_->reward(*s, act);
    EXPECT_DOUBLE_EQ(100.0, reward);
    s->set(-1.0, 0);
    reward = this->model_->reward(*s, act);
    EXPECT_DOUBLE_EQ(100.0, reward);
    s->set(-1.5, 0);
    reward = this->model_->reward(*s, act);
    EXPECT_DOUBLE_EQ(50.0, reward);
    s->set(-2.0, 0);
    reward = this->model_->reward(*s, act);
    EXPECT_DOUBLE_EQ(-100.0, reward);

    double start = -10.0;
    double delta = 0.1;
    State* s2 = this->model_->allocateState();
    for (int i = 0; i < 80; i++) {
        double statePos = start + i * delta;
        s2->set(statePos, 0);
        double reward = this->model_->reward(*s2, act);
        EXPECT_DOUBLE_EQ(-100.0, reward);
    }

    start = -2.0;
    for (int i = 0; i < 5; i++) {
        double statePos = start + i * delta;
        s2->set(statePos, 0);
        double reward = this->model_->reward(*s2, act);
        EXPECT_DOUBLE_EQ(500 - 300 * std::abs(statePos), reward);
    }

    start = -1.5;
    for (int i = 0; i < 5; i++) {
        double statePos = start + i * delta;
        s2->set(statePos, 0);
        double reward = this->model_->reward(*s2, act);
        EXPECT_DOUBLE_EQ(200 - 100 * std::abs(statePos), reward);
    }

    start = -1.0;
    for (int i = 0; i < 20; i++) {
        double statePos = start + i * delta;
        s2->set(statePos, 0);
        double reward = this->model_->reward(*s2, act);
        EXPECT_DOUBLE_EQ(100.0, reward);
    }
    start = 1.0;
    for (int i = 0; i < 5; i++) {
        double statePos = start + i * delta;
        s2->set(statePos, 0);
        double reward = this->model_->reward(*s2, act);
        EXPECT_DOUBLE_EQ(200 - 100 * std::abs(statePos), reward);
    }

    start = 1.5;
    for (int i = 0; i < 5; i++) {
        double statePos = start + i * delta;
        s2->set(statePos, 0);
        double reward = this->model_->reward(*s2, act);
        EXPECT_DOUBLE_EQ(500 - 300 * std::abs(statePos), reward);
    }

    start = 2.0;
    for (int i = 0; i < 80; i++) {
        double statePos = start + i * delta;
        s2->set(statePos, 0);
        double reward = this->model_->reward(*s2, act);
        EXPECT_DOUBLE_EQ(-100.0, reward);
    }
    this->model_->freeState(s2);
}

TEST_F(CldModelTest, TestObsModelSigmaAnyStatePos) { // NOLINT
    double statePos = -8.942687;
    double obsSigma = dynamic_cast<cld::ContLightDark*>(this->model_.get())->obsModelSigma(statePos);
    EXPECT_DOUBLE_EQ(27.289004863188516, obsSigma); // value from julia code
}

TEST_F(CldModelTest, TestObsModelSigmaPosLightLoc) { // NOLINT
    double statePos = cld::lightSourceLoc;
    double obsSigma = dynamic_cast<cld::ContLightDark*>(this->model_.get())->obsModelSigma(statePos);
    EXPECT_DOUBLE_EQ(0.5, obsSigma); // value from julia code
}

// draw multiple random samples from obs model and check if moments of sample
// set match moments of distribution
TEST_F(CldModelTest, TestObsModel) { // NOLINT
    State* s = this->model_->allocateState();
    double statePos = cld::lightSourceLoc;
    double obsSigma = dynamic_cast<cld::ContLightDark*>(this->model_.get())->obsModelSigma(statePos);
    s->set(statePos, 0);
    std::vector<Observation*> obsVec;
    for (int i = 0; i < this->N_s; i++) {
        Observation* obs = this->model_->observation(*s);
        // allocate state
        obsVec.push_back(obs);
    }

    std::vector<double> obsMean = Point::mean(obsVec);
    std::vector<double> obsVar = Point::variance(obsVec);
    std::vector<double> obsStd = util::weightedSampleVarianceToStd(obsVar);
    double estimatedMu = obsMean[0];
    double estimatedStd = obsStd[0];
    double diffMean = std::abs(statePos - estimatedMu);
    double diffStd = std::abs(obsSigma - estimatedStd);
    EXPECT_LE(diffMean, deltaObservationModel);
    EXPECT_LE(diffStd, deltaObservationModel);
}

TEST_F(CldModelTest, TestObsProb) { // NOLINT
    State* s = this->model_->allocateState();
    Observation* obs = this->model_->allocateObs();
    s->set(-8.942687, 0);
    obs->set(-12.329459, 0);
    double obsProb = this->model_->obsProb(*s, *obs);

    EXPECT_DOUBLE_EQ(0.014507003500894332, obsProb); // value from julia code
}

// draw multiple random samples from transition model and check if moments of
// sample set match moments of distribution
TEST_F(CldModelTest, TestTransitionModel) { // NOLINT
    State* s = this->model_->allocateState();
    double initpos = 0.0;
    s->set(initpos, 0);
    auto act = static_cast<Action>(cld::CLDAction::POS3);
    double resultpos = initpos + cld::actToValueMap[act];
    std::vector<State*> statesPost;
    for (int i = 0; i < this->N_s; i++) {
        State* sp = this->model_->transition(*s, act);
        statesPost.push_back(sp);
    }
    State::normalizeWeights(statesPost);
    auto b = std::make_unique<ParticleBelief>(statesPost, false, this->model_, this->rand_);

    State* statesPostMean = b->mean();
    State* statesPostStd = b->std();
    double estimatedMu = statesPostMean->get(0);
    double estimatedStd = statesPostStd->get(0);
    this->model_->freeState(statesPostMean);
    this->model_->freeState(statesPostStd);

    double diffMean = std::abs(resultpos - estimatedMu);
    double diffStd = std::abs(cld::sigmaTransition - estimatedStd);
    EXPECT_LE(diffMean, deltaTransitionModel);
    EXPECT_LE(diffStd, deltaTransitionModel);
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