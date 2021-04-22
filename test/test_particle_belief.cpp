#include <gtest/gtest.h>

#include <limits>
#include <solver_ipft/core/pomdp_world.hpp>
#include <solver_ipft/interface/rollout.hpp>
#include <solver_ipft/interface/spaces.hpp>
#include <solver_ipft/util/util.hpp>

#include "test_particle_data.hpp"

namespace solver_ipft {
namespace test {

// The fixture for testing class ParticleBelief.
class ParticleBeliefTest : public ParticleBeliefData {
   protected:
    // You can remove any or all of the following functions if their bodies would
    // be empty.

    ParticleBeliefTest() {
        // You can do set-up work for each test here.
    }

    ~ParticleBeliefTest() override {
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

// Tests if particle set variance is calculated properly
TEST_F(ParticleBeliefTest, ParticleSetVariance) {
    State* wSampleStd = belief1->std();
    double wstd = wSampleStd->get(0);
    belief1->model_->freeState(wSampleStd);
    EXPECT_DOUBLE_EQ(sqrt(5.365756272242247), wstd) << "The calculated std is: " << wstd;
}

TEST_F(ParticleBeliefTest, ParticleSetVarianceZero) {
    State* wSampleStd = belief2->std();
    double wstd = wSampleStd->get(0);
    belief2->model_->freeState(wSampleStd);
    EXPECT_TRUE(wstd < Globals::DOUBLEEQLIM) << "The calculated std is: " << wstd;
}

// Tests if particle set mean is calculated properly
TEST_F(ParticleBeliefTest, ParticleSetMean) {
    State* wSampleMean = belief1->mean();
    double mean = wSampleMean->get(0);
    belief1->model_->freeState(wSampleMean);
    EXPECT_DOUBLE_EQ(2.1138270500000003, mean) << "The calculated mean is: " << mean;
}

// check if mean is moved
TEST_F(ParticleBeliefTest, StepUpdateMoveMean) {
    // initial position
    double initPos = 4.253895;
    LOG(INFO) << "initial pos: " << initPos;
    State* priorMean = belief2->mean();
    EXPECT_DOUBLE_EQ(initPos, priorMean->get(0));
    LOG(INFO) << "initial belief: " << *belief2;
    // action
    Action act = static_cast<Action>(cld::CLDAction::NEG3);
    double posteriorPos = initPos + cld::actToValueMap[act];
    LOG(INFO) << this->model_->to_string(act);
    LOG(INFO) << "new true state pos: " << posteriorPos;
    // generate observation in the near of true state
    double obsPos = 10;
    Observation* obs = this->model_->allocateObs();
    obs->set(obsPos, 0);
    LOG(INFO) << "observation: " << *obs;
    // belief update
    LOG(INFO) << "--------------belief update";
    belief2->update(act, *obs);
    LOG(INFO) << "posterior belief: ";
    LOG(INFO) << belief2->detailedText(belief2->weighted_posterior_particles_);
    LOG(INFO) << belief2->detailedText();
    State* posteriorMean = belief2->mean();
    State* posteriorStd = belief2->std();
    EXPECT_LE(std::abs(posteriorMean->get(0) - posteriorPos), 0.1);
    EXPECT_LE(posteriorStd->get(0), 0.2);
    this->model_->freeState(priorMean);
    this->model_->freeState(posteriorMean);
    this->model_->freeState(posteriorStd);
}

// check if mean is moved and variance is reduced
TEST_F(ParticleBeliefTest, StepUpdateFromUniform) {
    // ParticleBelief* belief = createUniformParticleSet(30, -10, 10);
    ParticleBelief* belief = createNormalParticleSet(30, 0, 10);
    State* priorMean = belief->mean();
    LOG(INFO) << belief->detailedText();
    // action
    Action act = static_cast<Action>(cld::CLDAction::POS3);
    LOG(INFO) << this->model_->to_string(act);
    // generate observation in the near of true state
    double obsPos = 10;
    Observation* obs = this->model_->allocateObs();
    obs->set(obsPos, 0);
    LOG(INFO) << "observation: " << *obs;
    // belief update
    LOG(INFO) << "--------------belief update";
    double reward = belief->update(act, *obs);
    LOG(INFO) << "reward: " << reward;
    LOG(INFO) << "posterior belief: ";
    LOG(INFO) << belief->detailedText(belief->weighted_posterior_particles_);
    LOG(INFO) << belief->detailedText();
    State* posteriorMean = belief->mean();
    State* posteriorStd = belief->std();
    // Expect posterior mean closer to obsPos than prior mean
    EXPECT_LE(std::abs(posteriorMean->get(0) - obsPos), std::abs(priorMean->get(0) - obsPos));
    // EXPECT_LE(posteriorStd[0], 0.2);
    this->model_->freeState(priorMean);
    this->model_->freeState(posteriorMean);
    this->model_->freeState(posteriorStd);
}

// check if pf can track world state in case of interaction with pomdp world object
TEST_F(ParticleBeliefTest, SequenceUpdateWorldStateTracked) {
    // scenario: solver belief initially uniformly distributed, world starts with state -5.0
    // initialisation
    this->world_ = new POMDPWorld(this->model_);
    // init pos
    double initPos = -5.0;
    // action sequence
    Action act3 = static_cast<Action>(cld::CLDAction::POS3);
    std::vector<Action> actionSeq = {act3, act3, act3, act3, act3};
    // Action act1 = static_cast<Action>(cld::CLDAction::POS1);
    // std::vector<Action> actionSeq = {act1, act1, act1, act1, act1};
    // std::vector<Action> actionSeq = {static_cast<Action>(cld::CLDAction::POS3),
    //                                  static_cast<Action>(cld::CLDAction::POS1),
    //                                  static_cast<Action>(cld::CLDAction::POS3),
    //                                  static_cast<Action>(cld::CLDAction::NEG3),
    //                                  static_cast<Action>(cld::CLDAction::NEG3)};
    DeterministicActionChooser actCh(actionSeq);
    // create initial state
    State* s = this->model_->allocateState();
    s->set(initPos, 0);
    // set world start state
    this->world_->setState(s);
    // create initial belief
    // ParticleBelief* b = createUniformParticleSet(10000, -20, 20);
    ParticleBelief* b = createNormalParticleSet(10000, 0, 10);
    Observation* o = nullptr;
    // print initial state and initial belief
    LOG(INFO) << "Initial state: " << *s;
    LOG(INFO) << "Initial belief: " << *b;
    // execution of action sequence 3,3,3,3,3 to get to final state 10.0
    while (actCh.numberOfActionsLeft() > 0) {
        State* s = this->world_->getCurrentState();
        Action a = actCh.chooseAction(b);
        this->world_->executeAction(a, o);
        State* sp = this->world_->getCurrentState();
        cld::CLDActionValue act(a);
        // belief update
        b->update(a, *o);
        std::cerr << "(s,a,sp,o,b): (" << *s << " | " << act << " | " << *sp << " | " << *o << " |" << b->text() << ")" << std::endl;
        this->model_->freeState(s);
        this->model_->freeState(sp);
        this->model_->freeObs(o);
    }
    // expect: solver belief tracks the world state mean around 10.0 with low variance
    s = this->world_->getCurrentState();
    State* mean = b->mean();
    double absdiff = std::abs(mean->get(0) - s->get(0));
    EXPECT_LE(absdiff, 5.0) << "World State: " << s->get(0) << ", Solver Mean: " << mean->get(0);
    this->model_->freeState(s);
    b->model_->freeState(mean);
    delete b;
}

// check if pf can track world state in case of interaction with pomdp world object
//* Use this action sequence as test case
// ---------------------------------------------------------------------------------------------------------------------------------
// Step |    State | Solv.Mean | Solv.Std |   Action | StatePost | Observation |  ObsProb | Solv.MeanPost | Solv.StdPost |    Reward
// ---------------------------------------------------------------------------------------------------------------------------------
//    0 | -14.2260 |   -0.3686 |   9.7104 |   3.0000 |  -11.2460 |     -3.9420 | 0.012692 |       -0.0278 |       7.1668 | -1.000000
//    1 | -11.2460 |   -0.0278 |   7.1668 |   3.0000 |   -8.1918 |      4.2255 | 0.013598 |        4.1763 |       4.7600 | -1.000000
//    2 |  -8.1918 |    4.1763 |   4.7600 |   3.0000 |   -5.3134 |     -9.4116 | 0.017700 |        1.6291 |       4.7972 | -1.000000
//    3 |  -5.3134 |    1.6291 |   4.7972 |   3.0000 |   -2.2995 |      5.7384 | 0.020155 |        5.6189 |       3.0171 | -1.000000
//    4 |  -2.2995 |    5.6189 |   3.0171 |   3.0000 |    0.7549 |      3.1500 | 0.028935 |        6.3821 |       2.3698 | -1.000000
//    5 |   0.7549 |    6.3821 |   2.3698 |  -3.0000 |   -2.3222 |      4.8546 | 0.020541 |        3.7990 |       1.8335 | -1.000000
//    6 |  -2.3222 |    3.7990 |   1.8335 |  -3.0000 |   -5.3013 |     -2.8043 | 0.017905 |        0.9629 |       1.5279 | -1.000000
//    7 |  -5.3013 |    0.9629 |   1.5279 |  -1.0000 |   -6.2878 |     17.4035 | 0.010213 |        0.4179 |       3.2666 | -1.000000
//    8 |  -6.2878 |    0.4179 |   3.2666 |   0.0000 |   -6.3095 |    -11.7572 | 0.016483 |        0.0875 |       1.8303 | -100.000000
// Result from julia code (same action and observation sequence)
// (--initial belief--)
// (mPc, stdPc) = (-0.01746798735116072, 7.1247737916577805)
// (mPc, stdPc) = (4.240455331184715, 4.657939864900925)
// (mPc, stdPc) = (1.800269403280704, 4.770788287546803)
// (mPc, stdPc) = (5.706291182001573, 2.9492997521162567)
// (mPc, stdPc) = (6.387122112016366, 2.3153165486802965)
// (mPc, stdPc) = (3.778966482660967, 1.7407823160292104)
// (mPc, stdPc) = (0.9573248344380069, 1.523539966673673)
// (mPc, stdPc) = (0.5513595257094986, 3.8393851418754026)
// (mPc, stdPc) = (0.0749427027459313, 1.9827545911466833) (<-- step 8 post)
// Total discounted reward = -73.07
TEST_F(ParticleBeliefTest, SequenceUpdateWorldStateTracked2) {
    // scenario: solver belief initially uniformly distributed, world starts with state -5.0
    // initialisation
    this->world_ = new POMDPWorld(this->model_);
    // number of particles
    int num_particles = 10000;
    // init pos
    double initPos = -14.226;
    // action sequence
    Action act3 = static_cast<Action>(cld::CLDAction::POS3);
    Action actN3 = static_cast<Action>(cld::CLDAction::NEG3);
    Action actN1 = static_cast<Action>(cld::CLDAction::NEG1);
    Action act0 = static_cast<Action>(cld::CLDAction::ZERO);
    std::vector<Action> actionSeq = {act3, act3, act3, act3, act3, actN3, actN3, actN1, act0};
    // Action act1 = static_cast<Action>(cld::CLDAction::POS1);
    // std::vector<Action> actionSeq = {act1, act1, act1, act1, act1};
    // std::vector<Action> actionSeq = {static_cast<Action>(cld::CLDAction::POS3),
    //                                  static_cast<Action>(cld::CLDAction::POS1),
    //                                  static_cast<Action>(cld::CLDAction::POS3),
    //                                  static_cast<Action>(cld::CLDAction::NEG3),
    //                                  static_cast<Action>(cld::CLDAction::NEG3)};
    DeterministicActionChooser actCh(actionSeq);
    // create initial state
    State* s = this->model_->allocateState();
    s->set(initPos, 0);
    // set world start state
    this->world_->setState(s);
    // create initial belief
    // ParticleBelief* b = createUniformParticleSet(10000, -20, 20);
    ParticleBelief* b = createNormalParticleSet(num_particles, 0, 10);
    Observation* o = nullptr;
    // print initial state and initial belief
    LOG(INFO) << "Initial state: " << *s;
    LOG(INFO) << "Initial belief: " << *b;
    while (actCh.numberOfActionsLeft() > 0) {
        State* s = this->world_->getCurrentState();
        Action a = actCh.chooseAction(b);
        this->world_->executeAction(a, o);
        State* sp = this->world_->getCurrentState();
        cld::CLDActionValue act(a);
        // belief update
        b->update(a, *o);
        std::cerr << "(s,a,sp,o,b): (" << *s << " | " << act << " | " << *sp << " | " << *o << " |" << b->text() << ")" << std::endl;
        this->model_->freeState(s);
        this->model_->freeState(sp);
        this->model_->freeObs(o);
    }
    // expect: solver belief tracks the world state mean around 10.0 with low variance
    s = this->world_->getCurrentState();
    State* mean = b->mean();
    double absdiff = std::abs(mean->get(0) - s->get(0));
    EXPECT_LE(absdiff, 3.0) << "World State: " << s->get(0) << ", Solver Mean: " << mean->get(0);
    this->model_->freeState(s);
    b->model_->freeState(mean);
    delete b;
}

TEST_F(ParticleBeliefTest, TestLowVarianceResamplerAllParticlesEqual) {
    std::vector<double> numbers = {1, 2, 3};

    std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
    ParticleBelief* b = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());
    LOG(INFO) << b->detailedText();
    LOG(INFO) << "-----------RESAMPLING";
    std::vector<State*> res = b->sample(3);
    LOG(INFO) << b->detailedText(res);
    // TODO add check particle sets equal
}

TEST_F(ParticleBeliefTest, TestLowVarianceResamplerWeightOnOneParticle) {
    std::vector<double> numbers = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<double> weights = {1, 1, 1, 1, 1, 50, 1, 1, 1, 100};
    std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
    for (int i = 0; i < states.size(); i++) {
        states[i]->weight_ = weights[i];
    }
    State::normalizeWeights(states);
    ParticleBelief* b = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());
    LOG(INFO) << b->detailedText();
    LOG(INFO) << "-----------RESAMPLING";
    std::vector<State*> res = b->sample(10);
    LOG(INFO) << b->detailedText(res);
    // TODO add check particle set contains only "heavy" particles
}

TEST_F(ParticleBeliefTest, WeightTest) {
    LOG(INFO) << "MINIMUM DOUBLE VAL: " << std::numeric_limits<double>::min();
}

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