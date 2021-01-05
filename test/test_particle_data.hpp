#pragma once
#include <gtest/gtest.h>

#include <ipft/util/debug.hpp>

#include "test_ipft_objects.hpp"

namespace ipft {
namespace test {

// The fixture for testing with particle beliefs / particle sets
class ParticleBeliefData : public IpftObjects {
   protected:
    // You can remove any or all of the following functions if their bodies would
    // be empty.

    ParticleBelief* belief1;
    ParticleBelief* belief2;
    ParticleBelief* allEq5;
    ParticleBelief* belErr1Step;

    ParticleBeliefData() {
        // init model
        this->model_ = new cld::ContLightDark(this->rand_);
        // init belief1
        // belief1 variance =   5.365756272242247
        // belief1 std =        2.316410212428327
        // belief1 mean =       2.1138270500000003
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

        std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
        belief1 = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());

        // init belief2
        // belief2 variance =   0.0
        // belief2 std =        0.0
        // belief2 mean =       4.253895
        numbers = {4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895,
                   4.253895};

        states = debug::doubleVec2StateVec(numbers, this->model_);
        belief2 = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());

        // init belief allEq5
        numbers = {5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0,
                   5.0};
        states = debug::doubleVec2StateVec(numbers, this->model_);
        allEq5 = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());

        // belErr1Step
        numbers = {9.3953,
                   9.8427,
                   10.1268,
                   9.6946,
                   9.2815,
                   10.4726,
                   10.7598,
                   3.0242,
                   10.6167,
                   10.2247,
                   9.0881,
                   11.5099,
                   9.6757,
                   9.6167,
                   8.9347,
                   10.1384,
                   9.6376,
                   9.6736,
                   10.7702,
                   9.7005};

        states = debug::doubleVec2StateVec(numbers, this->model_);
        belErr1Step = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());
    }

    virtual ~ParticleBeliefData() override {
        // You can do clean-up work that doesn't throw exceptions here.
        delete belief1;
        delete belief2;
        delete allEq5;
        delete belErr1Step;
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

    ParticleBelief* createUniformParticleSet(int numParticles, double lower, double upper) {
        std::vector<State*> particleSet;
        for (int i = 0; i < numParticles; i++) {
            State* particle = this->model_->allocateState();
            double pos = rand_->nextUniform(lower, upper);
            particle->set(pos, 0);  // dimension 0 (CLDStates have only 1 dimension)
            particleSet.push_back(particle);
        }
        State::normalizeWeights(particleSet, State::weightSum(particleSet));
        return new ParticleBelief(particleSet, false, this->model_, this->rand_, new ObsAdaptiveReinvigorator(this->model_, this->rand_));
    }

    ParticleBelief* createNormalParticleSet(int numParticles, double mu, double sigma) {
        std::vector<State*> particleSet;
        for (int i = 0; i < numParticles; i++) {
            State* particle = this->model_->allocateState();
            double pos = rand_->nextNormal(mu, sigma);
            particle->set(pos, 0);  // dimension 0 (CLDStates have only 1 dimension)
            particleSet.push_back(particle);
        }
        State::normalizeWeights(particleSet, State::weightSum(particleSet));
        return new ParticleBelief(particleSet, false, this->model_, this->rand_, new ObsAdaptiveReinvigorator(this->model_, this->rand_));
    }
};
}  // namespace test
}  // namespace ipft