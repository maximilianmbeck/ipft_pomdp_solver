#include "solver_ipft/util/random.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <cassert>
#include <chrono>
namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                               Implementations                              */
/* -------------------------------------------------------------------------- */

Random* Random::_rand = 0;

Random* Random::gen() {
    if (_rand == 0) {
        _rand = new Random();
    }
    return _rand;
}

Random::Random() : Random(0) {}

Random::Random(int seedOffset) {
    // std::random_device r;
    // std::seed_seq seed{r(), r(), r(), r(), r(), r(), r(), r()};
    // _randEngine = new std::mt19937_64(seed);
    _randEngine = new std::mt19937_64(
        static_cast<std::uint32_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count()) + seedOffset);
}

Random::~Random() {
    delete _randEngine;
}

std::mt19937_64* Random::engine() const {
    return this->_randEngine;
}

double Random::nextNormal(const double& mean, const double& stddev) const {
    std::normal_distribution<> nd(mean, stddev);
    return nd(*(this->_randEngine));
}

double Random::nextUniform(const double& min, const double& max) const {
    std::uniform_real_distribution<> ud(min, max);
    return ud(*(this->_randEngine));
}

unsigned Random::nextUnsigned() const {
    return (*_randEngine)();
}

int Random::nextUniformInt(int min, int max) const {
    std::uniform_int_distribution<int> uid(min, max);
    return uid(*(this->_randEngine));
}

int Random::nextUniformInt(int n) const {
    assert(n > 0);
    return nextUniformInt(0, n - 1);
}

int Random::nextDiscrProbDistrInt(const std::vector<double>& discreteProbDistrWeights) const {
    // check discrete probability distribution
    // double totalProb = 0.0;
    // for (int i = 0; i < discreteProbDistrWeights.size(); i++) {
    //     totalProb += discreteProbDistrWeights[i];
    // }
    // CHECK_DOUBLE_EQ(totalProb, 1.0);
    // CHECK_GT(totalProb, 0) << ": Invalid probability distribution!";

    // // generate accumulated probs
    // std::vector<double> accProbDistr(discreteProbDistr.size());
    // int index = 0;
    // double accProb = 0.0;
    // std::generate(accProbDistr.begin(), accProbDistr.end(), [&discreteProbDistr, &index, &accProb]() -> double {
    //     accProb += discreteProbDistr[index];
    //     index++;
    //     return accProb;
    // });
    // // sample rand num uniformly from [0;1]
    // double sample = this->nextUniform(0.0, 1.0);
    // // find route index (the interval in which the sampled number falls)
    // for (int i = 0; i < accProbDistr.size(); i++) {
    //     if (sample < accProbDistr[i]) {
    //         return i;
    //     }
    // }
    // LOG(FATAL) << "Random number generation failed!";

    // use c++ stdlib
    std::discrete_distribution<> dd(discreteProbDistrWeights.begin(), discreteProbDistrWeights.end());
    return dd(*(this->_randEngine));
}

}  // namespace solver_ipft