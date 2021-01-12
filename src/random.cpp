#include "ipft/util/random.hpp"

#include <cassert>
#include <chrono>

namespace ipft {

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

double Random::nextNormal(const double& mu, const double& sigma) const {
    std::normal_distribution<> nd(mu, sigma);
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

}  // namespace ipft