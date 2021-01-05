#include "ipft/core/information_gain.hpp"

#include <cmath>
#include <glog/logging.h>

namespace ipft {
// IPFT paper equation (3)
// returns actually the negative entropy
double EntropyInfGain::computeEntropyEstimate(const ParticleBelief* b) const {
    double entropyValue = 0.0;
    double temp1;
    double temp2;
    double weight;
    for (int i = 0; i < b->numParticles(); i++) {
        const State* s = b->particle(i);
        temp1 = this->densEstimator->computeDensityValue(s, b);
        if (temp1 > 0.0) {
            temp2 = std::log(temp1);
        } else {
            LOG(WARNING) << "density value = " << temp1 << "<= 0";
            temp2 = -1e10; // avoid temp2 being -inf (since log(0) = -inf)
        }
        weight = s->weight_;
        entropyValue += s->weight_ * temp2;
    }
    CHECK(!std::isnan(entropyValue)) << "Entropy value is not a number!";
    return entropyValue;
}

double EntropyInfGain::computeDiscInfGain(double discFactor,
                                          const ParticleBelief* bnext,
                                          const ParticleBelief* b) const {
    return (discFactor * this->computeEntropyEstimate(bnext) - this->computeEntropyEstimate(b)); // no minus needed
}

} // namespace ipft