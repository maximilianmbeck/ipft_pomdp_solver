#include "solver_ipft/core/information_gain.hpp"

#include <glog/logging.h>

#include <cmath>

namespace solver_ipft {

/* -------------------------- EntropyInfGain class -------------------------- */

// IPFT paper equation (3)
// returns actually the negative entropy
double EntropyInfGain::computeEntropyEstimate(const Belief *b) const {
  auto pb = dynamic_cast<const ParticleBelief *>(b);
  double entropyValue = 0.0;
  double temp1;
  double temp2;
  double weight;
  for (int i = 0; i < pb->numParticles(); i++) {
    const State *s = pb->particle(i);
    temp1 = this->densEstimator->computeDensityValue(s, pb);
    if (temp1 > 0.0) {
      temp2 = std::log(temp1);
    } else {
      LOG(WARNING) << "density value = " << temp1 << "<= 0";
      temp2 = -1e10; // avoid temp2 being -inf (since log(0) = -inf)
    }
    entropyValue += s->weight_ * temp2;
  }
  CHECK(!std::isnan(entropyValue)) << "Entropy value is not a number!";
  return entropyValue;
}

double EntropyInfGain::computeDiscInfGain(double discFactor,
                                          const Belief *bnext,
                                          const Belief *b) const {
  return (discFactor * this->computeEntropyEstimate(bnext) -
          this->computeEntropyEstimate(b)); // no minus needed
}

/* ----------------------------- NoInfGain class ---------------------------- */

double NoInfGain::computeDiscInfGain(double discFactor, const Belief *bnext,
                                     const Belief *b) const {
  return 0.0;
}
} // namespace solver_ipft