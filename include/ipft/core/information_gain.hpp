#pragma once

#include "ipft/core/density_estimation.hpp"
#include "ipft/core/particle_belief.hpp"

namespace solver_ipft {

/* -------------------------------- Interface ------------------------------- */

class DiscountedInformationGain {
   public:
    DiscountedInformationGain(){};
    virtual ~DiscountedInformationGain(){};

    virtual double computeDiscInfGain(double discFactor,
                                      const ParticleBelief* bnext,
                                      const ParticleBelief* b) const = 0;
};

/* ------------------- Classes implementing the interface ------------------- */

class EntropyInfGain : public DiscountedInformationGain {
   private:
    DensityEstimator* densEstimator;

   public:
    EntropyInfGain() {
        densEstimator = new KernelDensityEstimator();  // default density estimator
    };
    virtual ~EntropyInfGain() {
        delete densEstimator;
    };

    double computeDiscInfGain(double discFactor, const ParticleBelief* bnext, const ParticleBelief* b) const override;

    // protected:
    double computeEntropyEstimate(const ParticleBelief* b) const;
};

class NoInfGain : public DiscountedInformationGain {
   public:
    NoInfGain() {}
    virtual ~NoInfGain() {}

    double computeDiscInfGain(double discFactor,
                              const ParticleBelief* bnext,
                              const ParticleBelief* b) const override;
};

}  // namespace solver_ipft