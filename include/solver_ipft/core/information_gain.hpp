#pragma once

#include <solver_ipft/core/density_estimation.hpp>
#include <solver_ipft/core/particle_belief.hpp>

namespace solver_ipft {

/* -------------------------------- Interface ------------------------------- */

class DiscountedInformationGain {
public:
  DiscountedInformationGain() = default;
  virtual ~DiscountedInformationGain() = default;

  DiscountedInformationGain(const DiscountedInformationGain &) = delete;
  DiscountedInformationGain(DiscountedInformationGain &&) = delete;
  DiscountedInformationGain &
  operator=(const DiscountedInformationGain &) = delete;
  DiscountedInformationGain &operator=(DiscountedInformationGain &&) = delete;

  virtual double computeDiscInfGain(double discFactor, const Belief *bnext,
                                    const Belief *b) const = 0;
};

/* ------------------- Classes implementing the interface ------------------- */

class EntropyInfGain : public DiscountedInformationGain {
private:
  std::unique_ptr<DensityEstimator> densEstimator;

public:
  EntropyInfGain() {
    densEstimator =
        std::make_unique<KernelDensityEstimator>(); // default density estimator
  };
  ~EntropyInfGain() override = default;

  EntropyInfGain(const EntropyInfGain &) = delete;
  EntropyInfGain(EntropyInfGain &&) = delete;
  EntropyInfGain &operator=(const EntropyInfGain &) = delete;
  EntropyInfGain &operator=(EntropyInfGain &&) = delete;

  double computeDiscInfGain(double discFactor, const Belief *bnext,
                            const Belief *b) const override;

  // protected:
  double computeEntropyEstimate(const Belief *b) const;
};

class NoInfGain : public DiscountedInformationGain {
public:
  NoInfGain() = default;
  ~NoInfGain() override = default;

  NoInfGain(const NoInfGain &) = delete;
  NoInfGain(NoInfGain &&) = delete;
  NoInfGain &operator=(const NoInfGain &) = delete;
  NoInfGain &operator=(NoInfGain &&) = delete;

  double computeDiscInfGain(double discFactor, const Belief *bnext,
                            const Belief *b) const override;
};

} // namespace solver_ipft