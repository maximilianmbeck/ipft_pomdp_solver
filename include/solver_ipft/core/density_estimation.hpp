#pragma once

#include "solver_ipft/core/particle_belief.hpp"

namespace solver_ipft {

/* -------------------------------- Interface ------------------------------- */

class DensityEstimator {
public:
  DensityEstimator(){};
  virtual ~DensityEstimator(){};

  virtual double computeDensityValue(const State *s,
                                     const ParticleBelief *b) const = 0;
};

/* -------------------------------- Interface ------------------------------- */

class BandwidthSelector {
public:
  BandwidthSelector(){};
  virtual ~BandwidthSelector(){};

  virtual double computeBandwidth(const ParticleBelief *b) const = 0;
  virtual std::vector<double>
  computeBandwidthMatrixDiagElmts(const ParticleBelief *b) const = 0;
};

/* -------------------------------- Interface ------------------------------- */

class KernelFunction {
public:
  virtual double univariateValue(const State *s, const ParticleBelief *belief,
                                 const double &bandwidth) const = 0;
  virtual double
  multivariateValue(const State *s, const ParticleBelief *belief,
                    const std::vector<double> &bandwidth) const = 0;
};

/* ------------------- Classes implementing the interface ------------------- */

// Rule-of-Thumb Bandwidth selector (default bandwidth selector)
class RotBandwidthSelector : public BandwidthSelector {
public:
  RotBandwidthSelector(){};
  virtual ~RotBandwidthSelector(){};

  double computeBandwidth(const ParticleBelief *b) const override;
  std::vector<double>
  computeBandwidthMatrixDiagElmts(const ParticleBelief *b) const override;
};

// Normal (Gaussian) kernel (default kernel)
class NormalKernel : public KernelFunction {
public:
  double univariateValue(const State *s, const ParticleBelief *belief,
                         const double &bandwidth) const override;
  double multivariateValue(const State *s, const ParticleBelief *belief,
                           const std::vector<double> &bandwidth) const override;
};

class KernelDensityEstimator : public DensityEstimator {
private:
  BandwidthSelector *bwSelector;
  KernelFunction *kernel;

public:
  KernelDensityEstimator() {
    bwSelector = new RotBandwidthSelector(); // default bandwidth selector
    kernel = new NormalKernel();             // default kernel
  }

  virtual ~KernelDensityEstimator() {
    delete bwSelector;
    delete kernel;
  }

  double computeDensityValue(const State *s,
                             const ParticleBelief *b) const override;
};

} // namespace solver_ipft