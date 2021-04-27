#pragma once

#include <solver_ipft/core/particle_belief.hpp>

namespace solver_ipft {

/* -------------------------------- Interface ------------------------------- */

class DensityEstimator {
public:
  DensityEstimator() = default;
  virtual ~DensityEstimator() = default;

  DensityEstimator(const DensityEstimator &) = delete;
  DensityEstimator(DensityEstimator &&) = delete;
  DensityEstimator &operator=(const DensityEstimator &) = delete;
  DensityEstimator &operator=(DensityEstimator &&) = delete;

  virtual double computeDensityValue(const State *s,
                                     const ParticleBelief *b) const = 0;
};

/* -------------------------------- Interface ------------------------------- */

class BandwidthSelector {
public:
  BandwidthSelector() = default;
  virtual ~BandwidthSelector() = default;

  BandwidthSelector(const BandwidthSelector &) = delete;
  BandwidthSelector(BandwidthSelector &&) = delete;
  BandwidthSelector &operator=(const BandwidthSelector &) = delete;
  BandwidthSelector &operator=(BandwidthSelector &&) = delete;

  virtual double computeBandwidth(const ParticleBelief *b) const = 0;
  virtual std::vector<double>
  computeBandwidthMatrixDiagElmts(const ParticleBelief *b) const = 0;
};

/* -------------------------------- Interface ------------------------------- */

class KernelFunction {
public:
  KernelFunction() = default;
  virtual ~KernelFunction() = default;

  KernelFunction(const KernelFunction &) = delete;
  KernelFunction(KernelFunction &&) = delete;
  KernelFunction &operator=(const KernelFunction &) = delete;
  KernelFunction &operator=(KernelFunction &&) = delete;

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
  RotBandwidthSelector() = default;
  ~RotBandwidthSelector() override = default;

  RotBandwidthSelector(const RotBandwidthSelector &) = delete;
  RotBandwidthSelector(RotBandwidthSelector &&) = delete;
  RotBandwidthSelector &operator=(const RotBandwidthSelector &) = delete;
  RotBandwidthSelector &operator=(RotBandwidthSelector &&) = delete;

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
  BandwidthSelector *bwSelector_;
  KernelFunction *kernel_;

public:
  KernelDensityEstimator() {
    bwSelector_ = new RotBandwidthSelector(); // default bandwidth selector
    kernel_ = new NormalKernel();             // default kernel
  }

  ~KernelDensityEstimator() override {
    delete bwSelector_;
    delete kernel_;
  }

  KernelDensityEstimator(const KernelDensityEstimator &) = delete;
  KernelDensityEstimator(KernelDensityEstimator &&) = delete;
  KernelDensityEstimator &operator=(const KernelDensityEstimator &) = delete;
  KernelDensityEstimator &operator=(KernelDensityEstimator &&) = delete;

  double computeDensityValue(const State *s,
                             const ParticleBelief *b) const override;
};

} // namespace solver_ipft