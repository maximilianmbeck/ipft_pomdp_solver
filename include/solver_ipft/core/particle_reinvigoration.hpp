#pragma once

#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/spaces.hpp>
#include <solver_ipft/util/random.hpp>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                       ParticleReinvigorator interface                      */
/* -------------------------------------------------------------------------- */

class ParticleReinvigorator {
protected:
  std::shared_ptr<POMDP> model_;

public:
  explicit ParticleReinvigorator(std::shared_ptr<POMDP> model)
      : model_(std::move(model)) {}
  virtual ~ParticleReinvigorator() = default;

  ParticleReinvigorator(const ParticleReinvigorator &) = default;
  ParticleReinvigorator(ParticleReinvigorator &&) = default;
  ParticleReinvigorator &operator=(const ParticleReinvigorator &) = default;
  ParticleReinvigorator &operator=(ParticleReinvigorator &&) = default;

  virtual bool
  particleReinvigorationNeeded(const std::vector<State *> &particleSet,
                               const Action &act,
                               const Observation &obs) const = 0;
  virtual std::vector<State *>
  reinvigorate(const std::vector<State *> &particleSet, const Action &act,
               const Observation &obs) const = 0;
  virtual std::unique_ptr<ParticleReinvigorator> clone() const = 0;
};

/* -------------------------------------------------------------------------- */
/*                              NoReinvigoration                              */
/* -------------------------------------------------------------------------- */

class NoReinvigoration final : public ParticleReinvigorator {
public:
  NoReinvigoration() : ParticleReinvigorator(nullptr) {}
  bool particleReinvigorationNeeded(const std::vector<State *> &particleSet,
                                    const Action &act,
                                    const Observation &obs) const final;
  std::vector<State *> reinvigorate(const std::vector<State *> &particleSet,
                                    const Action &act,
                                    const Observation &obs) const final;
  std::unique_ptr<ParticleReinvigorator> clone() const override;
};

/* -------------------------------------------------------------------------- */
/*                      SimpleParticleReinvigorator class                     */
/* -------------------------------------------------------------------------- */

class SimpleParticleReinvigorator : public ParticleReinvigorator {
public:
  explicit SimpleParticleReinvigorator(std::shared_ptr<POMDP> model)
      : ParticleReinvigorator(std::move(model)) {}
  bool particleReinvigorationNeeded(const std::vector<State *> &particleSet,
                                    const Action &act,
                                    const Observation &obs) const override;
  std::vector<State *> reinvigorate(const std::vector<State *> &particleSet,
                                    const Action &act,
                                    const Observation &obs) const override;
  std::unique_ptr<ParticleReinvigorator> clone() const override;

private:
  bool allParticlesEqual(const std::vector<State *> &particleSet) const;
};

/* -------------------------------------------------------------------------- */
/*                          ObsAdaptiveReinvigorator                          */
/* -------------------------------------------------------------------------- */
// reimplementation of obs_adaptive_pf.jl from SunbergTypes
// (https://github.com/zsunberg/ContinuousPOMDPTreeSearchExperiments.jl/blob/master/src/updaters.jl)
// model needs to have methods newParticle() and maxPossibleWeight()
class ObsAdaptiveReinvigorator : public ParticleReinvigorator {
public:
  static constexpr double max_frac_replaced = 0.05;

protected:
  std::shared_ptr<Random> rand_;
  mutable int n_replaced_;

public:
  ObsAdaptiveReinvigorator(std::shared_ptr<POMDP> model,
                           std::shared_ptr<Random> rand)
      : ParticleReinvigorator(std::move(model)), rand_(std::move(rand)),
        n_replaced_(0) {}
  // particleSet must be weighted and unnormalized
  bool particleReinvigorationNeeded(const std::vector<State *> &particleSet,
                                    const Action &act,
                                    const Observation &obs) const override;
  std::vector<State *> reinvigorate(const std::vector<State *> &particleSet,
                                    const Action &act,
                                    const Observation &obs) const override;
  std::unique_ptr<ParticleReinvigorator> clone() const override;
};

} // namespace solver_ipft