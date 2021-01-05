#pragma once

#include <ipft/interface/pomdp.hpp>
#include <ipft/interface/spaces.hpp>
#include <ipft/util/random.hpp>

namespace ipft {

/* -------------------------------------------------------------------------- */
/*                       ParticleReinvigorator interface                      */
/* -------------------------------------------------------------------------- */

class ParticleReinvigorator {
   protected:
    const POMDP* model_;

   public:
    ParticleReinvigorator(const POMDP* model) : model_(model) {}
    virtual bool particleReinvigorationNeeded(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const = 0;
    virtual std::vector<State*> reinvigorate(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const = 0;
    virtual ParticleReinvigorator* clone() const = 0;
};

/* -------------------------------------------------------------------------- */
/*                              NoReinvigoration                              */
/* -------------------------------------------------------------------------- */

class NoReinvigoration final : public ParticleReinvigorator {
   public:
    NoReinvigoration() : ParticleReinvigorator(nullptr) {}
    bool particleReinvigorationNeeded(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const final;
    std::vector<State*> reinvigorate(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const final;
    virtual ParticleReinvigorator* clone() const override;
};

/* -------------------------------------------------------------------------- */
/*                      SimpleParticleReinvigorator class                     */
/* -------------------------------------------------------------------------- */

class SimpleParticleReinvigorator : public ParticleReinvigorator {
   public:
    SimpleParticleReinvigorator(const POMDP* model) : ParticleReinvigorator(model) {}
    virtual bool particleReinvigorationNeeded(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const override;
    virtual std::vector<State*> reinvigorate(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const override;
    virtual ParticleReinvigorator* clone() const override;

   private:
    bool allParticlesEqual(const std::vector<State*>& particleSet) const;
};

/* -------------------------------------------------------------------------- */
/*                          ObsAdaptiveReinvigorator                          */
/* -------------------------------------------------------------------------- */
// reimplementation of obs_adaptive_pf.jl from SunbergTypes (https://github.com/zsunberg/ContinuousPOMDPTreeSearchExperiments.jl/blob/master/src/updaters.jl)
// model needs to have methods newParticle() and maxPossibleWeight()
class ObsAdaptiveReinvigorator : public ParticleReinvigorator {
   public:
    static constexpr double max_frac_replaced = 0.05;

   protected:
    const Random* rand_;
    mutable int n_replaced;

   public:
    ObsAdaptiveReinvigorator(const POMDP* model, const Random* rand) : ParticleReinvigorator(model), rand_(rand) {}
    // particleSet must be weighted and unnormalized
    virtual bool particleReinvigorationNeeded(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const override;
    virtual std::vector<State*> reinvigorate(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const override;
    virtual ParticleReinvigorator* clone() const override;
};

}  // namespace ipft