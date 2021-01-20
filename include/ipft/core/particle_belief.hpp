#pragma once

#include "ipft/core/particle_reinvigoration.hpp"
#include "ipft/interface/belief.hpp"
#include "ipft/util/random.hpp"

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                            ParticleBelief class                            */
/* -------------------------------------------------------------------------- */

class ParticleBelief : public Belief {
    // class for console output of the belief
    friend class ParticleSetToString;

   protected:
    const Random* rand_;
    const ParticleReinvigorator* afterResampleReinvigorator_;
    int num_particles_;
    bool beliefTerminated_;

   public:
    std::vector<State*> particles_;                     // resampled unweighted particle set
    std::vector<State*> weighted_posterior_particles_;  // weighted particle set before resampling (predicted, reweighted samples)
   public:
    ParticleBelief(std::vector<State*> particles, bool beliefTerminated, const POMDP* model, const Random* rand, const ParticleReinvigorator* afterResampleReinvigorator = new NoReinvigoration());
    ParticleBelief(std::vector<State*> particles, std::vector<State*> wp_particles, bool beliefTerminated, const POMDP* model, const Random* rand, const ParticleReinvigorator* afterResampleReinvigorator = new NoReinvigoration());

    virtual ~ParticleBelief();

    /**
     * @brief Returns a copy of the particles the belief consists of
     *
     * @return std::vector<State*> the particles
     */
    virtual std::vector<State*> particles() const;

    virtual const State* particle(int i) const;

    virtual int numParticles() const;

    virtual std::vector<State*> sample(int num, const std::vector<State*>& particleSet) const;

    virtual std::vector<State*> sample(int num) const override;

    virtual State* sample() const override;

    virtual ParticleBelief* sampleParticleBelief(int num) const override;

    virtual double update(const Action& action, const Observation& obs);

    virtual std::string text(const std::vector<State*>& particleSet) const;
    virtual std::string text() const override;

    virtual std::string detailedText(const std::vector<State*>& particleSet) const;
    virtual std::string detailedText() const override;

    // makes a full / deep copy of the belief
    virtual Belief* clone() const override;

    virtual bool isTerminalBelief() const override;

    virtual double particleWeightSum() const;

    virtual State* mean() const override;

    virtual State* std() const override;

    virtual void setReinvigorationStrategy(const ParticleReinvigorator* reinvigorator);

    friend std::ostream& operator<<(std::ostream& os, const ParticleBelief& partBelief);

   private:
    void init();

   protected:
    virtual void reinvigorateParticlesAfterResampling(const Action& act, const Observation& obs);
};

}  // namespace solver_ipft