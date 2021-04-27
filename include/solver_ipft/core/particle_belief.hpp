#pragma once

#include <solver_ipft/core/particle_reinvigoration.hpp>
#include <solver_ipft/interface/belief.hpp>
#include <solver_ipft/util/random.hpp>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                            ParticleBelief class                            */
/* -------------------------------------------------------------------------- */

class ParticleBelief : public Belief {
  // class for console output of the belief
  friend class ParticleSetToString;

protected:
  const Random *rand_;
  const ParticleReinvigorator *afterResampleReinvigorator_;
  int num_particles_;
  bool beliefTerminated_;

public:
  std::vector<State *> particles_; // resampled unweighted particle set
  std::vector<State *>
      weighted_posterior_particles_; // weighted particle set before resampling
                                     // (predicted, reweighted samples)
public:
  ParticleBelief(std::vector<State *> particles, bool beliefTerminated,
                 const POMDP *model, const Random *rand,
                 const ParticleReinvigorator *afterResampleReinvigorator =
                     new NoReinvigoration());
  ParticleBelief(std::vector<State *> particles,
                 std::vector<State *> wp_particles, bool beliefTerminated,
                 const POMDP *model, const Random *rand,
                 const ParticleReinvigorator *afterResampleReinvigorator =
                     new NoReinvigoration());

  ~ParticleBelief() override;

  ParticleBelief(const ParticleBelief &) = delete;
  ParticleBelief(ParticleBelief &&) = delete;
  ParticleBelief &operator=(const ParticleBelief &) = delete;
  ParticleBelief &operator=(ParticleBelief &&) = delete;

  /**
   * @brief Returns a copy of the particles the belief consists of
   *
   * @return std::vector<State*> the particles
   */
  virtual std::vector<State *> particles() const;

  virtual const State *particle(int i) const;

  virtual int numParticles() const;

  virtual std::vector<State *>
  sample(int num, const std::vector<State *> &particleSet) const;

  std::vector<State *> sample(int num) const override;

  State *sample() const override;

  ParticleBelief *sampleParticleBelief(int num) const override;

  double update(const Action &action, const Observation &obs) override;

  virtual std::string text(const std::vector<State *> &particleSet) const;
  std::string text() const override;

  virtual std::string
  detailedText(const std::vector<State *> &particleSet) const;
  std::string detailedText() const override;

  // makes a full / deep copy of the belief
  Belief *clone() const override;

  bool isTerminalBelief() const override;

  virtual double particleWeightSum() const;

  State *mean() const override;

  State *std() const override;

  virtual void
  setReinvigorationStrategy(const ParticleReinvigorator *reinvigorator);

  friend std::ostream &operator<<(std::ostream &os,
                                  const ParticleBelief &partBelief);

private:
  void init();

protected:
  virtual void reinvigorateParticlesAfterResampling(const Action &act,
                                                    const Observation &obs);
};

} // namespace solver_ipft