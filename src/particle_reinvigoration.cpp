#include <glog/logging.h>

#include <algorithm>
#include <ipft/core/particle_reinvigoration.hpp>
#include <numeric>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                              NoReinvigoration                              */
/* -------------------------------------------------------------------------- */

bool NoReinvigoration::particleReinvigorationNeeded(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const {
    return false;
}

std::vector<State*> NoReinvigoration::reinvigorate(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const {
    return particleSet;
}

ParticleReinvigorator* NoReinvigoration::clone() const {
    return new NoReinvigoration();
}

/* -------------------------------------------------------------------------- */
/*                      SimpleParticleReinvigorator class                     */
/* -------------------------------------------------------------------------- */

bool SimpleParticleReinvigorator::particleReinvigorationNeeded(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const {
    return this->allParticlesEqual(particleSet);
}

std::vector<State*> SimpleParticleReinvigorator::reinvigorate(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const {
    // all particles are equal
    CHECK(this->allParticlesEqual(particleSet)) << "Reinvigorate particle set, but reinvigoration criterion not met.";

    State* s = particleSet[0];

    std::vector<State*> reinvigoratedStates = this->model_->similarStates(*s, particleSet.size());
    return reinvigoratedStates;
}

bool SimpleParticleReinvigorator::allParticlesEqual(const std::vector<State*>& particleSet) const {
    for (int i = 1; i < particleSet.size(); i++) {
        const State* prevState = particleSet[i - 1];
        const State* curState = particleSet[i];
        if (!(prevState->equals(*curState))) {
            return false;
        }
    }
    return true;
}

ParticleReinvigorator* SimpleParticleReinvigorator::clone() const {
    return new SimpleParticleReinvigorator(this->model_);
}

/* -------------------------------------------------------------------------- */
/*                          ObsAdaptiveReinvigorator                          */
/* -------------------------------------------------------------------------- */

bool ObsAdaptiveReinvigorator::particleReinvigorationNeeded(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const {
    CHECK_GT(particleSet.size(), 0);
    // calculate n_replaced
    double mpw = this->model_->maxPossibleWeight(act, obs);
    // maximal weight in particle set
    auto stateMaxWeight = std::max_element(particleSet.begin(), particleSet.end(), State::weightCompare);
    double maxWeight = (*stateMaxWeight)->weight_;
    double frac_replaced = ObsAdaptiveReinvigorator::max_frac_replaced * std::max(0.0, 1 - maxWeight / mpw);
    this->n_replaced = static_cast<int>(std::floor(frac_replaced * particleSet.size()));
    // DLOG(INFO) << "[PF] n_replaced: " << n_replaced;
    return this->n_replaced > 0;
}
std::vector<State*> ObsAdaptiveReinvigorator::reinvigorate(const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const {
    DLOG(WARNING) << "[PF] Number of particles replaced: " << this->n_replaced;
    // pick randomly (uniformly) n_replaced particles from the particle set
    // create an vector holding all the indices for the particle set ([0:particleSet.size()-1])
    std::vector<int> indices(particleSet.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), *(this->rand_->engine()));
    // replace the selected particles by new ones
    std::vector<State*> obsAdaptedPs = particleSet;
    for (int i = 0; i < this->n_replaced; i++) {
        this->model_->newParticle(obsAdaptedPs[indices[i]], particleSet, act, obs);
    }
    return std::move(obsAdaptedPs);
}

ParticleReinvigorator* ObsAdaptiveReinvigorator::clone() const {
    return new ObsAdaptiveReinvigorator(this->model_, this->rand_);
}

}  // namespace solver_ipft