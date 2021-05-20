#include <glog/logging.h>

#include <algorithm>
#include <numeric>
#include <solver_ipft/core/particle_reinvigoration.hpp>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                              NoReinvigoration                              */
/* -------------------------------------------------------------------------- */

bool NoReinvigoration::particleReinvigorationNeeded(const std::vector<State*>& particleSet,
                                                    const Action& act,
                                                    const Observation& obs) const {
    return false;
}

std::vector<State*> NoReinvigoration::reinvigorate(const std::vector<State*>& particleSet,
                                                   const Action& act,
                                                   const Observation& obs) const {
    return particleSet;
}

std::unique_ptr<ParticleReinvigorator> NoReinvigoration::clone() const {
    return std::make_unique<NoReinvigoration>();
}


/* -------------------------------------------------------------------------- */
/*                          ObsAdaptiveReinvigorator                          */
/* -------------------------------------------------------------------------- */

bool ObsAdaptiveReinvigorator::particleReinvigorationNeeded(const std::vector<State*>& particleSet,
                                                            const Action& act,
                                                            const Observation& obs) const {
    CHECK_GT(particleSet.size(), 0);
    // calculate n_replaced
    double mpw = this->model_->maxPossibleWeight(act, obs);
    // maximal weight in particle set
    auto stateMaxWeight = std::max_element(particleSet.begin(), particleSet.end(), State::weightCompare);
    double maxWeight = (*stateMaxWeight)->weight;
    double frac_replaced = ObsAdaptiveReinvigorator::max_frac_replaced * std::max(0.0, 1 - maxWeight / mpw);
    this->n_replaced_ = static_cast<int>(std::floor(frac_replaced * particleSet.size()));
    // DLOG(INFO) << "[PF] n_replaced: " << n_replaced_;
    return this->n_replaced_ > 0;
}
std::vector<State*> ObsAdaptiveReinvigorator::reinvigorate(const std::vector<State*>& particleSet,
                                                           const Action& act,
                                                           const Observation& obs) const {
    DLOG(WARNING) << "[PF] Number of particles replaced: " << this->n_replaced_;
    // pick randomly (uniformly) n_replaced particles from the particle set
    // create an vector holding all the indices for the particle set
    // ([0:particleSet.size()-1])
    std::vector<int> indices(particleSet.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), *(this->rand_->engine()));
    // replace the selected particles by new ones
    std::vector<State*> obsAdaptedPs = particleSet;
    for (int i = 0; i < this->n_replaced_; i++) {
        this->model_->newParticle(obsAdaptedPs[indices[i]], particleSet, act, obs);
    }
    auto o = std::move(obsAdaptedPs);
    return o;
}

std::unique_ptr<ParticleReinvigorator> ObsAdaptiveReinvigorator::clone() const {
    return std::make_unique<ObsAdaptiveReinvigorator>(this->model_, this->rand_);
}

} // namespace solver_ipft
