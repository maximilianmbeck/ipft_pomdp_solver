#include <stdexcept>
#include <solver_ipft/interface/belief.hpp>
#include <solver_ipft/interface/pomdp.hpp>

namespace solver_ipft {

std::vector<Action> POMDP::preferredActions(const Belief* belief) const {
    return std::vector<Action>();
}

std::vector<Action> POMDP::legalActions(const Belief* belief) const {
    return std::vector<Action>();
}

std::vector<Observation*> POMDP::copyObss(const std::vector<Observation*>& obss) const {
    std::vector<Observation*> copy;
    copy.reserve(obss.size());
    for (auto& obs : obss) {
        copy.push_back(this->copyObs(obs));
    }
    return copy;
}

std::vector<State*> POMDP::copyStates(const std::vector<State*>& states) const {
    std::vector<State*> copy;
    copy.reserve(states.size());
    for (auto& state : states) {
        copy.push_back(this->copyState(state));
    }
    return copy;
}

void POMDP::freeObss(const std::vector<Observation*>& obss) const {
    for (auto& obs : obss) {
        this->freeObs(obs);
    }
}

void POMDP::freeStates(const std::vector<State*>& states) const {
    for (auto& state : states) {
        this->freeState(state);
    }
}

/* -------------------------- dummy implementations ------------------------- */

double POMDP::obsProb(const State& statePosterior, const Observation& obs) const {
    throw std::runtime_error("POMDP::obsProb not supported!");
    return 0.0;
}

double POMDP::obsProb(const State& state,
                      const Action& action,
                      const State& statePosterior,
                      const Observation& obs) const {
    return obsProb(statePosterior, obs);
}

double POMDP::reward(const State& state, const Action& action) const {
    throw std::runtime_error("POMDP::Reward not supported!");
    return 0.0;
}

double POMDP::reward(const State& state, const Action& action, const State& statePosterior) const {
    return reward(state, action);
}

State* POMDP::createStartState() const {
    throw std::runtime_error("POMDP::createStartState not supported!");
    return nullptr;
}

std::unique_ptr<Belief> POMDP::initialBelief(const std::string& type) {
    throw std::runtime_error("POMDP::initialBelief not supported!");
    return nullptr;
}

int POMDP::numDimStateSpace() const {
    throw std::runtime_error("POMDP::numDimStateSpace not supported!");
    return 0;
}

void POMDP::newParticle(State* particle,
                        const std::vector<State*>& particleSet,
                        const Action& act,
                        const Observation& obs) const {
    throw std::runtime_error("POMDP::newParticle not supported!");
}

std::unique_ptr<ActionValue> POMDP::valueOfAction(const Action& act) const {
    throw std::runtime_error("POMDP::valueOfAction not supported!");
    return nullptr;
}

double POMDP::maxPossibleWeight(const Action& action, const Observation& obs) const {
    throw std::runtime_error("POMDP::maxPossibleWeight not supported!");
    return 0.0;
}

} // namespace solver_ipft