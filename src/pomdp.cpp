#include <ipft/interface/pomdp.hpp>
#include <stdexcept>

namespace solver_ipft {

POMDP::POMDP() {}

POMDP::~POMDP() {}

std::vector<Action> POMDP::preferredActions(const Belief *belief) const {
    return std::vector<Action>();
}

std::vector<Action> POMDP::legalActions(const Belief *belief) const {
    return std::vector<Action>();
}

std::vector<Observation *> POMDP::copyObss(const std::vector<Observation *> &obss) const {
    std::vector<Observation *> copy;
    for (int i = 0; i < obss.size(); i++) {
        copy.push_back(this->copyObs(obss[i]));
    }
    return copy;
}

std::vector<State *> POMDP::copyStates(const std::vector<State *> &states) const {
    std::vector<State *> copy;
    for (int i = 0; i < states.size(); i++) {
        copy.push_back(this->copyState(states[i]));
    }
    return copy;
}

void POMDP::freeObss(const std::vector<Observation *> &obss) const {
    for (int i = 0; i < obss.size(); i++) {
        this->freeObs(obss[i]);
    }
}

void POMDP::freeStates(const std::vector<State *> &states) const {
    for (int i = 0; i < states.size(); i++) {
        this->freeState(states[i]);
    }
}

/* -------------------------- dummy implementations ------------------------- */

double POMDP::obsProb(const State &statePosterior, const Observation &obs) const {
    throw std::runtime_error("POMDP::obsProb not supported!");
    return 0.0;
}

double POMDP::obsProb(const State &state, const Action &action, const State &statePosterior, const Observation &obs) const {
    return obsProb(statePosterior, obs);
}

double POMDP::reward(const State &state, const Action &action) const {
    throw std::runtime_error("POMDP::Reward not supported!");
    return 0.0;
}

double POMDP::reward(const State &state, const Action &action, const State &statePosterior) const {
    return reward(state, action);
}

State *POMDP::createStartState() const {
    throw std::runtime_error("POMDP::createStartState not supported!");
    return nullptr;
}

Belief *POMDP::initialBelief(std::string type) const {
    throw std::runtime_error("POMDP::initialBelief not supported!");
    return nullptr;
}

int POMDP::numDimStateSpace() const {
    throw std::runtime_error("POMDP::numDimStateSpace not supported!");
    return 0;
}

void POMDP::newParticle(State *particle, const std::vector<State *> &particleSet, const Action &act, const Observation &obs) const {
    throw std::runtime_error("POMDP::newParticle not supported!");
}

std::vector<State *> POMDP::similarStates(const State &state, int count) const {
    throw std::runtime_error("POMDP::similarStates not supported!");
    std::vector<State *> vec;
    return vec;
}

std::unique_ptr<ActionValue> POMDP::valueOfAction(const Action &act) const {
    throw std::runtime_error("POMDP::valueOfAction not supported!");
    return nullptr;
}

double POMDP::maxPossibleWeight(const Action &act, const Observation &obs) const {
    throw std::runtime_error("POMDP::maxPossibleWeight not supported!");
    return 0.0;
}

}  // namespace solver_ipft