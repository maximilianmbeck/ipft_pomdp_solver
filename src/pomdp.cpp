#include <ipft/interface/pomdp.hpp>

namespace ipft {

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

}  // namespace ipft