#include <glog/logging.h>

#include <solver_ipft/interface/rollout.hpp>
#include <solver_ipft/util/debug.hpp>
#include <solver_ipft/util/random.hpp>
namespace solver_ipft {

/* ------------------------- Default rollout policy ------------------------- */

IpftValue BeliefRolloutPolicy::rollout(std::unique_ptr<Belief>&& belief, int depth) const {
    int rolloutIteration = 0; // start index = 0, since rollout is called with value = locReward +
                              // gamma * rollout() [gamma already present in first iteration]
    IpftValue value;          // default constructor initializes to zero
    if (depth == 0) {
        return value;
    }
    for (int d = depth; d >= 0; d--) {
        //* Random action selection
        Action act = this->actionChooser_->chooseAction(belief.get());

        //* Random observation generation
        State* s = belief->sample();
        State* statePosterior = this->model_->transition(*s, act);
        Observation* obs = this->model_->observation(*statePosterior);

        //* belief update and state reward calculation
        // copy of belief for transition step
        auto bNext = belief->clone();

        double stateReward = bNext->update(act, *obs);

        CHECK(!std::isnan(stateReward)) << "State reward is nan.";

        // free unused variables
        this->model_->freeObs(obs);
        this->model_->freeState(s);
        this->model_->freeState(statePosterior);

        // information reward calculation
        double informationReward = 0.0;
        if (this->infGainRewardCalculator_ != nullptr) {
            informationReward = this->infGainRewardCalculator_->computeDiscInfGain(
                Globals::config.inf_discount_gamma, bNext.get(), belief.get());
        }

        CHECK(!std::isnan(informationReward)) << "Information reward is nan.";

        IpftValue locReward(stateReward, informationReward);

        //* update value
        value += (locReward * std::pow(Globals::config.discount_gamma, rolloutIteration));
        rolloutIteration++;

        //* terminal belief reached?
        if (!bNext->isTerminalBelief()) {
            belief = std::move(bNext);
        } else {
            break;
        }
    }
    return value;
}

/* ------------------------ Uniform action selection ------------------------ */

Action RandomActionChooser::chooseAction(const Belief* belief) const {
    //? Actions are discrete and internally stored as integers, starting with
    //"action index" 0
    // to choose an action randomly, sample integer from uniform distribution
    // between [0, numActions-1]
    int randomActionIndex = this->rand_->nextUniformInt(belief->model_->numActions());
    auto act = static_cast<Action>(randomActionIndex);
    return act;
}

/* --------------------- Deterministic action selection --------------------- */

DeterministicActionChooser::DeterministicActionChooser(std::vector<Action> actions)
        : actions_(std::move(actions)), round_(0) {
}

Action DeterministicActionChooser::chooseAction(const Belief* belief) const {
    Action act;
    if (round_ >= this->actions_.size()) {
        act = Globals::noAction; // error action
    } else {
        act = actions_[round_];
        round_++;
    }
    return act;
}

int DeterministicActionChooser::numberOfActionsLeft() const {
    return actions_.size() - round_;
}

void DeterministicActionChooser::reset() {
    round_ = 0;
}

/* ----------------- DeterministicSingleActionChooser class ----------------- */

Action DeterministicSingleActionChooser::chooseAction(const Belief* belief) const {
    return this->deterministicAction;
}

} // namespace solver_ipft