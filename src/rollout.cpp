#include <glog/logging.h>

#include <ipft/interface/rollout.hpp>
#include <ipft/util/debug.hpp>
#include <ipft/util/random.hpp>
namespace solver_ipft {

/* ------------------------- Default rollout policy ------------------------- */

IpftValue BeliefRolloutPolicy::rollout(Belief* belief, int depth) const {
    int rolloutIteration = 0;  // start index = 0, since rollout is called with value = locReward + gamma * rollout()
                               // [gamma already present in first iteration]
    IpftValue value;           // default constructor initializes to zero
    if (depth == 0)            // if (belief->isTerminalBelief())
    {
        delete belief;
        return value;
    }
    for (int d = depth; d >= 0; d--) {
        //* Random action selection
        Action act = this->actionChooser_->chooseAction(belief);

        //* Random observation generation
        State* s = belief->sample();
        State* statePosterior = this->model_->transition(*s, act);
        Observation* obs = this->model_->observation(*statePosterior);

        // State* s = belief->sample();                              // in-out (out value unused)
        // Observation* obs = this->model_->allocateObs();           // in-out (out value is the generated observation)
        // double r;                                                 // in-out (unused)
        // bool t = this->model_->transitionStep(*s, act, *obs, r);  // return value unused
        // this->model_->freeState(s);

        //* belief update and state reward calculation
        // copy of belief for transition step
        Belief* bNext = belief->clone();

        double stateReward = bNext->update(act, *obs);

        // free unused variables
        this->model_->freeObs(obs);
        this->model_->freeState(s);
        this->model_->freeState(statePosterior);

        // information reward calculation
        double informationReward = 0.0;
        if (this->infGainRewardCalculator_ != nullptr) {
            informationReward =
                this->infGainRewardCalculator_->computeDiscInfGain(Globals::config.inf_discount_gamma,
                                                                   static_cast<const ParticleBelief*>(bNext),
                                                                   static_cast<const ParticleBelief*>(belief));
        }

        CHECK(!std::isnan(informationReward)) << "Information reward is nan.";

        IpftValue locReward(stateReward, informationReward);

        //* update value
        value += (locReward * std::pow(Globals::config.discount_gamma, rolloutIteration));
        rolloutIteration++;

        //* terminal belief reached?
        if (bNext->isTerminalBelief()) {
            delete belief;
            delete bNext;
            break;
        } else {
            delete belief;
            belief = bNext;
        }
        if (d == 0)  // last iteration
        {
            // clean up memory
            delete bNext;
        }
    }
    return value;
}

/* ------------------------ Uniform action selection ------------------------ */

Action RandomActionChooser::chooseAction(const Belief* belief) const {
    //? Actions are discrete and internally stored as integers, starting with "action index" 0
    // to choose an action randomly, sample integer from uniform distribution between [0, numActions-1]
    int randomActionIndex = this->rand_->nextUniformInt(belief->model_->numActions());
    Action act = static_cast<Action>(randomActionIndex);
    return act;
}

/* --------------------- Deterministic action selection --------------------- */

DeterministicActionChooser::DeterministicActionChooser(std::vector<Action> actions)
    : actions_(actions), round_(0) {}

DeterministicActionChooser::~DeterministicActionChooser() {}

Action DeterministicActionChooser::chooseAction(const Belief* belief) const {
    Action act;
    if (round_ >= this->actions_.size()) {
        act = Globals::noAction;  // error action
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

}  // namespace solver_ipft