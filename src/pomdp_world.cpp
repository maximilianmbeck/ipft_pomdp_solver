#include <solver_ipft/core/pomdp_world.hpp>

namespace solver_ipft {
POMDPWorld::POMDPWorld(std::shared_ptr<POMDP> model) : model_(std::move(model)), stepReward_(0.0) {
}

POMDPWorld::~POMDPWorld() {
    if (state_ != nullptr) {
        this->model_->freeState(state_);
    }
    this->model_->freeStates(stateSequence_);
}

bool POMDPWorld::connect() {
    return true;
}

State* POMDPWorld::initialize() {
    // clear state sequence
    this->model_->freeStates(stateSequence_);
    stateSequence_.clear();

    this->state_ = this->model_->createStartState();
    State* s = this->model_->copyState(this->state_);
    this->stateSequence_.push_back(s);
    return this->state_;
}

State* POMDPWorld::getCurrentState() const {
    State* copy = this->model_->copyState(this->state_);
    return copy;
}

void POMDPWorld::setState(State* state) {
    if (state_ != nullptr) {
        this->model_->freeState(this->state_);
    }
    this->state_ = state;
    // add state to state sequence
    State* s = this->model_->copyState(this->state_);
    this->stateSequence_.push_back(s);
}

bool POMDPWorld::executeAction(const Action& action, Observation*& obs) {
    // transition world state
    State* stateP = this->model_->transition(*this->state_, action);
    stepReward_ = this->model_->reward(*this->state_, action, *stateP);
    Observation* o = this->model_->observation(*stateP);
    // obs.set(o->get(0), 0); // necessary if obs passed by reference
    obs = o;
    bool terminal = this->model_->terminalState(action, *stateP);

    this->setState(stateP);
    return terminal;
}

double POMDPWorld::getReward() const {
    return this->stepReward_;
}

std::vector<State*> POMDPWorld::copyWorldStateSequence() const {
    std::vector<State*> stateSeq;
    for (auto& state : this->stateSequence_) {
        stateSeq.push_back(this->model_->copyState(state));
    }
    return stateSeq;
}

} // namespace solver_ipft
