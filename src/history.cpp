#include <solver_ipft/core/history.hpp>
#include <solver_ipft/core/particle_belief.hpp>

#include <iomanip>
#include <sstream>

namespace solver_ipft {
// do not use (only for vector initialization)
History::History() : model_(nullptr) {
}

History::History(std::shared_ptr<POMDP> model) : model_(std::move(model)) {
}

History::~History() {
    if (this->model_ != nullptr) {
        this->model_->freeObss(this->observations_);
    }
}

// copy constructor
History::History(const History& other) : model_(other.model_), actions_(other.actions_) {
    for (int i = 0; i < other.size(); i++) {
        this->observations_.push_back(other.observation(i));
        this->beliefs_.emplace_back(other.belief(i));
    }
    // add last belief
    this->beliefs_.emplace_back(other.lastBelief());
}

// assignment operator
History& History::operator=(const History& rhs) {
    if (this != &rhs) {
        this->actions_ = rhs.actions_;
        this->model_ = rhs.model_;
        this->model_->freeObss(this->observations_);
        this->observations_.clear();
        this->observations_.reserve(rhs.observations_.size());
        this->beliefs_.clear();
        this->beliefs_.reserve(rhs.beliefs_.size());
        for (int i = 0; i < rhs.size(); i++) {
            this->observations_.push_back(rhs.observation(i));
            this->beliefs_.emplace_back(rhs.belief(i));
        }
        // add last belief
        this->beliefs_.emplace_back(rhs.lastBelief());
    }
    return *this;
}
// move copyconstructor
History::History(History&& other) noexcept {
    std::swap(this->actions_, other.actions_);
    std::swap(this->observations_, other.observations_);
    std::swap(this->beliefs_, other.beliefs_);
    this->model_ = other.model_;
}
// move assignment operator
History& History::operator=(History&& rhs) noexcept {
    if (this != &rhs) {
        std::swap(this->actions_, rhs.actions_);
        std::swap(this->observations_, rhs.observations_);
        std::swap(this->beliefs_, rhs.beliefs_);
        this->model_ = rhs.model_;
    }
    return *this;
}
void History::add(Action action, Observation* obs, std::unique_ptr<Belief>&& b) {
    actions_.emplace_back(ValuedAction(action));
    observations_.push_back(obs);
    beliefs_.emplace_back(std::move(b));
}

void History::add(const ValuedAction& valAction, Observation* obs, std::unique_ptr<Belief>&& b) {
    actions_.push_back(valAction);
    observations_.push_back(obs);
    beliefs_.emplace_back(std::move(b));
}

void History::addInitialBelief(std::unique_ptr<Belief>&& b) {
    beliefs_.emplace_back(std::move(b));
}

void History::RemoveLast() {
    actions_.pop_back();
    Observation* obs = observations_.back();
    this->model_->freeObs(obs);
    observations_.pop_back();
}

Action History::action(int t) const {
    return actions_[t].action_;
}

ValuedAction History::valuedAction(int t) const {
    return actions_[t];
}

Observation* History::observation(int t) const {
    Observation* obs = this->model_->copyObs(this->observations_[t]);
    return obs;
}

const Observation* History::observationPointer(int t) const {
    return this->observations_[t];
}

std::unique_ptr<Belief> History::belief(int t) const {
    std::unique_ptr<Belief> b = this->beliefs_[t]->clone();
    return b;
}

const Belief* History::beliefPointer(int t) const {
    return this->beliefs_[t].get();
}

size_t History::size() const {
    return actions_.size();
}

void History::truncate(int d) {
    // free observations to be truncated first
    if (d < size()) {
        for (int i = d + 1; i < this->observations_.size(); i++) {
            this->model_->freeObs(this->observations_[i]);
        }
    }
    actions_.resize(d);
    observations_.resize(d);
}

Action History::lastAction() const {
    return actions_.back().action_;
}

ValuedAction History::lastValuedAction() const {
    return actions_.back();
}

Observation* History::lastObservation() const {
    Observation* obs = this->model_->copyObs(this->observations_.back());
    return obs;
}

std::unique_ptr<Belief> History::lastBelief() const {
    std::unique_ptr<Belief> b = this->beliefs_.back()->clone();
    return b;
}

/**
 * @brief Returns the history with action-observation tuples from timestep s to
 * the end.
 *
 * @param s the action-observation tupes before s will be truncated
 * @return History the new history
 */
History History::suffix(int s) const {
    History history(this->model_);
    for (int i = s; i < size(); i++) {
        ValuedAction act = valuedAction(i); // Action space is discrete
        Observation* obs = this->observation(i);
        std::unique_ptr<Belief> b = belief(i);
        history.add(act, obs, std::move(b));
    }
    return history;
}

/* ----------------------------- printer methods ---------------------------- */

std::ostream& operator<<(std::ostream& os, const History& history) {
    os << std::left << std::setw(50) << history.shortDescription();
    return os;
}
std::string History::shortDescription() const {
    constexpr int maxNumberActionsToPrint = 6;
    std::stringstream ss;
    ss << "[History(" << this->size() << "): ";
    int i;
    for (i = 0; (i < this->size()) && (i < maxNumberActionsToPrint); i++) {
        Action a = this->action(i);
        if (i != (this->size() - 1)) {
            ss << "Act(" << a << "), ";
        } else {
            ss << "Act(" << a << ")";
        }
    }
    if (i == (maxNumberActionsToPrint - 1) && (this->size() - 1) > i) {
        ss << "... ";
    }
    ss << "]";
    return ss.str();
}

std::string History::text() const {
    using namespace std; // NOLINT
    stringstream ss;
    ss << "[History(" << this->size() << "):" << endl;
    for (int i = 0; i < this->size(); i++) {
        ss << right << setw(7) << "(" + to_string(i) + ") - " << this->actions_[i]; // << endl; // setw next line 7
        ss << right << setw(3) << " - " << *(dynamic_cast<ParticleBelief*>(this->beliefs_[i].get())) << " - "
           << this->model_->to_string(this->observations_[i]);
        if (i < (this->size() - 1)) {
            ss << endl;
        }
    }
    ss << "]" << endl;
    return ss.str();
}

} // namespace solver_ipft