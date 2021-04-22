#include <solver_ipft/core/solver.hpp>

namespace solver_ipft {

/* ----------------------------- Value printers ----------------------------- */

std::ostream& operator<<(std::ostream& os, const Value& v) {
    os << v.text();
    return os;
}

/* --------------------- Solver function implementations -------------------- */

Solver::Solver(const POMDP* model, Belief* belief) : model_(model), belief_(belief), root_(nullptr) {
    this->history_ = new History(this->model_);
    // add initial belief to history
    Belief* initialBelief = this->belief_->clone();
    this->history_->addInitialBelief(initialBelief);
}

Solver::Solver(const POMDP* model) : model_(model), belief_(nullptr), root_(nullptr) {
    this->history_ = new History(this->model_);
}

Solver::~Solver() {
    if (root_ != nullptr) {
        delete root_;
    }
    if (belief_ != nullptr) {
        delete belief_;
    }
    delete history_;
}

History* Solver::copyHistory() const {
    History* hist = new History(*(this->history_));
    return hist;
}

const VNode* Solver::getSearchTree() const {
    return root_;
}

}  // namespace solver_ipft