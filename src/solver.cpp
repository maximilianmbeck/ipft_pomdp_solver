#include <solver_ipft/core/solver.hpp>

namespace solver_ipft {

/* ----------------------------- Value printers ----------------------------- */

std::ostream &operator<<(std::ostream &os, const Value &v) {
  os << v.text();
  return os;
}

/* --------------------- Solver function implementations -------------------- */

Solver::Solver(std::shared_ptr<POMDP> model, std::unique_ptr<Belief> &&belief)
    : model_(std::move(model)), belief_(std::move(belief)), root_(nullptr),
      history_(std::make_unique<History>(model_)) {
  // add initial belief to history
  auto initialBelief = this->belief_->clone();
  this->history_->addInitialBelief(std::move(initialBelief));
}

Solver::Solver(std::shared_ptr<POMDP> model)
    : model_(std::move(model)), belief_(nullptr), root_(nullptr),
      history_(std::make_unique<History>(model_)) {}

History Solver::copyHistory() const { return *this->history_; }

std::shared_ptr<VNode> Solver::getSearchTree() const { return this->root_; }

void Solver::resetBelief(std::unique_ptr<Belief> &&b) {
  // clear history
  this->history_ = std::make_unique<History>(this->model_);

  // set new belief
  this->belief_ = std::move(b);
  // add initial belief to history
  this->history_->addInitialBelief(this->belief_->clone());

  // clear root
  this->root_ = nullptr;
}

void Solver::setBelief(std::unique_ptr<Belief> &&b) {
  // set new belief
  this->belief_ = std::move(b);

  // clear root
  this->root_ = nullptr;
}

const Belief *Solver::getBelief() const { return this->belief_.get(); }

std::unique_ptr<Belief> Solver::copyBelief() const {
  return this->belief_->clone();
}

} // namespace solver_ipft