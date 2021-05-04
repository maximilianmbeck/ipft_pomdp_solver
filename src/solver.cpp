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
    : Solver(std::move(model), nullptr) {}

History Solver::copyHistory() const { return *this->history_; }

std::shared_ptr<VNode> Solver::getSearchTree() const { return this->root_; }

} // namespace solver_ipft