#include <solver_ipft/core/node.hpp>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                                 Node class                                 */
/* -------------------------------------------------------------------------- */

void Node::setCount(int count) {
    this->count_ = count;
}

int Node::getCount() const {
    return this->count_;
}

int Node::getTreelevel() const {
    return this->treelevel_;
}

void Node::setValue(std::unique_ptr<Value>&& value) {
    this->value_ = std::move(value);
}

std::unique_ptr<Value> Node::cloneValue() const {
    return this->value_->clone();
}

const Value* Node::getValue() const {
    return this->value_.get();
}

double Node::getTotalValue() const {
    return this->value_->total();
}

void Node::updateValueCount(const Value& v) {
    this->value_->update(v, this->count_);
    count_++;
}

/* -------------------------------------------------------------------------- */
/*                                 VNode class                                */
/* -------------------------------------------------------------------------- */

VNode::VNode(std::shared_ptr<POMDP> model, std::shared_ptr<QNode> parent,
             Observation *obs, std::unique_ptr<Belief> &&belief, int level)
    : Node(std::move(model), level), parent_(std::move(parent)), obsEdge_(obs),
      belief(std::move(belief)) {}

VNode::~VNode() {
    // free observations
    if (obsEdge_ != nullptr) {
        this->model_->freeObs(obsEdge_);
    }
    this->model_->freeObss(this->obs_archive_);
}

Observation* VNode::getObservationObj() const {
    return this->model_->copyObs(this->obsEdge_);
}

void VNode::setObs(Observation* obs) {
    if (this->obsEdge_ != nullptr) {
        this->obs_archive_.push_back(this->obsEdge_);
    }
    this->obsEdge_ = obs;
}

std::unique_ptr<Belief> VNode::getBelief() const {
  if (this->belief) {
    return this->belief->clone();
  }
    return nullptr;
}

void VNode::setBelief(std::unique_ptr<Belief>&& belief) {
  if (this->belief) {
    this->belief_archive_.emplace_back(std::move(this->belief));
  };
    this->belief = std::move(belief);
}

bool VNode::isLeaf() const {
    return this->actChildren_.empty();
}

std::shared_ptr<QNode> VNode::getParent() const {
    return parent_;
}

std::vector<std::shared_ptr<QNode>>& VNode::children() const {
    return this->actChildren_;
}

std::shared_ptr<QNode> VNode::child(Action action) const {
    return this->actChildren_[action];
}

std::vector<ValuedAction> VNode::getValuedActions() const {
    std::vector<ValuedAction> valuedActions;
    for (auto& actChild : this->actChildren_) {
        ValuedAction va(actChild->getAction(), actChild->cloneValue(), actChild->getCount());
        valuedActions.push_back(va);
    }
    return valuedActions;
}

std::shared_ptr<QNode> VNode::maxChild() const {
    std::shared_ptr<QNode> maxChild = nullptr;
    double maxVal = Globals::NEG_INFTY;
    for (auto& actChild : this->actChildren_) {
        double curActVal = actChild->getTotalValue();
        if (curActVal > maxVal) {
            maxVal = curActVal;
            maxChild = actChild;
        }
    }
    return maxChild;
}

History VNode::maximumValueActionObservationSequence(const Action& action) const {
    History actObsSeq(this->model_);

    auto qn = this->child(action);
    assert(qn);
    while (!(qn->isLeaf())) {
        auto vn = qn->maxChild();
        ValuedAction act(qn->getAction(), qn->cloneValue(), qn->getCount());
        Observation* obs = vn->getObservationObj();
        auto b = vn->getBelief();
        // add action-observation pair to history
        actObsSeq.add(act, obs, std::move(b));

        // go down the tree to the next (deeper) treelevel
        qn = vn->maxChild();
        assert(qn);
    }

    return actObsSeq;
}

/* -------------------------------------------------------------------------- */
/*                                 QNode class                                */
/* -------------------------------------------------------------------------- */

QNode::QNode(std::shared_ptr<POMDP> model, std::shared_ptr<VNode> parent, Action edge, int level)
        : Node(std::move(model), level), parent_(std::move(parent)), actEdge_(edge) {
}

QNode::~QNode() = default;

Action QNode::getAction() const {
    return this->actEdge_;
}

std::shared_ptr<VNode> QNode::getParent() const {
    return parent_;
}

std::vector<std::shared_ptr<VNode>>& QNode::children() const {
    return this->obsChildren_;
}

std::shared_ptr<VNode> QNode::maxChild() const {
    std::shared_ptr<VNode> maxChild = nullptr;
    double maxVal = Globals::NEG_INFTY;
    for (auto& obsChild : this->obsChildren_) {
        double curObsVal = obsChild->getTotalValue();
        if (curObsVal > maxVal) {
            maxVal = curObsVal;
            maxChild = obsChild;
        }
    }

    return maxChild;
}

bool QNode::isLeaf() const {
    return this->obsChildren_.empty();
}

} // namespace solver_ipft