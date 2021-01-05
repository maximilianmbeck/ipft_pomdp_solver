#include "ipft/core/node.hpp"

namespace ipft {

/* -------------------------------------------------------------------------- */
/*                                 Node class                                 */
/* -------------------------------------------------------------------------- */

Node::~Node() {
    if (this->value_ != nullptr) {
        delete this->value_;
    }
}

// Node::Node(const Node &other)
//     : model_(other.model_), count_(other.count_), treelevel_(other.treelevel_), value_(other.value_->clone()) {}

// Node::Node(Node &&other) noexcept
//     : model_(other.model_), count_(other.count_), treelevel_(other.treelevel_)
// {
//     this->value_ = other.value_;
//     other.value_ = nullptr;
// }

void Node::setCount(int count) {
    this->count_ = count;
}

int Node::getCount() const {
    return this->count_;
}

int Node::getTreelevel() const {
    return this->treelevel_;
}

void Node::setValue(Value* value) {
    if (this->value_ != nullptr) {
        delete this->value_;
    }
    this->value_ = value;
}

Value* Node::getValueObj() const {
    return this->value_->clone();
}

const Value* Node::getValueRef() const {
    return this->value_;
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

VNode::VNode(const POMDP* model, QNode* parent, Observation* obs, Belief* belief, int depth)
    : Node(model, depth), parent_(parent), obsEdge_(obs), belief_(belief) {
}

VNode::~VNode() {
    if (obsEdge_ != nullptr) {
        this->model_->freeObs(obsEdge_);
    }
    delete this->belief_;
    for (int i = 0; i < this->actChildren_.size(); i++) {
        delete this->actChildren_[i];
    }
}

Observation* VNode::getObservationObj() const {
    return this->model_->copyObs(this->obsEdge_);
}

void VNode::setObs(Observation* obs) {
    this->model_->freeObs(this->obsEdge_);
    this->obsEdge_ = obs;
}

Belief* VNode::getBelief() const {
    return this->belief_->clone();
}

void VNode::setBelief(Belief* belief) {
    if (this->belief_ != nullptr) delete this->belief_;
    this->belief_ = belief;
}

bool VNode::isLeaf() const {
    return this->actChildren_.size() == 0;
}

const QNode* VNode::getParent() const {
    return parent_;
}

const std::vector<QNode*>& VNode::children() const {
    return this->actChildren_;
}

std::vector<QNode*>& VNode::children() {
    return this->actChildren_;
}

const QNode* VNode::child(Action action) const {
    return this->actChildren_[action];
}

QNode* VNode::child(Action action) {
    return this->actChildren_[action];
}

std::vector<ValuedAction> VNode::getValuedActions() const {
    std::vector<ValuedAction> valuedActions;
    for (int i = 0; i < this->actChildren_.size(); i++) {
        ValuedAction va(this->actChildren_[i]->getAction(),
                        this->actChildren_[i]->getValueObj(),
                        this->actChildren_[i]->getCount());
        valuedActions.push_back(va);
    }
    return valuedActions;
}

const QNode* VNode::maxChild() const {
    const QNode* maxChild = nullptr;
    double maxVal = Globals::NEG_INFTY;
    for (int i = 0; i < this->actChildren_.size(); i++) {
        double curActVal = this->actChildren_[i]->getTotalValue();
        if (curActVal > maxVal) {
            maxVal = curActVal;
            maxChild = this->actChildren_[i];
        }
    }
    return maxChild;
}

History VNode::maximumValueActionObservationSequence(const Action& action) const {
    History actObsSeq(this->model_);

    const QNode* qn = this->child(action);
    assert(qn != nullptr);
    while (!(qn->isLeaf())) {
        const VNode* vn = qn->maxChild();
        ValuedAction act(qn->getAction(), qn->getValueObj(), qn->getCount());
        Observation* obs = vn->getObservationObj();
        Belief* b = vn->getBelief();
        // add action-observation pair to history
        actObsSeq.add(act, obs, b);

        // go down the tree to the next (deeper) treelevel
        qn = vn->maxChild();
        assert(qn != nullptr);
    }

    return actObsSeq;
}

/* -------------------------------------------------------------------------- */
/*                                 QNode class                                */
/* -------------------------------------------------------------------------- */

QNode::QNode(const POMDP* model, VNode* parent, Action edge, int depth)
    : Node(model, depth), parent_(parent), actEdge_(edge) {
}

QNode::~QNode() {
    for (int i = 0; i < this->obsChildren_.size(); i++) {
        delete this->obsChildren_[i];
    }
}

Action QNode::getAction() const {
    return this->actEdge_;
}

const VNode* QNode::getParent() const {
    return parent_;
}

const std::vector<VNode*>& QNode::children() const {
    return this->obsChildren_;
}

std::vector<VNode*>& QNode::children() {
    return this->obsChildren_;
}

const VNode* QNode::maxChild() const {
    const VNode* maxChild = nullptr;
    double maxVal = Globals::NEG_INFTY;
    for (int i = 0; i < this->obsChildren_.size(); i++) {
        double curObsVal = this->obsChildren_[i]->getTotalValue();
        if (curObsVal > maxVal) {
            maxVal = curObsVal;
            maxChild = this->obsChildren_[i];
        }
    }

    return maxChild;
}

bool QNode::isLeaf() const {
    return this->obsChildren_.size() == 0;
}

}  // namespace ipft