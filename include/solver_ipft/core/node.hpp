#pragma once

#include <memory>
#include <solver_ipft/core/solver.hpp>
#include <solver_ipft/interface/belief.hpp>
#include <solver_ipft/interface/spaces.hpp>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                                Node classes                                */
/* -------------------------------------------------------------------------- */

class Node {
public:
    std::shared_ptr<POMDP> model_;

protected:
    int count_;                    // number of visits on the node
    const int treelevel_;          // depth / level of node in the tree
    std::unique_ptr<Value> value_; // value of the node
public:
    Node(std::shared_ptr<POMDP> model, int level)
            : model_(std::move(model)), treelevel_(level), count_(0), value_(nullptr) {
    }
    virtual ~Node() = default;
    Node(const Node& other) = delete;
    Node(Node&& other) noexcept = delete;
    Node& operator=(const Node&) = delete;
    Node& operator=(Node&&) = delete;

    virtual void setCount(int count);
    virtual int getCount() const;
    virtual int getTreelevel() const;
    virtual void setValue(std::unique_ptr<Value>&& value);
    virtual std::unique_ptr<Value> cloneValue() const;
    virtual const Value* getValue() const;
    virtual double getTotalValue() const;
    virtual void updateValueCount(const Value& v);
};

// forward declarations
class QNode;

/**
 * @brief A belief-node/V-node/OR-node in the search tree.
 * (see Ross, Pineau "Online Planning Algorithms for POMDPs")
 * The incoming edge is an observation.
 * The outgoing edges are actions (going to QNodes).
 * At these nodes an action must be chosen.
 */
class VNode : public Node {
public:
    std::unique_ptr<Belief> belief_;
    Observation* obsEdge_;                                // the input edge to this VNode
    std::vector<std::unique_ptr<Belief>> belief_archive_; // holds all previousely sampled beliefs in this node
                                                          // (first sampled -> index 0)
    std::vector<Observation*> obs_archive_;               // holds all previously sampled observations for this node
                                                          // (first sampled -> index 0)

protected:
    std::shared_ptr<QNode> parent_;
    mutable std::vector<std::shared_ptr<QNode>> actChildren_;

public:
    VNode(std::shared_ptr<POMDP> model,
          std::shared_ptr<QNode> parent,
          Observation* obs,
          std::unique_ptr<Belief>&& belief,
          int level);
    ~VNode() override;

    VNode(const VNode&) = delete;
    VNode(VNode&&) = delete;
    VNode& operator=(const VNode&) = delete;
    VNode& operator=(VNode&&) = delete;

    // Return a pointer to a copy of the observation edge leading to this vnode
    Observation* getObservationObj() const;
    // Clear old obs and set to new one
    void setObs(Observation* obs);
    // Return a pointer to a copy of the belief
    std::unique_ptr<Belief> getBelief() const;
    // Clear old belief and set to new one
    void setBelief(std::unique_ptr<Belief>&& belief);
    bool isLeaf() const;

    std::shared_ptr<QNode> getParent() const;
    std::vector<std::shared_ptr<QNode>>& children() const;
    std::shared_ptr<QNode> child(Action action) const;
    std::shared_ptr<QNode> maxChild() const;
    History maximumValueActionObservationSequence(const Action& action) const;

    std::vector<ValuedAction> getValuedActions() const;
};

/**
 * @brief A Q-node/AND-node in the search tree.
 * (see Ross, Pineau "Online Planning Algorithms for POMDPs")
 * The incoming edge is an action.
 * The outgoing edges are observations (going to VNodes).
 * At these nodes all possible observations that lead to subsequent beliefs must
 * be considered.
 */
class QNode : public Node {
protected:
    std::shared_ptr<VNode> parent_;
    Action actEdge_; // the input edge to this QNode
    mutable std::vector<std::shared_ptr<VNode>> obsChildren_;

public:
    QNode(std::shared_ptr<POMDP> model, std::shared_ptr<VNode> parent, Action edge, int level);
    ~QNode() override;

    QNode(const QNode&) = delete;
    QNode(QNode&&) = delete;
    QNode& operator=(const QNode&) = delete;
    QNode& operator=(QNode&&) = delete;

    Action getAction() const;

    std::shared_ptr<VNode> getParent() const;
    std::vector<std::shared_ptr<VNode>>& children() const;

    std::shared_ptr<VNode> maxChild() const;
    bool isLeaf() const;
};

} // namespace solver_ipft