#pragma once

#include <solver_ipft/core/solver.hpp>
#include <solver_ipft/interface/belief.hpp>
#include <solver_ipft/interface/spaces.hpp>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                                Node classes                                */
/* -------------------------------------------------------------------------- */

class Node {
public:
  const POMDP *model_;

protected:
  int count_;           // number of visits on the node
  const int treelevel_; // depth / level of node in the tree
  Value *value_;        // value of the node
public:
  Node(const POMDP *model, int level)
      : model_(model), treelevel_(level), count_(0), value_(nullptr) {}
  virtual ~Node();
  Node(const Node &other) = delete;
  Node(Node &&other) noexcept = delete;
  Node &operator=(const Node &) = delete;
  Node &operator=(Node &&) = delete;

  virtual void setCount(int count);
  virtual int getCount() const;
  virtual int getTreelevel() const;
  virtual void setValue(Value *value);
  virtual Value *getValueObj() const;
  virtual const Value *getValueRef() const;
  virtual double getTotalValue() const;
  virtual void updateValueCount(const Value &v);
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
  Belief *belief_;
  Observation *obsEdge_; // the input edge to this VNode
  std::vector<Belief *>
      belief_archive_; // holds all previousely sampled beliefs in this node
                       // (first sampled -> index 0)
  std::vector<Observation *>
      obs_archive_; // holds all previously sampled observations for this node
                    // (first sampled -> index 0)

protected:
  QNode *parent_;
  std::vector<QNode *> actChildren_;

public:
  VNode(const POMDP *model, QNode *parent, Observation *obs, Belief *belief,
        int level);
  ~VNode() override;

  VNode(const VNode &) = delete;
  VNode(VNode &&) = delete;
  VNode &operator=(const VNode &) = delete;
  VNode &operator=(VNode &&) = delete;

  // Return a pointer to a copy of the observation edge leading to this vnode
  Observation *getObservationObj() const;
  // Clear old obs and set to new one
  void setObs(Observation *obs);
  // Return a pointer to a copy of the belief
  Belief *getBelief() const;
  // Clear old belief and set to new one
  void setBelief(Belief *belief);
  bool isLeaf() const;

  const QNode *getParent() const;
  const std::vector<QNode *> &children() const;
  std::vector<QNode *> &children();
  const QNode *child(Action action) const;
  QNode *child(Action action);
  const QNode *maxChild() const;
  History maximumValueActionObservationSequence(const Action &action) const;

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
  VNode *parent_;
  Action actEdge_; // the input edge to this QNode
  std::vector<VNode *> obsChildren_;

public:
  QNode(const POMDP *model, VNode *parent, Action edge, int level);
  ~QNode() override;

  QNode(const QNode &) = delete;
  QNode(QNode &&) = delete;
  QNode &operator=(const QNode &) = delete;
  QNode &operator=(QNode &&) = delete;

  Action getAction() const;

  const VNode *getParent() const;
  const std::vector<VNode *> &children() const;
  std::vector<VNode *> &children();

  const VNode *maxChild() const;
  bool isLeaf() const;
};

} // namespace solver_ipft