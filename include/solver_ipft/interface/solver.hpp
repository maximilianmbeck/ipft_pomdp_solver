#pragma once

#include <iostream>
#include <memory>
#include <solver_ipft/core/history.hpp>
#include <solver_ipft/core/node.hpp>
#include <solver_ipft/core/valued_action.hpp>
#include <solver_ipft/interface/belief.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/spaces.hpp>
namespace solver_ipft {

class SearchStatistics {
public:
    std::shared_ptr<POMDP> model;

    // store both optimal action and optimal value with ValuedAction
    ValuedAction optimalAction;
    std::vector<ValuedAction> valuedActions;
    explicit SearchStatistics(std::shared_ptr<POMDP> model) : model(std::move(model)) {
    }

    virtual ~SearchStatistics() = default;
    virtual std::string text() const = 0;
};


class Value {
public:
    Value() = default;
    virtual ~Value() = default;

    Value(const Value&) = default;
    Value(Value&&) = default;
    Value& operator=(const Value&) = default;
    Value& operator=(Value&&) = default;

    virtual void add(const Value& val) = 0;
    virtual void update(const Value& val, int count) = 0;
    virtual void set(const Value& val) = 0;
    virtual void setComponent(int index, const double& val) = 0;
    virtual double getRawComponent(int index) const = 0;
    virtual double getWeightedComponent(int index) const = 0;
    virtual int getComponentCount() const = 0;
    virtual std::string text() const = 0;

    virtual double total() const = 0;
    virtual std::unique_ptr<Value> clone() const = 0;

    friend std::ostream& operator<<(std::ostream& os, const Value& v);
};


// forward declarations
class VNode;

class Solver {
protected:
    std::shared_ptr<POMDP> model_;
    std::unique_ptr<Belief> belief_;
    std::shared_ptr<VNode> root_;
    std::unique_ptr<History> history_; // contains only the action, not the action value

public:
    Solver(std::shared_ptr<POMDP> model, std::unique_ptr<Belief>&& belief);
    explicit Solver(std::shared_ptr<POMDP> model);

    virtual ~Solver() = default;

    Solver(const Solver&) = delete;
    Solver(Solver&&) = delete;
    Solver& operator=(const Solver&) = delete;
    Solver& operator=(Solver&&) = delete;

    /**
     * @brief
     * Find the optimal action for current belief, and optionally return the
     * found value for the action. Return the value Globals::NEG_INFTY if the
     * value is not to be used.
     *
     * @return ValuedAction
     */
    virtual ValuedAction search() = 0;

    /**
     * @brief Returns a pointer to the root of the search tree
     *
     * @return the root of the search tree
     */
    std::shared_ptr<VNode> getSearchTree() const;

    /**
     * @brief
     * Update current belief and history
     *
     * @param action the action selected by the previous search
     * @param obs the observation received from the environment
     */
    virtual void beliefUpdate(const Action& action, const Observation& obs) = 0;

    /**
     * @brief Reset the Belief
     * Belief are reset to the new belief. History is reset: Action,
     * Observation and Belief history are cleared.
     * @param b the new belief
     */
    virtual void resetBelief(std::unique_ptr<Belief>&& b);

    /**
     * @brief Set the Belief to a new belief ("weak" version of "resetBelief")
     * History is not reset. Only the current belief is changed.
     * @param b the new belief
     */
    virtual void setBelief(std::unique_ptr<Belief>&& b);
    virtual const Belief* getBelief() const;
    virtual std::unique_ptr<Belief> copyBelief() const;

    /**
     * @brief Returns the search statistics and resets statistics in the solver
     *
     * @return SearchStatistics the search statistics object
     */
    virtual std::unique_ptr<SearchStatistics> getSearchStatistics() = 0;

    /**
     * @brief returns a copy of the History object
     *
     * @return copy of the History object
     */
    virtual History copyHistory() const;
};

} // namespace solver_ipft