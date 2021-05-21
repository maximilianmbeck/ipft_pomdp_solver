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

/**
 * @brief Interface for saving statistics of a single search step.
 */
class SearchStatistics {
public:
    // include to call toString() alike methods
    std::shared_ptr<POMDP> model;

    // store both optimal action and optimal value with ValuedAction
    ValuedAction optimalAction;
    std::vector<ValuedAction> valuedActions;
    explicit SearchStatistics(std::shared_ptr<POMDP> model) : model(std::move(model)) {
    }

    virtual ~SearchStatistics() = default;
    virtual std::string text() const = 0;
};

/**
 * @brief Interface for value vectors.
 */
class Value {
public:
    Value() = default;
    virtual ~Value() = default;

    Value(const Value&) = default;
    Value(Value&&) = default;
    Value& operator=(const Value&) = default;
    Value& operator=(Value&&) = default;

    /**
     * @brief Add another value to the current Value
     * @param val
     */
    virtual void add(const Value& val) = 0;

    /**
     * @brief Update value and count
     * @param val The value to be added (R in IPFT-paper in Alg. 1, L22)
     * @param count The number of value updates (N(ha) in IPFT-paper in Alg. 1, L22)
     */
    virtual void update(const Value& val, int count) = 0;

    /**
     * @brief Set a new value
     * @param val
     */
    virtual void set(const Value& val) = 0;

    /**
     * @brief Sets only a single value component
     * @param index
     * @param val
     */
    virtual void setComponent(int index, const double& val) = 0;

    /**
     * @brief Get the Component
     * Return all the (raw/unweighted) values
     * @param index
     * @return double
     */
    virtual double getComponent(int index) const = 0;

    /**
     * @brief Get the Weighted Component
     * Some Value components might be weighted. This method returns all the values by multiplying with their weights, if
     * defined any.
     * @param index
     * @return double
     */
    virtual double getWeightedComponent(int index) const = 0;

    /**
     * @brief Get the number of components in Value
     * @return int
     */
    virtual int getComponentCount() const = 0;

    /**
     * @brief Returns string representation for information output.
     * @return std::string
     */
    virtual std::string text() const = 0;

    /**
     * @brief Return total value (cumulative sum)
     * @return double
     */
    virtual double total() const = 0;

    /**
     * @brief Perform a deepcopy of the Value.
     * @return std::unique_ptr<Value>
     */
    virtual std::unique_ptr<Value> clone() const = 0;

    /**
     * @brief Output operator.
     * @return std::ostream&
     */
    friend std::ostream& operator<<(std::ostream& os, const Value& v);
};


// forward declarations
class VNode;

/**
 * @brief Interface for solver.
 */
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
     * @brief Find the optimal action for current belief
     *
     * @return ValuedAction The estimated value of the action.
     */
    virtual ValuedAction search() = 0;

    /**
     * @brief Returns a pointer to the root of the search tree
     *
     * @return the root of the search tree
     */
    std::shared_ptr<VNode> getSearchTree() const;

    /**
     * @brief Update current belief and history
     *
     * @param action prior action
     * @param obs current observation
     */
    virtual void beliefUpdate(const Action& action, const Observation& obs) = 0;

    /**
     * @brief Reset the Belief to a new belief.
     * During this, History is reset: Action, Observation and Belief history are cleared.
     *
     * Used at the end of a simulation run.
     * @param b the new belief
     */
    virtual void resetBelief(std::unique_ptr<Belief>&& b);

    /**
     * @brief Set the Belief to a new belief ("weak" version of "resetBelief").
     * During this, History is NOT reset. Only the current belief is changed.
     *
     * In some cases, the environment does not change but the belief needs to be modified.
     * In such, call getBelief(), modify it, and then setBelief().
     * @param b the new belief
     */
    virtual void setBelief(std::unique_ptr<Belief>&& b);

    /**
     * @brief Get an unmodifiable pointer to the Belief.
     *
     * @return const Belief*
     */
    virtual const Belief* getBelief() const;

    /**
     * @brief Perform a deepcopy of the Belief.
     *
     * @return std::unique_ptr<Belief>
     */
    virtual std::unique_ptr<Belief> copyBelief() const;

    /**
     * @brief Returns the search statistics and resets statistics in the solver.
     *
     * @return SearchStatistics the search statistics object.
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