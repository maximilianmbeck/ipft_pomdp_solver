#pragma once

#include <iostream>
#include <ipft/core/history.hpp>
#include <ipft/core/node.hpp>
#include <ipft/core/valued_action.hpp>
#include <ipft/interface/belief.hpp>
#include <ipft/interface/pomdp.hpp>
#include <ipft/interface/spaces.hpp>
namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                         SearchStatistics interface                         */
/* -------------------------------------------------------------------------- */

class SearchStatistics {
   public:
    ValuedAction optimalAction;  // contains optimal action + optimal value
    std::vector<ValuedAction> valuedActions;
    SearchStatistics() {
    }
    virtual ~SearchStatistics() {
    }
    virtual std::string text() const = 0;
};

/* -------------------------------------------------------------------------- */
/*                               Value interface                              */
/* -------------------------------------------------------------------------- */

class Value {
   public:
    Value() {
    }
    Value(double v) {
    }
    virtual ~Value() {
    }

    virtual void add(const Value& val) = 0;
    virtual void update(const Value& val, int count) = 0;
    virtual void set(const Value& val) = 0;
    virtual void setComponent(int index, const double& val) = 0;
    virtual double getRawComponent(int index) const = 0;
    virtual double getWeightedComponent(int index) const = 0;
    virtual int getComponentCount() const = 0;
    virtual std::string text() const = 0;

    virtual double total() const = 0;
    virtual Value* clone() const = 0;

    friend std::ostream& operator<<(std::ostream& os, const Value& v);
};

/* -------------------------------------------------------------------------- */
/*                                Solver class                                */
/* -------------------------------------------------------------------------- */

// forward declarations
class VNode;

class Solver {
   protected:
    const POMDP* model_;
    Belief* belief_;
    VNode* root_;
    History* history_;  // contains only the action, not the action value

   public:
    Solver(const POMDP* model, Belief* belief);

    virtual ~Solver();

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
     * @brief Returns a pointer to the root of the search tree, memory is still managed by the solver
     * 
     * @return const VNode* the root of the search tree
     */
    const VNode* getSearchTree() const;

    /**
     * @brief
     * Update current belief, history, and any other internal states that is
     * needed for Search() to function correctly.
     *
     * @param action selected by the previous search
     * @param obs the observation received from the environment
     */
    virtual void beliefUpdate(const Action& action, const Observation& obs) = 0;

    /**
     * @brief Set the Belief object
     * Set initial belief for planning. Make sure internal states associated with
     * initial belief are reset. In particular, history need to be cleaned, and
     * allocated memory from previous searches need to be cleaned if not.
     * @param b
     */
    virtual void setBelief(Belief* b) = 0;
    virtual Belief* getBelief() const = 0;

    /**
     * @brief Returns the search statistics
     *
     * @return SearchStatistics* the search statistics object (parent type, exact type is specified at runtime by the
     * respective solver)
     */
    virtual SearchStatistics* getSearchStatistics() const = 0;

    virtual History* copyHistory() const;
};

}  // namespace solver_ipft