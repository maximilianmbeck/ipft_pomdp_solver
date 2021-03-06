#pragma once

#include <solver_ipft/core/valued_action.hpp>
#include <solver_ipft/interface/belief.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/spaces.hpp>

namespace solver_ipft {

/**
 * @brief A class representing the action-observation history of a solver.
 * the history looks as follows: b0, a0, o0, b1, a1, o1, b2, ...
 * i.e. due to the initial belief there is always one belief more in the history
 */
class History {
private:
    std::shared_ptr<POMDP> model_;
    std::vector<ValuedAction> actions_; // Actions are discrete
    std::vector<Observation*> observations_;
    std::vector<std::unique_ptr<Belief>> beliefs_;

public:
    History();
    explicit History(std::shared_ptr<POMDP> model);
    virtual ~History();
    // copy constructor
    History(const History& other);
    // assignment operator
    History& operator=(const History& rhs);
    // move copyconstructor
    History(History&& other) noexcept;
    // move assignment operator
    History& operator=(History&& rhs) noexcept;

    void add(Action action, Observation* obs, std::unique_ptr<Belief>&& b);
    void add(const ValuedAction& valAction, Observation* obs, std::unique_ptr<Belief>&& b);
    void addInitialBelief(std::unique_ptr<Belief>&& b);
    void RemoveLast();
    Action action(int t) const;
    ValuedAction valuedAction(int t) const;
    Observation* observation(int t) const;
    const Observation* observationPointer(int t) const;
    std::unique_ptr<Belief> belief(int t) const;
    const Belief* beliefPointer(int t) const;
    size_t size() const;
    /**
     * @brief Removes/truncates all action-observation tuples after d timesteps.
     *
     * @param d the depth / number of timesteps
     */
    void truncate(int d);
    Action lastAction() const;
    ValuedAction lastValuedAction() const;
    Observation* lastObservation() const;
    std::unique_ptr<Belief> lastBelief() const;
    /**
     * @brief Returns the history with action-observation tuples from timestep s
     * to the end.
     *
     * @param s the action-observation tupes before s will be truncated
     * @return History the new history
     */
    History suffix(int s) const;

    /* ----------------------------- printer methods ----------------------------
     */

    friend std::ostream& operator<<(std::ostream& os, const History& history);
    std::string shortDescription() const;
    std::string text() const;
};

} // namespace solver_ipft