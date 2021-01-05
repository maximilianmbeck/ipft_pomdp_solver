#pragma once

#include "ipft/core/valued_action.hpp"
#include "ipft/interface/belief.hpp"
#include "ipft/interface/pomdp.hpp"
#include "ipft/interface/spaces.hpp"

namespace ipft {

/**
 * @brief A class representing the action-observation history of a solver.
 * the history looks as follows: b0, a0, o0, b1, a1, o1, b2, ... 
 * i.e. due to the initial belief there is always one belief more in the history
 */
class History {
   private:
    const POMDP* model_;
    std::vector<ValuedAction> actions_;  // Actions are discrete
    std::vector<Observation*> observations_;
    std::vector<Belief*> beliefs_;

   public:
    History();
    History(const POMDP* model);
    virtual ~History();
    // copy constructor
    History(const History& other);
    // assignment operator
    History& operator=(const History& rhs);
    // move copyconstructor
    History(History&& other) noexcept;
    // move assignment operator
    History& operator=(History&& rhs) noexcept;

    void add(Action action, Observation* obs, Belief* b);
    void add(const ValuedAction& valAction, Observation* obs, Belief* b);
    void addInitialBelief(Belief* b);
    void RemoveLast();
    Action action(int t) const;
    ValuedAction valuedAction(int t) const;
    Observation* observation(int t) const;
    const Observation* observationPointer(int t) const;
    Belief* belief(int t) const;
    const Belief* beliefPointer(int t) const;
    size_t
    size() const;
    /**
     * @brief Removes/truncates all action-observation tuples after d timesteps.
     *
     * @param d
     */
    void truncate(int d);
    Action lastAction() const;
    ValuedAction lastValuedAction() const;
    Observation* lastObservation() const;
    Belief* lastBelief() const;
    /**
     * @brief Returns the history with action-observation tuples from timestep s to the end.
     *
     * @param s the action-observation tupes before s will be truncated
     * @return History the new history
     */
    History suffix(int s) const;

    /* ----------------------------- printer methods ---------------------------- */

    friend std::ostream& operator<<(std::ostream& os, const History& history);
    std::string shortDescription() const;
    std::string text() const;
};

}  // namespace ipft