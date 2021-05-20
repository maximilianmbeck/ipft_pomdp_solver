#pragma once

#include <memory>

#include <solver_ipft/core/globals.hpp>
#include <solver_ipft/interface/spaces.hpp>

namespace solver_ipft {
class Belief;

class POMDP : public std::enable_shared_from_this<POMDP> {
public:
    POMDP() = default;
    virtual ~POMDP() = default;

    POMDP(const POMDP&) = delete;
    POMDP(POMDP&&) = delete;
    POMDP& operator=(const POMDP&) = delete;
    POMDP& operator=(POMDP&&) = delete;

    /* --------------------------------------------------------------------------
     */
    /*                   Simulative model and related functions */
    /* --------------------------------------------------------------------------
     */

    /**
     * @brief Returns a starting state of the simulation
     * Used to generate states in the initial belief or the true starting state of
     * a POMDP-based world.
     * @return State* the starting state
     */
    virtual State* createStartState() const;

    /**
     * @brief Returns the initial belief.
     *
     * @param type      type of the initial belief (e.g. Gaussian)
     * @return Belief*  the initial belief
     */
    virtual std::unique_ptr<Belief> initialBelief(const std::string& type);

    /**
     * @brief Transition model of the pomdp.
     *
     * @param state     Prior state
     * @param action    Action applied in prior state
     * @return State*   Generated posterior state
     */
    virtual State* transition(const State& state, const Action& action) const = 0;

    /**
     * @brief Observation model of the pomdp.
     *  Depends only on the posterior state
     * @param statePosterior    Posterior state
     * @return Observation*     Generated observation
     */
    virtual Observation* observation(const State& statePosterior) const = 0;

    /**
     * @brief Observation probability factor used for particle weighting.
     * The result is not a probability; it needs to be normalized.
     * @param statePosterior
     * @param obs               Observation
     * @return double           Observation probability factor
     */
    virtual double obsProb(const State& statePosterior, const Observation& obs) const;

    /**
     * @brief Observation probability factor used for particle weighting.
     * The result is not a probability; it needs to be normalized.
     * @param state             Prior state
     * @param action            Prior action
     * @param statePosterior    Posterior state
     * @param obs               Observation
     * @return double           Observation probability factor
     */
    virtual double obsProb(const State& state,
                           const Action& action,
                           const State& statePosterior,
                           const Observation& obs) const;

    /**
     * @brief Returns the maximum value of the observation probability distribution.
     * Returns the maximum value obsProb can return.
     * @param action            Prior action
     * @param obs               Observation
     * @return double           Maximum observation probability factor
     */
    virtual double maxPossibleWeight(const Action& action, const Observation& obs) const;

    /**
     * @brief Returns the reward for taking an action at a state
     *
     * @param state     Prior state
     * @param action    Prior action
     * @return double   Reward for taking action in state
     */
    virtual double reward(const State& state, const Action& action) const;

    /**
     * @brief Returns the reward for taking an action at state and transition to state posterior.
     * Can consider the reward associated with statePosterior as well, based on structure of the pomdp model.
     * @param state             Prior state
     * @param action            Prior action
     * @param statePosterior    Posterior state
     * @return double           Reward for taking action in prior state and transition to posterior state
     */
    virtual double reward(const State& state, const Action& action, const State& statePosterior) const;

    /**
     * @brief Check if prior action leads to terminal state.
     *
     * @param action            Prior action
     * @param statePosterior    Posterior state
     * @return true             Terminal state reached
     * @return false            Terminal state not reached
     */
    virtual bool terminalState(const Action& action, const State& statePosterior) const = 0;

    /**
     * @brief Returns number of actions.
     * @return int Number of Actions
     */
    virtual int numActions() const = 0;

    /**
     * @brief Function mapping an action(index) to the action value.
     * Must be defined by the concrete model. ActionValue object is a Point
     * @param act the action index
     * @return ActionValue& the action value (concrete type must be defined with the concrete model)
     */
    virtual std::unique_ptr<ActionValue> valueOfAction(const Action& act) const;

    /**
     * @brief Returns the (current) number of dimensions of the state space.
     * "current" means: the dimension of the states to be created next.
     * @return int number of dimensions of the state space
     */
    virtual int numDimStateSpace() const;

    /**
     * @brief Returns states in the vicinity of the state.
     * Used for particle reinvigoration.
     * @param state the state around all the other states are distributed (the state is copied internally)
     * @param count the number of similar states to return (count > 0).
     * @return std::vector<State*> the set of similar states including the parameter state
     */
    virtual std::vector<State*> similarStates(const State& state, int count) const;

    /**
     * @brief Replaces @param particle with a resampled particle in the vicinity of the current observation @param obs
     * Used for particle reinvigoration in ObsAdaptiveReinvigorator.
     * @param particle the particle to replace
     * @param particleSet the unweighted / resampled particle set --> all weights must be 1/N_particles
     * @param act current action
     * @param obs current observation
     */
    virtual void newParticle(State* particle,
                             const std::vector<State*>& particleSet,
                             const Action& act,
                             const Observation& obs) const;

    /**
     * [Optional] default returns empty vector
     * @brief Returns an ordered vector with preferred actions.
     * No duplicates! Each action can be only once in the set of preferred
     * actions. *All actions in this set will be initialized with a small positive
     * count and value! (refers to the action nodes in the search tree)
     * @param belief the current belief (on which preferred actions can be based
     * on)
     * @return std::vector<Action> ordered, preferred actions
     */
    virtual std::vector<Action> preferredActions(const Belief* belief) const;

    /**
     * [Optional] default returns empty vector.
     * @brief Returns an ordered vector of all legal actions.
     * No duplicates! Each action can be only once in the set of preferred
     * actions.
     *
     * @param belief
     * @return std::vector<Action>
     */
    virtual std::vector<Action> legalActions(const Belief* belief) const;

    /* --------------------------------------------------------------------------
     */
    /*                              Display functions */
    /* --------------------------------------------------------------------------
     */

    virtual std::string to_string(const State* state) const = 0;

    virtual std::string to_string(const Observation* obs) const = 0;

    virtual std::string to_string(const Action& action) const = 0;

    virtual std::string to_string(const Belief* belief) const = 0;

    /* --------------------------------------------------------------------------
     */
    /*                              Memory management */
    /* --------------------------------------------------------------------------
     */

    virtual Observation* allocateObs() const = 0;

    virtual State* allocateState() const = 0;

    virtual Observation* copyObs(const Observation* obs) const = 0;

    virtual std::vector<Observation*> copyObss(const std::vector<Observation*>& obss) const;

    virtual State* copyState(const State* state) const = 0;

    virtual std::vector<State*> copyStates(const std::vector<State*>& states) const;

    virtual void freeObs(Observation* obs) const = 0;

    virtual void freeObss(const std::vector<Observation*>& obss) const;

    virtual void freeState(State* state) const = 0;

    virtual void freeStates(const std::vector<State*>& states) const;

    virtual int numActiveObs() const = 0;

    virtual int numActiveStates() const = 0;
};

} // namespace solver_ipft