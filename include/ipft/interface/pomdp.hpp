#pragma once

#include <memory>

#include "ipft/core/globals.hpp"
#include "ipft/interface/spaces.hpp"

namespace ipft {
class Belief;

class POMDP {
   public:
    POMDP();
    virtual ~POMDP();

    /* -------------------------------------------------------------------------- */
    /*                    Simulative model and related functions                  */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief Returns a starting state of the simulation
     * Used to generate states in the initial belief or the true starting state of a POMDP-based world.
     * @return State* the starting state
     */
    virtual State* createStartState() const = 0;

    /**
     * @brief Returns the initial belief.
     *
     * @param type      type of the initial belief (e.g. Gaussian)
     * @return Belief*  the initial belief
     */
    virtual Belief* initialBelief(std::string type) const = 0;

    virtual State* transition(const State& state, const Action& action) const = 0;

    virtual Observation* observation(const State& statePosterior) const = 0;

    virtual double obsProb(const State& statePosterior, const Observation& obs) const = 0;

    virtual double maxPossibleWeight(const Action& act, const Observation& obs) const = 0;

    /**
     * @brief Returns the reward for taking an action at a state
     *
     * @param state     Current state of the world
     * @param action    Action to be taken
     * @return double   Reward for taking action in state
     */
    virtual double reward(const State& state, const Action& action) const = 0;

    virtual bool terminalState(const State& statePosterior, const Action& action) const = 0;

    /**
     * @brief Returns number of actions.
     * @return int Number of Actions
     */
    virtual int numActions() const = 0;  // TODO: Concept to check for continuous action space (MAX_INT ?)

    /**
     * @brief Function mapping an action(index) to the action value
     * Must be defined by the concrete model. ActionValue object is a Point
     * @param act the action index
     * @return ActionValue& the action value (concrete type must be defined with the concrete model)
     */
    virtual std::unique_ptr<ActionValue> valueOfAction(const Action& act) const = 0;

    /**
     * @brief Returns the (current) number of dimensions of the state space.
     * "current" means: the dimension of the states to be created next, if model does not change.
     * @return int number of dimensions of the state space
     */
    virtual int numDimStateSpace() const = 0;

    /**
     * @brief Returns states in the near of the State state. Used for particle reinvigoration.
     *
     * @param state the state around all the other states are distributed (the state is copied internally)
     * @param count the number of similar states to return (count > 0, if count = 1 then the set of similar states only
     * contains the parameter state)
     * @return std::vector<State*> the set of similar states including the parameter state
     */
    virtual std::vector<State*> similarStates(const State& state, int count) const = 0;

    virtual void newParticle(State* particle, const std::vector<State*>& particleSet, const Action& act, const Observation& obs) const = 0;

    /**
     * [Optional] default returns empty vector
     * @brief Returns an ordered vector with preferred actions.
     * No duplicates! Each action can be only once in the set of preferred actions.
     * *All actions in this set will be initialized with a small positive count and value!
     * (refers to the action nodes in the search tree)
     * @param belief the current belief (on which preferred actions can be based on)
     * @return std::vector<Action> ordered, preferred actions
     */
    virtual std::vector<Action> preferredActions(const Belief* belief) const;

    /**
     * [Optional] default returns empty vector.
     * @brief Returns an ordered vector of all legal actions.
     * No duplicates! Each action can be only once in the set of preferred actions.
     *
     * @param belief
     * @return std::vector<Action>
     */
    virtual std::vector<Action> legalActions(const Belief* belief) const;

    /* -------------------------------------------------------------------------- */
    /*                              Display functions                             */
    /* -------------------------------------------------------------------------- */

    virtual std::string to_string(const State& state) const = 0;

    virtual std::string to_string(const Observation& obs) const = 0;

    virtual std::string to_string(const Action& action) const = 0;

    virtual std::string to_string(const Belief& belief) const = 0;

    /* -------------------------------------------------------------------------- */
    /*                              Memory management                             */
    /* -------------------------------------------------------------------------- */

    // TODO: find a better solution (a Creational Pattern)

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

}  // namespace ipft