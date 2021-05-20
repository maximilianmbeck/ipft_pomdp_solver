#pragma once

#include <solver_ipft/interface/spaces.hpp>

namespace solver_ipft {

class World {

public:
    World() = default;
    virtual ~World() = default;

    World(const World&) = delete;
    World(World&&) = delete;
    World& operator=(const World&) = delete;
    World& operator=(World&&) = delete;

public:
    /**
     * @brief Establish connection to simulator or system
     */
    virtual bool connect() = 0;

    /**
     * @brief Initialize or reset the (simulation) environment, return the start
     * state if applicable
     *
     * @return State* the initial state
     */
    virtual State* initialize() = 0;

    /**
     * @brief To help to construct initial belief and to print debug informations
     * in Logger
     *
     * @return State* the current state
     */
    virtual State* getCurrentState() const = 0;

    virtual void setState(State* state) = 0;

    /**
     * @brief send action, receive reward, observation and terminal
     *
     * @param action Action to be executed in the real-world system
     * @param obs Observation sent back from the real-world system
     * @return action executed successful
     */
    virtual bool executeAction(const Action& action, Observation*& obs) = 0;

    virtual double getReward() const = 0;

    virtual std::vector<State*> copyWorldStateSequence() const = 0;
};

} // namespace solver_ipft