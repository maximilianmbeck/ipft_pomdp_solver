#pragma once

#include "solver_ipft/core/solver.hpp"
#include "solver_ipft/interface/pomdp.hpp"
#include "solver_ipft/interface/world.hpp"

namespace solver_ipft {

class POMDPWorld : public World {
   protected:
    const POMDP* model_;
    std::vector<State*> stateSequence_;

    double stepReward_;  // reward of the last step

   public:
    POMDPWorld(const POMDP* model);

    virtual ~POMDPWorld();

    // establish connection to simulator or system
    bool connect() override;
    // initialize or reset the simulation environment
    State* initialize() override;
    // helps to construct initial belief or to print debug information
    State* getCurrentState() const override;
    void setState(State* state) override;
    // send action, receive reward, obs and terminal
    bool executeAction(const Action& action, Observation*& obs) override;
    // receive reward from the last action
    double getReward() const override;

    std::vector<State*> copyWorldStateSequence() const override;
};

}  // namespace solver_ipft