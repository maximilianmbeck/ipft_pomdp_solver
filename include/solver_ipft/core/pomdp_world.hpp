#pragma once

#include <solver_ipft/core/solver.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/world.hpp>

namespace solver_ipft {

class POMDPWorld : public World {
protected:
  State *state_{nullptr};
  std::shared_ptr<POMDP> model_;
  std::vector<State *> stateSequence_;

  double stepReward_; // reward of the last step

public:
  explicit POMDPWorld(std::shared_ptr<POMDP> model);

  ~POMDPWorld() override;

  POMDPWorld(const POMDPWorld &) = delete;
  POMDPWorld(POMDPWorld &&) = delete;
  POMDPWorld &operator=(const POMDPWorld &) = delete;
  POMDPWorld &operator=(POMDPWorld &&) = delete;

  // establish connection to simulator or system
  bool connect() override;
  // initialize or reset the simulation environment
  State *initialize() override;
  // helps to construct initial belief or to print debug information
  State *getCurrentState() const override;
  void setState(State *state) override;
  // send action, receive reward, obs and terminal
  bool executeAction(const Action &action, Observation *&obs) override;
  // receive reward from the last action
  double getReward() const override;

  std::vector<State *> copyWorldStateSequence() const override;
};

} // namespace solver_ipft