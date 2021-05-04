#pragma once

#include <solver_ipft/plannerbase.hpp>

namespace solver_ipft {

class Planner : public Plannerbase {
protected:
  int step_;
  int round_;

  std::unique_ptr<SimulationStatistics> sim_stats_;

public:
  Planner();

  ~Planner() override = default;

  Planner(const Planner &) = delete;
  Planner(Planner &&) = delete;
  Planner &operator=(const Planner &) = delete;
  Planner &operator=(Planner &&) = delete;

  /**
   * @brief Perform one search-execute-update step
   *
   */
  virtual bool runStep();

  virtual void initializePlanner();

  virtual void runPlanningLoop();

  virtual void resetPlanner();

  /**
   * @brief returns the simulation statistics and resets the statistics in the
   * planner
   *
   * @return SimulationStatistics the simulation statistics object
   */
  virtual std::unique_ptr<SimulationStatistics> getSimulationStatistics();

  /**
   * @brief Run and evaluate POMDP planning for a given number of rounds
   *
   */
  virtual int runPlanning(int argc, char *argv[]);

protected:
  /**
   * @brief Loop the search-execute-update process for a given number of steps
   *
   */
  virtual void planningLoop();

  virtual ValuedAction findAction();

  virtual bool takeAction(const ValuedAction &valuedAct);
};

} // namespace solver_ipft