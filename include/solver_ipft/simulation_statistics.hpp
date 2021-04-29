#pragma once

#include <chrono>
#include <vector>

#include "solver_ipft/core/solver.hpp"
#include "solver_ipft/interface/pomdp.hpp"
#include "solver_ipft/interface/world.hpp"

namespace solver_ipft {

class SimulationStatistics {
protected:
  POMDP *model_;
  World *world_;
  Solver *solver_;

  std::ostream *out_;

  std::chrono::time_point<std::chrono::high_resolution_clock> round_clock_start;

public:
  int step_count_;

  double total_discounted_reward_;
  double total_undiscounted_reward_;

  std::vector<double> state_rewards_;
  std::vector<double> obs_probs_;
  std::vector<SearchStatistics *> search_stats_;
  History *solver_hist_;
  std::vector<State *> world_hist_;

  double total_round_time; // in ms

public:
  SimulationStatistics(POMDP *model, World *world, Solver *solver,
                       std::ostream *out);

  virtual ~SimulationStatistics();

  SimulationStatistics(const SimulationStatistics &) = delete;
  SimulationStatistics(SimulationStatistics &&) = delete;
  SimulationStatistics &operator=(const SimulationStatistics &) = delete;
  SimulationStatistics &operator=(SimulationStatistics &&) = delete;

  void initRound(int round);

  void endRound(int step_count, History *solver_hist,
                const std::vector<State *> &world_hist);

  /**
   * @brief Saves step statistics and prints them out if specified
   *
   */
  void summarizeRewards(int round, int step, double reward);

  void initStep(int round, int step);

  void summarizeSearch();

  void summarizeExecuteAction(const Action &act, const Observation &obs,
                              const double &stateReward);

  void summarizeBeliefUpdate();

  void printRoundResultsTable(std::ostream &os) const;

protected:
  void printStateActionSequence() const;

  std::vector<std::string> getStepResultsForTable(int step) const;
};

} // namespace solver_ipft
