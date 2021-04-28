#pragma once

#include <chrono>

#include "solver_ipft/planner.hpp"

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                         Evaluation statistics class                        */
/* -------------------------------------------------------------------------- */

class EvaluationStatistics {
protected:
  std::ostream *out_;

public:
  int round_count_;

  // std::vector<SimulationStatistics*> round_results_;
  std::vector<int> step_counts_;
  std::vector<double> total_disc_rewards_;
  std::vector<double> total_undisc_rewards_;

  double total_eval_time; // in seconds

  std::chrono::time_point<std::chrono::high_resolution_clock> eval_clock_start;

public:
  explicit EvaluationStatistics(std::ostream *out);

  void initEvaluation();

  void summarizeRound(SimulationStatistics *round_stats);

  void endEvaluation();

  void printRewardStatistics(std::ostream &out) const;

  void averageRoundReward(double &discountedAvg, double &undiscountedAvg) const;

  void stderrRoundReward(double &discountedStderr,
                         double &undiscountedStderr) const;

  virtual void writeEvalutionResultsToFile(const std::string &filename) const;

  virtual void printEvaluationResults(std::ostream &os) const;
};

/* -------------------------------------------------------------------------- */
/*                               Evaluator class                              */
/* -------------------------------------------------------------------------- */

class Evaluator {
protected:
  Planner *planner_;

  EvaluationStatistics *eval_stats_;

  int round_;

public:
  explicit Evaluator(Planner *planner);
  virtual ~Evaluator();

  virtual int runEvaluation(int argc, char *argv[]);

protected:
  virtual void runEvaluationLoop();

  virtual void printEvaluationEnd(
      int num_rounds,
      const std::chrono::time_point<std::chrono::high_resolution_clock>
          &main_clock_start) const;

  virtual void saveEvaluationResults() const;
};

} // namespace solver_ipft
