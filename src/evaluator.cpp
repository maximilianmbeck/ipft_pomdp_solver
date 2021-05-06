#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <solver_ipft/evaluator.hpp>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                         Evaluation statistics class                        */
/* -------------------------------------------------------------------------- */

EvaluationStatistics::EvaluationStatistics(std::ostream *out) : out_(out) {
  round_count_ = 0;
}

void EvaluationStatistics::initEvaluation() {
  // start evaluation
  this->eval_clock_start = std::chrono::high_resolution_clock::now();
}

void EvaluationStatistics::summarizeRound(
    const SimulationStatistics &round_stats) {
  using namespace std; // NOLINT
  this->step_counts_.push_back(round_stats.step_count_);
  this->total_disc_rewards_.push_back(round_stats.total_discounted_reward_);
  this->total_undisc_rewards_.push_back(round_stats.total_undiscounted_reward_);

  // current statistics
  double avg_disc_rew, avg_undisc_rew;
  double stderr_disc_rew, stderr_undisc_rew;
  this->averageRoundReward(avg_disc_rew, avg_undisc_rew);
  this->stderrRoundReward(stderr_disc_rew, stderr_undisc_rew);
  int nwidth = 5;
  // print round summary in one line
  *out_ << fixed << setprecision(2) << "Round(" << right << setw(4)
        << round_count_ << "): "
        << "reward(" << right << setw(nwidth)
        << round_stats.total_discounted_reward_ << " / " << right
        << setw(nwidth) << round_stats.total_undiscounted_reward_ << ") "
        << "time(" << right << setw(nwidth) << round_stats.total_round_time
        << "ms) - "
        << "avg total reward: " << right << setw(nwidth) << avg_disc_rew
        << " (+-" << stderr_disc_rew << ") / " << right << setw(nwidth)
        << avg_undisc_rew << " (+-" << stderr_undisc_rew << ")" << endl;
  round_count_++;
}

void EvaluationStatistics::endEvaluation() {
  // record eval time
  auto eval_clock_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed =
      eval_clock_end - this->eval_clock_start;
  this->total_eval_time = elapsed.count();
  //
}

void EvaluationStatistics::printRewardStatistics(std::ostream &out) const {
  double avg_disc_rew, avg_undisc_rew;
  double stderr_disc_rew, stderr_undisc_rew;
  this->averageRoundReward(avg_disc_rew, avg_undisc_rew);
  this->stderrRoundReward(stderr_disc_rew, stderr_undisc_rew);
  // print average and std of discounted rewards
  out << "Average total discounted reward (stderr) = "
      << std::to_string(avg_disc_rew) << " (" << std::to_string(stderr_disc_rew)
      << ")" << std::endl;
  // print average and std of undiscounted rewards
  out << "Average total undiscounted reward (stderr) = "
      << std::to_string(avg_undisc_rew) << " ("
      << std::to_string(stderr_undisc_rew) << ")" << std::endl;
}

void EvaluationStatistics::averageRoundReward(double &discountedAvg,
                                              double &undiscountedAvg) const {
  int n = this->step_counts_.size();
  if (n <= 0) {
    // return values
    discountedAvg = 0.0;
    undiscountedAvg = 0.0;
  } else {
    double sumDiscounted = 0;
    double sumUndiscounted = 0;
    for (int i = 0; i < n; i++) {
      sumDiscounted += this->total_disc_rewards_[i];
      sumUndiscounted += this->total_undisc_rewards_[i];
    }
    // return values
    discountedAvg = sumDiscounted / n;
    undiscountedAvg = sumUndiscounted / n;
  }
}

void EvaluationStatistics::stderrRoundReward(double &discountedStderr,
                                             double &undiscountedStderr) const {
  double discAvg = 0.0;
  double undiscAvg = 0.0;
  averageRoundReward(discAvg, undiscAvg);
  // calculation of sample variance (see Puente: "Messtechnik" Def. 4.22)
  int n = this->step_counts_.size();
  if (n <= 1) {
    // return values
    discountedStderr = 0.0;
    undiscountedStderr = 0.0;
  } else {
    double sum2Disc = 0.0;
    double sum2Undisc = 0.0;
    for (int i = 0; i < n; i++) {
      double rewardDis = this->total_disc_rewards_[i];
      double rewardUndis = this->total_undisc_rewards_[i];
      double xDisc = rewardDis - discAvg;
      double xUndisc = rewardUndis - undiscAvg;
      sum2Disc += xDisc * xDisc;
      sum2Undisc += xUndisc * xUndisc;
    }
    auto nd = static_cast<double>(n - 1);
    double sampleVarDis = sum2Disc / nd;
    double sampleVarUndis = sum2Undisc / nd;

    // return values
    // divide by n since the results are Monte-Carlo samples (see P. Hennig
    // Probabilistic ML Lecture 4 Slide 10)
    discountedStderr = std::sqrt(sampleVarDis / n);
    undiscountedStderr = std::sqrt(sampleVarUndis / n);
  }
}

void EvaluationStatistics::writeEvalutionResultsToFile(
    const std::string &filename) const {
  // print rewards to file
  std::ofstream rf; // reward file

  rf.open(filename);
  this->printEvaluationResults(rf);

  rf << std::endl;

  this->printRewardStatistics(rf);

  rf.close();
}

void EvaluationStatistics::printEvaluationResults(std::ostream &os) const {
  using namespace std; // NOLINT
  // ----------------------------------------------
  // Round | #Steps | disc. Reward | undisc. Reward
  // ----------------------------------------------
  if (!this->step_counts_.empty()) {
    string roundCol = "Round";
    string stepCol = "#Steps";
    string discRewCol = "disc. Reward";
    string undiscRewCol = "undisc. Reward";

    string sep = " | ";

    // create header
    stringstream ss;
    ss << roundCol << sep << stepCol << sep << discRewCol << sep
       << undiscRewCol;
    string headline = ss.str();
    string line = string(headline.length(), '-');

    // print header
    os << line << endl << headline << endl << line << endl;

    // print body
    os.precision(3);
    for (int i = 0; i < this->step_counts_.size(); i++) {
      os << right << setw(roundCol.length()) << i << sep << right
         << setw(stepCol.length()) << this->step_counts_[i] << sep << right
         << setw(discRewCol.length()) << fixed << this->total_disc_rewards_[i]
         << sep << right << setw(undiscRewCol.length()) << fixed
         << this->total_undisc_rewards_[i] << endl;
    }
  }
}

/* -------------------------------------------------------------------------- */
/*                               Evaluator class                              */
/* -------------------------------------------------------------------------- */

Evaluator::Evaluator(std::unique_ptr<Planner> &&planner)
    : planner_(std::move(planner)), round_(0) {
  this->eval_stats_ = std::make_unique<EvaluationStatistics>(&std::cout);
}

int Evaluator::runEvaluation(int argc, char *argv[]) {
  auto main_clock_start = std::chrono::high_resolution_clock::now();

  planner_->initializePlanner();
  planner_->displayParameters();

  // initialize evaluation statistics
  this->eval_stats_->initEvaluation();

  // run evaluation
  runEvaluationLoop();

  // end evaluation
  this->eval_stats_->endEvaluation();
  printEvaluationEnd(round_, main_clock_start);

  // save results in file
  saveEvaluationResults();

  return 0;
}

void Evaluator::runEvaluationLoop() {
  for (round_ = 0; round_ < Globals::config.eval_len; round_++) {
    planner_->runPlanningLoop();
    auto round_results = planner_->getSimulationStatistics();
    this->eval_stats_->summarizeRound(*round_results);

    planner_->resetPlanner();
  }
}

void Evaluator::printEvaluationEnd(
    int num_rounds,
    const std::chrono::time_point<std::chrono::high_resolution_clock>
        &main_clock_start) const {
  std::cout << std::endl;
  // print completed # num_runs
  std::cout << "Completed " << num_rounds << " run(s)." << std::endl;
  std::cout << std::endl;
  this->eval_stats_->printEvaluationResults(std::cout);
  std::cout << std::endl;
  this->eval_stats_->printRewardStatistics(std::cout);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - main_clock_start;
  std::cout << "Total time: " << elapsed.count() << " seconds." << std::endl;
}

void Evaluator::saveEvaluationResults() const {
  // get current date and time in correct format
  auto nowTimePoint = std::chrono::system_clock::now();
  auto nowTp = std::chrono::system_clock::to_time_t(nowTimePoint);
  std::stringstream sstime;
  sstime << std::put_time(std::localtime(&nowTp), "%F_%T");
  // determine filename
  std::stringstream ssfn;
  ssfn << "Eval_" << this->eval_stats_->round_count_ << "_" << sstime.str()
       << ".txt";
  std::string filename = ssfn.str();
  // write to file
  this->eval_stats_->writeEvalutionResultsToFile(filename);
}

} // namespace solver_ipft
