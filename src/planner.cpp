#include <glog/logging.h>

#include <solver_ipft/planner.hpp>
namespace solver_ipft {

Planner::Planner() : step_(0), round_(0), sim_stats_(nullptr) {}

bool Planner::runStep() {
  ValuedAction valuedAct = findAction();

  bool terminal = takeAction(valuedAct);

  return terminal;
}

ValuedAction Planner::findAction() {
  this->sim_stats_->initStep(round_, step_);

  //* solver: search for action
  DLOG(WARNING) << "[runStep] Begin search...";
  // stop search time
  ValuedAction valuedAct = this->solver_->search();
  this->sim_stats_->summarizeSearch();
  DLOG(WARNING) << "[runStep] ...end search";

  return std::move(valuedAct);
}

bool Planner::takeAction(const ValuedAction &valuedAct) {
  //* world: execute action and receive observation
  DLOG(WARNING) << "[runStep] Begin world interaction...";
  // stop execution time
  Observation *obs = nullptr;
  bool terminal = this->world_->executeAction(valuedAct.action_, obs);
  double reward = this->world_->getReward();
  this->sim_stats_->summarizeExecuteAction(valuedAct.action_, *obs, reward);
  DLOG(WARNING) << "[runStep] ...end world interaction";

  //* solver: update internal belief with observation from world
  DLOG(WARNING) << "[runStep] Begin solver belief update...";
  // stop belief update time
  this->solver_->beliefUpdate(valuedAct.action_, *obs);
  this->sim_stats_->summarizeBeliefUpdate();
  DLOG(WARNING) << "[runStep] ...end solver belief update";

  //* end step
  this->sim_stats_->summarizeRewards(round_, step_, reward);
  step_++;

  // free obs
  this->model_->freeObs(obs);
  return terminal;
}

void Planner::planningLoop() {
  for (int i = 0; i < Globals::config.sim_len; i++) {
    bool terminal = this->runStep();

    if (terminal) {
      break;
    }
  }
}

int Planner::runPlanning(int argc, char *argv[]) {
  //* initialize parameters and random class
  // TODO(max) option parser --> gflags library

  auto main_clock_start = std::chrono::high_resolution_clock::now();

  initializePlanner();

  displayParameters();

  runPlanningLoop();

  printPlanningEnd(1, main_clock_start);

  return 0;
}

void Planner::runPlanningLoop() {
  this->sim_stats_->initRound(round_);
  planningLoop();

  // get solver history
  History solverHist = solver_->copyHistory();
  // get (true) world history
  std::vector<State *> worldHist = world_->copyWorldStateSequence();

  this->sim_stats_->endRound(this->step_, std::move(solverHist),
                             std::move(worldHist));
}

void Planner::initializePlanner() {
  //* initialize model
  this->model_ = initializeModel(); // the model is shared between the world and
                                    // the solver, i.e. they use the same model
  //* initialize world
  this->world_ = initializePOMDPWorld(this->model_);
  this->world_->initialize();
  //* initialize belief
  auto initialBelief = this->model_->initialBelief("DEFAULT");
  //* initialize solver
  this->solver_ = initializeSolver(this->model_, std::move(initialBelief));

  //* initialize simulation statistics
  this->sim_stats_ = std::make_unique<SimulationStatistics>(
      this->model_, this->world_, this->solver_, &std::cout);

  this->round_ = 0;
  this->step_ = 0;
}

std::unique_ptr<SimulationStatistics> Planner::getSimulationStatistics() {
  return std::move(this->sim_stats_);
}

void Planner::resetPlanner() {
  this->round_++;
  this->step_ = 0;

  // reset world
  this->world_->initialize();

  // initial belief
  auto initialBelief = this->model_->initialBelief("DEFAULT");
  // reset solver
  this->solver_->setBelief(std::move(initialBelief));

  // reset simulation statistics
  this->sim_stats_ = std::make_unique<SimulationStatistics>(
      this->model_, this->world_, this->solver_, &std::cout);
}

} // namespace solver_ipft
