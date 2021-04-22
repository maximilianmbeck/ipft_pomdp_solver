#include <glog/logging.h>

#include <cmath>
#include <iomanip>
#include <solver_ipft/core/pomdp_world.hpp>
#include <solver_ipft/simulation_statistics.hpp>

namespace solver_ipft {
constexpr int round_step_separator_len_ = 90;

SimulationStatistics::SimulationStatistics(POMDP* model, World* world, Solver* solver, std::ostream* out)
    : model_(model), world_(world), solver_(solver), out_(out), solver_hist_(nullptr) {
    total_discounted_reward_ = 0.0;
    total_undiscounted_reward_ = 0.0;
    step_count_ = 0;
}

SimulationStatistics::~SimulationStatistics() {
    if (solver_hist_ != nullptr)
        delete solver_hist_;
    this->model_->freeStates(world_hist_);
    for (int i = 0; i < this->search_stats_.size(); i++) {
        delete search_stats_[i];
    }
}

void SimulationStatistics::initRound(int round) {
    // print round header
    std::string roundHeaderSep = std::string(round_step_separator_len_, '#');
    *out_ << roundHeaderSep << " Round " << round << std::endl;
    // print initial world state
    *out_ << "Initial state: ";
    *out_ << this->model_->to_string(this->world_->getCurrentState());
    *out_ << std::endl;
    // print initial solver belief
    Belief* solverBel = this->solver_->getBelief();
    *out_ << "Initial solver belief: ";
    *out_ << this->model_->to_string(solverBel);
    *out_ << std::endl;
    //! DEBUG:
    // *out_ << solverBel->detailedText();
    delete solverBel;

    // start round
    this->round_clock_start = std::chrono::high_resolution_clock::now();
}

void SimulationStatistics::endRound(int step_count, History* solver_hist, std::vector<State*> world_hist) {
    // record round time
    auto round_clock_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed;
    elapsed = round_clock_end - this->round_clock_start;
    this->total_round_time = elapsed.count();

    *out_ << std::endl;
    // record other statistics
    this->step_count_ = step_count;
    this->solver_hist_ = solver_hist;
    this->world_hist_ = world_hist;

    // print round summary table
    this->printRoundResultsTable(*out_);
    *out_ << std::endl;

    // TODO write round summary to file

    // print total discounted reward
    *out_ << "Total discounted reward = " << this->total_discounted_reward_ << std::endl;
    // print total undiscounted reward
    *out_ << "Total undiscounted reward = " << this->total_undiscounted_reward_ << std::endl;
}

void SimulationStatistics::summarizeRewards(
    int round, int step, double reward) {
    // update rewards
    total_undiscounted_reward_ += reward;
    total_discounted_reward_ += Globals::discount(step) * reward;

    // print current rewards
    *out_ << "- Current rewards:" << std::endl
          << "  discounted / undiscounted = " << total_discounted_reward_ << " / " << total_undiscounted_reward_
          << std::endl;
}

void SimulationStatistics::initStep(int round, int step) {
    // print round step header
    std::string stepHeaderSep = std::string(round_step_separator_len_, '-');
    *out_ << stepHeaderSep << " Round " << round << " Step " << step << std::endl;

    // print previous world state
    *out_ << "- Prev. world state = ";
    State* s = this->world_->getCurrentState();
    *out_ << this->model_->to_string(s);
    *out_ << std::endl;

    // print previous belief of solver
    Belief* solverBel = this->solver_->getBelief();
    *out_ << "- Prev. solver belief = ";
    *out_ << this->model_->to_string(solverBel);
    *out_ << std::endl;
    //! DEBUG:
    // *out_ << solverBel->detailedText();
    delete solverBel;
}

void SimulationStatistics::summarizeSearch() {
    // record search statistics
    SearchStatistics* search_stats = nullptr;
    search_stats = this->solver_->getSearchStatistics();
    this->search_stats_.push_back(search_stats);

    // print search statistics
    if (Globals::config.print_search_step_results && search_stats != nullptr) {
        *out_ << search_stats->text() << std::endl;
    }
}

void SimulationStatistics::summarizeExecuteAction(const Action& act, const Observation& obs, const double& stateReward) {
    // print action (action is recorded in solver history)
    *out_ << "- Action = ";
    *out_ << this->model_->to_string(act);
    *out_ << std::endl;

    // print state (state is recorded in world object)
    *out_ << "- New world state = ";
    State* s = this->world_->getCurrentState();
    *out_ << this->model_->to_string(s);
    *out_ << std::endl;

    // print observation (observation is recorded in solver history)
    *out_ << "- Observation = ";
    *out_ << this->model_->to_string(&obs);
    *out_ << std::endl;

    // record and print obs prob
    double obsProb = this->model_->obsProb(*s, obs);
    this->obs_probs_.push_back(obsProb);
    this->model_->freeState(s);
    *out_ << "- ObsProb = " << std::to_string(obsProb) << std::endl;

    // record and print reward
    this->state_rewards_.push_back(stateReward);
    *out_ << "- Reward = " << std::to_string(stateReward) << std::endl;
}

void SimulationStatistics::summarizeBeliefUpdate() {
    //print belief of solver (belief is recorded in solver history)
    Belief* solverBel = this->solver_->getBelief();
    *out_ << "- new solver belief = ";
    *out_ << this->model_->to_string(solverBel);
    *out_ << std::endl;
    //! DEBUG:
    // *out_ << solverBel->detailedText();
    delete solverBel;
}

void SimulationStatistics::printRoundResultsTable(std::ostream& os) const {
    using namespace std;
    if (this->step_count_ > 0) {
        // ------------------------------------------------------------------------------------------------------------------------
        // Step | State | Solv.Mean | Solv.Std | Action | StatePost | Observation | ObsProb | Solv.MeanPost | Solv.StdPost | Reward
        // ------------------------------------------------------------------------------------------------------------------------

        vector<string> cols = {"Step", "   State", "Solv.Mean", "Solv.Std", "  Action", "StatePost", "Observation", "ObsProb", "Solv.MeanPost", "Solv.StdPost", "Reward"};

        string sep = " | ";

        // determine column widths
        vector<int> colws(cols.size(), 0);
        // initialize with header width
        for (int i = 0; i < cols.size(); i++) {
            colws[i] = cols[i].size();
        }
        // get first data row
        int step = 0;
        vector<string> dataRow = this->getStepResultsForTable(step);
        // adapt width to data
        CHECK_EQ(cols.size(), dataRow.size()) << "Number of header colums does not match nuber of data colums.";
        for (int i = 0; i < dataRow.size(); i++) {
            string dataField = dataRow[i];
            if (dataField.length() > colws[i]) {
                colws[i] = dataField.length();
            }
        }
        // print header
        stringstream ssh;
        for (int i = 0; i < cols.size(); i++) {
            string colHeader = cols[i];
            ssh << right << setw(colws[i]) << colHeader;
            if (i < (cols.size() - 1)) {
                ssh << sep;
            }
        }
        string headline = ssh.str();
        string line = string(headline.length(), '-');
        os << line << endl
           << headline << endl
           << line << endl;
        // print body
        do {
            // create data row string
            stringstream ssb;
            for (int i = 0; i < dataRow.size(); i++) {
                string dataField = dataRow[i];
                ssb << right << setw(colws[i]) << dataField;
                if (i < (cols.size() - 1)) {
                    ssb << sep;
                }
            }
            // add data row string to output
            os << ssb.str() << endl;
            step++;
            if (step >= this->step_count_) break;
            dataRow = this->getStepResultsForTable(step);
        } while (step < this->step_count_);
    }
}

std::vector<std::string> SimulationStatistics::getStepResultsForTable(int step) const {
    std::vector<std::string> dataRow;
    if (step < this->step_count_) {
        // step
        dataRow.push_back(std::to_string(step));
        // state
        dataRow.push_back(this->world_hist_[step]->text());
        // solver mean
        const Belief* b = this->solver_hist_->beliefPointer(step);
        State* mean = b->mean();
        dataRow.push_back(mean->text());
        b->model_->freeState(mean);
        // solver std
        State* std = b->std();
        dataRow.push_back(std->text());
        b->model_->freeState(std);
        // action
        std::unique_ptr<ActionValue> actVal = this->model_->valueOfAction(this->solver_hist_->action(step));
        dataRow.push_back(actVal->text());
        // state posterior
        dataRow.push_back(this->world_hist_[step + 1]->text());
        // observation
        const Observation* o = this->solver_hist_->observationPointer(step);
        dataRow.push_back(o->text());
        // obs prob
        dataRow.push_back(std::to_string(this->obs_probs_[step]));
        // solver mean posterior
        const Belief* bp = this->solver_hist_->beliefPointer(step + 1);
        State* meanp = bp->mean();
        dataRow.push_back(meanp->text());
        bp->model_->freeState(meanp);
        // solver std posterior
        State* stdp = bp->std();
        dataRow.push_back(stdp->text());
        bp->model_->freeState(stdp);
        // reward
        dataRow.push_back(std::to_string(this->state_rewards_[step]));
    }
    return dataRow;
}

void SimulationStatistics::printStateActionSequence() const {
    *out_ << "State Trajectory: " << std::endl;
    for (int i = 0; i < this->world_hist_.size(); i++) {
        *out_ << this->model_->to_string(world_hist_[i]);
        *out_ << std::endl;
    }
    *out_ << std::endl;
}

}  // namespace solver_ipft
