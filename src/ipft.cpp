#include <glog/logging.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <ipft/solver/ipft.hpp>
#include <ipft/util/debug.hpp>
#include <ipft/util/random.hpp>
#include <limits>
#include <sstream>
#include <stdexcept>
namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                         IpftSearchStatistics class                         */
/* -------------------------------------------------------------------------- */

/* ---------------------- IpftSearchStatistics printers --------------------- */

std::ostream& operator<<(std::ostream& os, const IpftSearchStatistics& statistics) {
    os << statistics.shortDescription();
    return os;
}

std::string IpftSearchStatistics::shortDescription() const {
    using namespace std;
    stringstream ss;
    ss << "[Search(" << time_search << "ms, " << num_simulations << " sims): " << optimalAction << "]";
    return ss.str();
}

std::string IpftSearchStatistics::text() const {
    using namespace std;
    string description = this->shortDescription();

    constexpr int indentFromRightSubHeader = 20;

    stringstream ss;
    string separator = string(description.length(), '=');
    string secSep = string(description.length() - 20, '-');
    ss << separator << endl;
    ss << description << endl;
    // ss << secSep << "Root belief" << endl;
    // ss << this->root_belief->detailedText();
    ss << secSep << "Actions(" << to_string(this->valuedActions.size()) << ")" << endl;
    ss << this->printValuedActions();
    ss << secSep << "Action-Observation-Sequences" << endl;
    ss << this->printActionObservationSequences();
    ss << secSep << "Time(" << this->time_search << "ms)" << endl;
    ss << this->printTimes();
    ss << secSep << "Nodes/Visits on level (" << to_string(this->num_tree_vnodes) << "nodes, "
       << to_string(this->num_simulations) << "sims)" << endl;
    ss << printNodeStats();
    ss << secSep << "Deepest sim depth(" << this->deepest_simulation_depth << ")" << endl;
    ss << separator << endl;
    return ss.str();
}

std::string IpftSearchStatistics::printValuedActions() const {
    using namespace std;
    constexpr char fillChar = ' ';
    constexpr char optimalChar = '*';
    constexpr int width1 = 2;

    stringstream ss;

    for (int i = 0; i < this->valuedActions.size(); i++) {
        char oChar = ' ';
        if (this->valuedActions[i].action_ == this->optimalAction.action_) {
            oChar = optimalChar;
        }
        ss << right << setw(width1) << setfill(fillChar) << oChar << setw(1) << " ";
        ss << left << setw(15) << setfill(fillChar) << this->model_->to_string(this->valuedActions[i].action_) << this->valuedActions[i] << this->actionSequences[i] << endl;
    }
    return ss.str();
}

std::string IpftSearchStatistics::printActionObservationSequences() const {
    using namespace std;
    constexpr char fillChar = ' ';
    constexpr char optimalChar = '*';
    constexpr int width1 = 2;
    stringstream ss;

    for (int i = 0; i < this->valuedActions.size(); i++) {
        char oChar = ' ';
        if (this->valuedActions[i].action_ == this->optimalAction.action_) {
            oChar = optimalChar;
        }
        ss << right << setw(width1) << setfill(fillChar) << oChar << setw(1) << " ";
        ss << this->actionSequences[i].text() << endl;
    }

    return ss.str();
}

std::string IpftSearchStatistics::printTimes() const {
    using namespace std;

    constexpr int width1 = 20;
    constexpr int width2 = 10;
    constexpr char fillChar = ' ';
    // TODO: print percentage of total time
    stringstream ss;
    ss << left << setw(width1) << setfill(fillChar) << "node selection:" << right << setw(width2) << setfill(fillChar)
       << to_string(time_node_selection) << "ms" << endl;
    ss << left << setw(width1) << setfill(fillChar) << "belief update:" << right << setw(width2) << setfill(fillChar)
       << to_string(time_belief_update) << "ms" << endl;
    ss << left << setw(width1) << setfill(fillChar) << "inf gain calc:" << right << setw(width2) << setfill(fillChar)
       << to_string(time_information_gain_computation) << "ms" << endl;
    ss << left << setw(width1) << setfill(fillChar) << "node rollout:" << right << setw(width2) << setfill(fillChar)
       << to_string(time_node_rollout) << "ms" << endl;
    ss << left << setw(width1) << setfill(fillChar) << "backup:" << right << setw(width2) << setfill(fillChar)
       << to_string(time_backup) << "ms" << endl;

    return ss.str();
}

std::string IpftSearchStatistics::printNodeStats() const {
    using namespace std;

    constexpr int width1 = 10;
    constexpr int width2 = 8;
    constexpr char fillChar = ' ';

    stringstream ss;
    ss << right << setw(width1) << setfill(fillChar) << "treelevel"
       << " | ";
    ss << right << setw(width2) << setfill(fillChar) << "#nodes"
       << " | ";
    ss << right << setw(width2) << setfill(fillChar) << "#visits" << endl;
    for (int i = 0; i < this->num_visits_vnodes_on_level.size(); i++) {
        ss << right << setw(width1) << setfill(fillChar) << to_string(i) << " | ";
        ss << right << setw(width2) << setfill(fillChar) << to_string(num_vnodes_on_level[i]) << " | ";
        ss << right << setw(width2) << setfill(fillChar) << to_string(num_visits_vnodes_on_level[i]) << endl;
    }
    return ss.str();
}

/* -------------------------------------------------------------------------- */
/*                              Ipft value class                              */
/* -------------------------------------------------------------------------- */

IpftValue::IpftValue() {
    value[0] = 0;
    value[1] = 0;
}

IpftValue::IpftValue(const double& stateValue, const double& informationValue) {
    value[0] = stateValue;
    value[1] = informationValue;
}

void IpftValue::add(const Value& val) {
    const IpftValue& v = static_cast<const IpftValue&>(val);
    value[0] += v.value[0];
    value[1] += v.value[1];
}

void IpftValue::update(const Value& val, int count) {
    const IpftValue& v = static_cast<const IpftValue&>(val);
    for (int i = 0; i < IpftValue::componentCount; i++) {
        // this->value[i] = (this->value[i] * count + v.value[i]) / (count + 1);
        this->value[i] += (v.value[i] - this->value[i]) / (count + 1);
    }
}

void IpftValue::set(const Value& val) {
    const IpftValue& v = static_cast<const IpftValue&>(val);
    value[0] = v.value[0];
    value[1] = v.value[1];
}

void IpftValue::setComponent(int index, const double& val) {
    if (index < componentCount && index >= 0) {
        this->value[index] = val;
    }
}

double IpftValue::getRawComponent(int index) const {
    return this->value[index];
}

double IpftValue::getWeightedComponent(int index) const {
    switch (index) {
        case 0:
            return this->value[index];
            break;
        case 1:
            return Globals::config.inf_gather_constant_lambda * value[index];
            break;
        default:
            throw std::out_of_range("IpftValue has only valid indices 0 and 1.");
            // return std::numeric_limits<double>::quiet_NaN();
            break;
    }
}

int IpftValue::getComponentCount() const {
    return componentCount;
}

double IpftValue::total() const {
    double total = value[0] + Globals::config.inf_gather_constant_lambda * value[1];
    return total;
}

Value* IpftValue::clone() const {
    return new IpftValue(*this);
}

IpftValue& IpftValue::operator+=(const IpftValue& add) {
    this->value[0] += add.value[0];
    this->value[1] += add.value[1];
    return *this;
}

IpftValue IpftValue::operator*(const double& factor) {
    IpftValue result(this->value[0] * factor, this->value[1] * factor);
    return result;
}

IpftValue IpftValue::operator+(const IpftValue& add) {
    IpftValue result(this->value[0] + add.value[0], this->value[1] + add.value[1]);
    return result;
}

std::string IpftValue::text() const {
    using namespace std;
    stringstream ss;
    ss << "[V: " << std::setprecision(2) << std::fixed << this->total()
       << " (" << std::setprecision(3) << this->value[0] << "|"
       << Globals::config.inf_gather_constant_lambda * this->value[1] << ")]";
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const IpftValue& v) {
    os << v.text();
    return os;
}

/* -------------------------------------------------------------------------- */
/*                              IPFT Solver class                             */
/* -------------------------------------------------------------------------- */

Ipft::Ipft(const POMDP* model, Belief* belief, const Random* rand, RolloutPolicy* rp)
    : Solver(model, belief), rand_(rand), rolloutPolicy_(rp) {
    infGainRewardCalculator_ = new EntropyInfGain();  // default information gain calculator
}

Ipft::Ipft(const POMDP* model, const Random* rand, RolloutPolicy* rp, DiscountedInformationGain* infGainRewardCalc)
    : Solver(model), rand_(rand), rolloutPolicy_(rp), infGainRewardCalculator_(infGainRewardCalc) {
}

Ipft::~Ipft() {
    delete rolloutPolicy_;
    delete infGainRewardCalculator_;
}

/* ---------------------------- solver interface ---------------------------- */

ValuedAction Ipft::search() {
    return search(Globals::config.time_per_move);
}

void Ipft::beliefUpdate(const Action& action, const Observation& obs) {
    // delete the current search tree
    if (this->root_ != nullptr) {
        delete this->root_;  //? reuse difficult: due to continuous observation space no observation is sampled twice
        this->root_ = nullptr;
    }
    //* use the particle filter to update the belief
    this->belief_->update(action, obs);
    //* update the history
    // add copy of new belief to history
    Belief* bp = this->belief_->clone();
    Observation* o = this->model_->copyObs(&obs);
    this->history_->add(action, o, bp);
}

void Ipft::setBelief(Belief* b) {
    // clear history
    delete this->history_;
    this->history_ = new History(this->model_);

    // clear belief
    if (this->belief_ != nullptr) {
        delete this->belief_;
    }

    // set new belief
    this->belief_ = b;
    // add initial belief to history
    Belief* initialBelief = this->belief_->clone();
    this->history_->addInitialBelief(initialBelief);

    // clear root
    if (this->root_ != nullptr) {
        delete this->root_;
        this->root_ = nullptr;
    }
}

Belief* Ipft::getBelief() const {
    return this->belief_->clone();
}

/* ----------------------------- helper methods ----------------------------- */

ValuedAction Ipft::search(double timeout) {
    using namespace std::chrono;
    // checking preconditions
    if (this->belief_->isTerminalBelief()) {
        LOG(FATAL) << "Treesearch with terminal belief!";
        // return no action
        return ValuedAction();
    }

    // initialize root for search
    if (this->root_ == nullptr) {
        if (Globals::config.record_statistics) {
            stats_ = new IpftSearchStatistics(this->model_);  // reset statistics
            // stats_->root_belief = static_cast<ParticleBelief*>(b->clone());
        }
        this->root_ = createVNode(nullptr, nullptr, nullptr, 0);
    }

    // std::cout << "[SEARCH] [BELIEF] " << this->belief_->detailedText() << std::endl; // LOG(INFO) truncates output

    auto start = high_resolution_clock::now();
    duration<double, std::milli> elapsed;
    int num_sims = 0;
    do {
        DLOG(INFO) << "[SEARCH] start simulation(" << num_sims << ")";
        // sample search particle set for root
        ParticleBelief* b = this->belief_->sampleParticleBelief(Globals::config.num_search_particles);
        b->setReinvigorationStrategy(new NoReinvigoration());

        DLOG(INFO) << "[SEARCH] sampled search particle set: " << b->detailedText();
        this->root_->setBelief(b);
        IpftValue val = simulate(this->root_, Globals::config.search_depth);
        DLOG(INFO) << "[SEARCH] end simulation(" << num_sims << ") " << val;
        DLOG(INFO) << "|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
        num_sims++;

        auto end = high_resolution_clock::now();
        elapsed = end - start;

        // convergence evaluation
        if (Globals::config.convergence_eval) {
            this->stats_->timesteps.push_back(elapsed.count());
            this->stats_->valuedActionsPerTimestep.push_back(this->root_->getValuedActions());
        }

    } while (
        (elapsed.count() < Globals::config.time_per_move) &&
        (num_sims <
         Globals::config.max_simulation_count));  // for fixed number of iterations change < to != in time comparison

    // find optimal action
    ValuedAction astar = optimalAction(this->root_);

    // search statistics update
    if (Globals::config.record_statistics) {
        this->stats_->time_search = elapsed.count();
        this->stats_->num_simulations = num_sims;
        // valued actions of the root
        this->stats_->valuedActions = this->root_->getValuedActions();
        // action-observation sequences
        for (int i = 0; i < this->stats_->valuedActions.size(); i++) {
            History actObsSeq(this->model_);
            actObsSeq = this->root_->maximumValueActionObservationSequence(this->stats_->valuedActions[i].action_);
            this->stats_->actionSequences.push_back(std::move(actObsSeq));
        }
        // optimal action
        this->stats_->optimalAction = astar;
    }

    return std::move(astar);
}

// Algorithm 1 in IPFT paper
IpftValue Ipft::simulate(VNode* vnode, int depth) {
    assert(vnode != nullptr);

    if (depth == 0) {
        IpftValue zero;
        vnode->updateValueCount(zero);
        return zero;
    }
    // log next tree level
    DLOG(WARNING) << "[SIM] tree level down..";
    auto startNodeSel = std::chrono::high_resolution_clock::now();  // time_node_selection

    //* Action space is discrete --> no progressive widening needed
    QNode* actionNode = ucbActionSelection(vnode);

    // log action selection
    DLOG(INFO) << "[SIM] select " << model_->to_string(actionNode->getAction()) << ", N(ha)=" << actionNode->getCount();

    //* Progressive widening for Observation space
    double pw_limit = Globals::config.pw_k_obs * std::pow(actionNode->getCount(), Globals::config.pw_alpha_obs);
    DLOG(INFO) << "[SIM][PW] num children C(ha)=" << actionNode->children().size() << " -> PW limit obs space: " << pw_limit;
    bool new_obs_sampled = false;
    VNode* obsNode = nullptr;    // the next observation node after the action node
    Observation* obs = nullptr;  // observation for the particle filter update

    //* observation generation
    // sample state s from b and generate observation o from (s,a)
    //? one observation is (almost) never sampled twice (continuous observation space)
    State* s = vnode->belief_->sample();
    State* statePosterior = this->model_->transition(*s, actionNode->getAction());
    Observation* o = this->model_->observation(*statePosterior);
    // states are unused
    this->model_->freeState(s);  // s not used anymore
    this->model_->freeState(statePosterior);

    // log observation generation
    DLOG(INFO) << "[SIM] generate " << model_->to_string(o)
               << " from " << model_->to_string(s) << model_->to_string(actionNode->getAction()) << model_->to_string(statePosterior);

    if (actionNode->children().size() <= pw_limit) {
        // log new obs node
        DLOG(INFO) << "[SIM] add new obs node";
        new_obs_sampled = true;  // new child vnode of actionNode will be created
        obs = o;
    } else {
        // select o / child vnode from children of actionNode uniformly
        int randomIndex = this->rand_->nextUniformInt(actionNode->children().size());
        obsNode = actionNode->children()[randomIndex];

        new_obs_sampled = false;  // use selected vnode obsNode for next simulation
        obs = o;

        // log observation selection
        DLOG(INFO) << "[SIM] select " << model_->to_string((obsNode->obsEdge_)) << " and replace by " << model_->to_string(obs);
    }

    //* Particle filter and reward calculation
    // get copy of current belief (bNext is b')
    Belief* bNext = vnode->getBelief();
    this->stats_->time_node_selection += stopTime(startNodeSel);  // time_node_selection

    // particle filter update belief / propagate belief forward in time (through model) and calculate reward
    auto startBelUpdate = std::chrono::high_resolution_clock::now();  // time_belief_update
    Action act = actionNode->getAction();
    double stateReward = bNext->update(act, *obs);
    CHECK(!std::isnan(stateReward)) << "State reward is nan.";
    this->stats_->time_belief_update += stopTime(startBelUpdate);  // time_belief_update

    // calculate information gathering reward term
    // vnode belief is b
    auto startInfGain = std::chrono::high_resolution_clock::now();  // time_information_gain_computation
    Belief* b = vnode->getBelief();                                 // a copy of the vnode belief
    double informationReward =
        this->infGainRewardCalculator_->computeDiscInfGain(Globals::config.inf_discount_gamma,
                                                           static_cast<const ParticleBelief*>(bNext),
                                                           static_cast<const ParticleBelief*>(b));
    delete b;
    CHECK(!std::isnan(informationReward)) << "Information reward is nan.";
    this->stats_->time_information_gain_computation += stopTime(startInfGain);  // time_information_gain_computation

    // local reward
    IpftValue locReward(stateReward, informationReward);

    // log local reward
    DLOG(INFO) << "[SIM] local reward " << locReward;

    //* terminal belief reached?
    if (bNext->isTerminalBelief()) {
        depth = 1;
    }

    //* Rollout and simulate
    IpftValue recursiveReward;
    if (new_obs_sampled) {
        // Add new VNode
        VNode* newChild = createVNode(actionNode, obs, bNext, vnode->getTreelevel() + 1);
        actionNode->children().push_back(newChild);

        // rollout
        recursiveReward = rollout(newChild, depth - 1);
    } else  // already existing child VNode is selected
    {
        assert(obsNode != nullptr);
        // set observation in vnode to obs, previous obs is archived
        obsNode->setObs(obs);
        // set belief in vnode to bNext, previous belief is archived
        obsNode->setBelief(bNext);

        // simulate
        recursiveReward = simulate(obsNode, depth - 1);
    }

    auto startBackup = std::chrono::high_resolution_clock::now();  // time_backup

    //* Compute accumulated discounted reward
    IpftValue discRecReward = (recursiveReward * Globals::config.discount_gamma);
    IpftValue accDiscountedReward = locReward + discRecReward;

    // log discounted recursive reward
    DLOG(INFO) << "[SIM] disc. recursive reward " << discRecReward;
    DLOG(INFO) << "[SIM] acc. disc. recursive reward " << accDiscountedReward;

    // for logging only
    Value* oldVNodeVal = vnode->getValueObj();
    Value* oldQNodeVal = actionNode->getValueObj();

    //* Backup
    vnode->updateValueCount(accDiscountedReward);
    // vnode->setCount(vnode->getCount() + 1);
    actionNode->updateValueCount(accDiscountedReward);

    // log backup
    DLOG(INFO) << "[SIM][BU] " << std::setfill(' ') << std::setw(15) << std::right << model_->to_string(vnode->obsEdge_)
               << " on level " << Globals::config.search_depth - depth << ": "
               << *(vnode->getValueRef()) << " = " << *oldVNodeVal << "<-" << accDiscountedReward << "|[C: " << vnode->getCount() << "]";
    DLOG(INFO) << "[SIM][BU] " << std::setfill(' ') << std::setw(15) << std::right << model_->to_string(actionNode->getAction())
               << " on level " << Globals::config.search_depth - depth << ": "
               << *(actionNode->getValueRef()) << " = " << *oldQNodeVal << "<-" << accDiscountedReward << "|[C: " << actionNode->getCount() << "]";

    // clean up temp logging objects
    delete oldVNodeVal;
    delete oldQNodeVal;

    if (Globals::config.record_statistics) {
        int treeLevel = vnode->getTreelevel();
        this->stats_->num_visits_vnodes_on_level[treeLevel]++;
        if (treeLevel > this->stats_->deepest_simulation_depth) {
            this->stats_->deepest_simulation_depth = treeLevel;
        }
        if (treeLevel == 0) {
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed;
            elapsed = end - startBackup;
            this->stats_->time_backup += elapsed.count();
        }
    }

    // log previous tree level
    DLOG(WARNING) << "[SIM] tree level up^^";

    //* return accumulated discounted reward
    return accDiscountedReward;
}

IpftValue Ipft::rollout(VNode* leaf, int depth) {
    // log rollout
    DLOG(WARNING) << "[ROLLOUT] estimate value of " << leaf->belief_->text() << "..";
    auto start = std::chrono::high_resolution_clock::now();
    IpftValue value = this->rolloutPolicy_->rollout(leaf->getBelief(), depth);
    this->stats_->time_node_rollout += stopTime(start);
    // update value of leaf / "newChild"
    leaf->updateValueCount(value);
    // log rollout result
    DLOG(WARNING) << "[ROLLOUT] result: " << value << "^^";
    return value;
}

SearchStatistics* Ipft::getSearchStatistics() const {
    return this->stats_;
}

ValuedAction Ipft::optimalAction(const VNode* vnode) const {
    const std::vector<QNode*>& qnodes = vnode->children();
    Value* initVal = new IpftValue(Globals::NEG_INFTY, 0.0);
    ValuedAction astar(-1, initVal, -1);
    double val = Globals::NEG_INFTY;
    for (int i = 0; i < qnodes.size(); i++) {
        double curNodeVal = qnodes[i]->getTotalValue();
        if (curNodeVal > val) {
            // delete ValuedAction
            // create new with copy of action
            astar = ValuedAction(qnodes[i]->getAction(), qnodes[i]->getValueObj(), qnodes[i]->getCount());
            val = curNodeVal;
        }
    }
    CHECK_NE(static_cast<int>(astar.action_), -1) << "Cannot select optimal action!";
    return astar;
}
// imitation of julia implementation
// QNode* Ipft::ucbActionSelection(VNode* vnode) const {
//     const std::vector<QNode*>& qnodes = vnode->children();
//     double best_ucb = Globals::NEG_INFTY;
//     QNode* best_qnode = nullptr;
//     double ltn = log(vnode->getCount());
//     for (int i = 0; i < qnodes.size(); i++) {
//         QNode* qchild = qnodes[i];
//         int n = qchild->getCount();
//         double wq = qchild->getTotalValue();
//         double ucbT = 0;
//         double ucb = 0;
//         if ((ltn <= 0 && n == 0) || Globals::config.explore_constant_c == 0) {
//             ucb = wq;
//         } else {
//             ucbT = Globals::config.explore_constant_c * sqrt(ltn / n);
//             ucb = wq + ucbT;
//         }

//         DLOG(INFO) << "[UCB] " << model_->to_string(qchild->getAction()) << ": [UCB1:"
//                    << std::right << std::setfill(' ') << std::setw(9) << std::setprecision(4) << std::fixed << ucb << "] = "
//                    << std::left << std::setfill(' ') << std::setw(31) << *(qchild->getValueRef())
//                    << "+[ucbT:" << std::right << std::setw(9) << ucbT << "]"
//                    << "|[C: " << qchild->getCount() << "]";

//         if (ucb >= best_ucb) {
//             best_ucb = ucb;
//             best_qnode = qchild;
//         }
//     }
//     CHECK(best_qnode != nullptr) << "Cannot select best UCB qnode!";
//     return best_qnode;
// }
// own version:
QNode* Ipft::ucbActionSelection(VNode* vnode) const {
    const std::vector<QNode*>& qnodes = vnode->children();
    double best_ucb = Globals::NEG_INFTY;
    QNode* best_qnode = nullptr;

    for (int i = 0; i < qnodes.size(); i++) {
        // try previously untried actions first
        if (qnodes[i]->getCount() == 0) {
            return qnodes[i];
        }
        // +1 in log(), since log(0) is undefined
        double ucbT = Globals::config.explore_constant_c * sqrt(log(vnode->getCount() + 1) / qnodes[i]->getCount());
        double qval = qnodes[i]->getTotalValue();
        double ucb = qval + ucbT;

        DLOG(INFO) << "[UCB] " << model_->to_string(qnodes[i]->getAction()) << ": [UCB1:"
                   << std::right << std::setfill(' ') << std::setw(9) << std::setprecision(4) << std::fixed << ucb << "] = "
                   << std::left << std::setfill(' ') << std::setw(31) << *(qnodes[i]->getValueRef())
                   << "+[ucbT:" << std::right << std::setw(9) << ucbT << "]"
                   << "|[C: " << qnodes[i]->getCount() << "]";

        if (ucb >= best_ucb) {
            best_ucb = ucb;
            best_qnode = qnodes[i];
        }
    }
    CHECK(best_qnode != nullptr) << "Cannot select best UCB qnode!";
    return best_qnode;
}

VNode* Ipft::createVNode(QNode* parent, Observation* obs, Belief* belief, int level) const {
    // initialize VNode
    VNode* vnode = new VNode(this->model_, parent, obs, belief, level);
    Value* initVal = new IpftValue();
    vnode->setValue(initVal);

    // statistics
    if (Globals::config.record_statistics) {
        this->stats_->num_vnodes_on_level[level]++;
        this->stats_->num_tree_vnodes++;
    }

    // get legal and preferred actions from model based on the current belief
    std::vector<Action> preferredActions = this->model_->preferredActions(belief);
    std::vector<Action> legalActions = this->model_->legalActions(belief);

    int num_pref_act = preferredActions.size();
    int num_legal_act = legalActions.size();

    assert(num_pref_act <= num_legal_act);

    // initialize children of vnode (for UCB action selection to work properly)
    if (num_legal_act == 0)  // no prior knowledge, all actions are equal
    {
        for (Action a = 0; a < this->model_->numActions(); a++) {
            QNode* qnode = new QNode(this->model_, vnode, a, level);  // same level as parent vnode
            qnode->setCount(0);
            Value* initVal = new IpftValue(0.0, 0.0);
            qnode->setValue(initVal);

            vnode->children().push_back(qnode);
        }
    } else {
        // initialize all action as penalized
        for (Action a = 0; a < this->model_->numActions(); a++) {
            QNode* qnode = new QNode(this->model_, vnode, a, level);
            qnode->setCount(Globals::ucb::large_count);
            Value* initVal = new IpftValue(Globals::ucb::neg_inf_val, 0.0);
            qnode->setValue(initVal);

            vnode->children().push_back(qnode);
        }

        // set legal actions to "regular" initialization
        for (int i = 0; i < num_legal_act; i++) {
            QNode* qnode = vnode->child(legalActions[i]);
            qnode->setCount(0);
            Value* initVal = new IpftValue(0.0, 0.0);
            qnode->setValue(initVal);
        }

        // set preferred actions to preferred values by ucb action selection
        for (int i = 0; i < num_pref_act; i++) {
            QNode* qnode = vnode->child(preferredActions[i]);
            qnode->setCount(
                Globals::ucb::smart_count);  // Improvement: different smart count/value for different actions
            Value* initVal = new IpftValue(Globals::ucb::smart_value, 0.0);
            qnode->setValue(initVal);
        }
    }
    return vnode;
}

/* -------------------------------------------------------------------------- */
/*                            Ipft helper functions                           */
/* -------------------------------------------------------------------------- */

double stopTime(const std::chrono::time_point<std::chrono::high_resolution_clock>& start) {
    if (Globals::config.record_statistics) {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed;
        elapsed = end - start;
        return elapsed.count();
    } else {
        return 0.0;
    }
}

}  // namespace solver_ipft