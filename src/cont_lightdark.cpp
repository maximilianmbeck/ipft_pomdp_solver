#include <iomanip>
#include <solver_ipft/core/particle_belief.hpp>
#include <solver_ipft/interface/belief.hpp>
#include <solver_ipft/problems/cont_lightdark.hpp>
#include <solver_ipft/util/prob_densities.hpp>
#include <solver_ipft/util/random.hpp>
namespace solver_ipft {
namespace cld {

/* -------------------------------------------------------------------------- */
/*                               CLDState class                               */
/* -------------------------------------------------------------------------- */
bool CLDState::equals(const Point &p) const {
  auto other = dynamic_cast<const CLDState *>(&p);
  return std::abs(this->get(0) - other->get(0)) <= this->epsilon();
}

int CLDState::dimensions() const { return 1; }

void CLDState::reset() { this->posState = 0.0; }

std::string CLDState::text() const {
  std::stringstream ss;
  ss.precision(cldPrec);
  ss << std::fixed << this->get(0);
  return ss.str();
}

double CLDState::get(int dim) const { return this->posState; }

void CLDState::set(const double &p, int dim) { this->posState = p; }

double CLDState::epsilon() const { return epsilonCLDState; }

std::ostream &operator<<(std::ostream &os, const CLDState &p) {
  os << p.text();
  return os;
}

/* -------------------------------------------------------------------------- */
/*                                CLDObs class                                */
/* -------------------------------------------------------------------------- */
bool CLDObs::equals(const Point &p) const {
  auto other = dynamic_cast<const CLDObs *>(&p);
  return std::abs(this->get(0) - other->get(0)) <= this->epsilon();
}

int CLDObs::dimensions() const { return 1; }

void CLDObs::reset() { this->posObs = 0.0; }

std::string CLDObs::text() const {
  std::stringstream ss;
  ss.precision(cldPrec);
  ss << std::fixed << this->get(0);
  return ss.str();
}

double CLDObs::get(int dim) const { return this->posObs; }

void CLDObs::set(const double &p, int dim) { this->posObs = p; }

double CLDObs::epsilon() const { return epsilonCLDObs; }

std::ostream &operator<<(std::ostream &os, const CLDObs &p) {
  os << p.text();
  return os;
}

/* -------------------------------------------------------------------------- */
/*                            CLDActionValue class                            */
/* -------------------------------------------------------------------------- */

CLDActionValue::CLDActionValue(const Action &act)
    : CLDObs(static_cast<double>(actToValueMap.at(act))) {}

std::ostream &operator<<(std::ostream &os, const CLDActionValue &p) {
  os << p.text();
  return os;
}

/* -------------------------------------------------------------------------- */
/*                             ContLightDark class                            */
/* -------------------------------------------------------------------------- */

/* ----------------------- Simulative model functions ----------------------- */

State *ContLightDark::createStartState() const {
  State *startState = this->allocateState();
  // in cld_table.jl / in ipft paper: initial state is drawn from initial belief
  // distribution
  double pos = this->rand_->nextNormal(muInitial, sigmaInitial);
  // double pos = -5.0;  // same as in sample_trajectory.jl
  startState->set(pos, 0);
  return startState;
}

std::unique_ptr<Belief> ContLightDark::initialBelief(const std::string &type) {
  std::vector<State *> particleSet;
  for (int i = 0; i < Globals::config.num_solver_particles; i++) {
    State *particle = this->allocateState();
    double pos = rand_->nextNormal(
        muInitial, sigmaInitial); // same as in sample_trajectory.jl
    particle->set(pos, 0); // dimension 0 (CLDStates have only 1 dimension)
    particleSet.push_back(particle);
  }
  State::normalizeWeights(particleSet, State::weightSum(particleSet));
  return std::make_unique<ParticleBelief>(
      particleSet, false, shared_from_this(), this->rand_,
      std::make_unique<ObsAdaptiveReinvigorator>(shared_from_this(),
                                                 this->rand_));
}

int ContLightDark::numActions() const { return numberOfActions; }

std::unique_ptr<ActionValue>
ContLightDark::valueOfAction(const Action &act) const {
  return std::make_unique<CLDActionValue>(
      act); // create a CLDActionValue object and return a unique pointer to it
}

State *ContLightDark::transition(const State &state,
                                 const Action &action) const {
  auto cldState = dynamic_cast<const CLDState *>(&state);
  auto cldAction = static_cast<const CLDAction>(action);

  // transition to the next state mu is the mean of the transition (normal)
  // distribution
  double mu = cldState->get(0) + actToValueMap.at(static_cast<int>(cldAction));
  double nextStatePos = rand_->nextNormal(
      mu, sigmaTransition); // nextState = s' (s prime) = state posterior

  // generate new state
  State *stateP = this->allocateState();
  stateP->set(nextStatePos, 0);

  return stateP;
}

Observation *ContLightDark::observation(const State &statePosterior) const {
  auto cldStatePost = dynamic_cast<const CLDState *>(&statePosterior);
  double statePostPos = cldStatePost->get(0);

  // draw random observation from observation distribution
  double sigmaObs = obsModelSigma(statePostPos);
  double nextObs = rand_->nextNormal(statePostPos, sigmaObs);

  // generate new observation
  Observation *obs = this->allocateObs();
  obs->set(nextObs, 0);

  return obs;
}

double ContLightDark::obsProb(const State &statePosterior,
                              const Observation &obs) const {
  auto cldCurState = dynamic_cast<const CLDState *>(&statePosterior);
  auto cldObs = dynamic_cast<const CLDObs *>(&obs);

  // probability for receiving observation obs in current state
  double sigmaObs = obsModelSigma(cldCurState->get(0));
  double x = cldObs->get(0);
  double pObs = NormalDistr::prob(x, cldCurState->get(0), sigmaObs);

  // return only pObs and not pObs*pTrans
  return pObs;
}

double ContLightDark::maxPossibleWeight(const Action &act,
                                        const Observation &obs) const {
  CLDState sp(obs.get(0));
  return obsProb(sp, obs);
}

double ContLightDark::obsModelSigma(const double &statePos) const {
  double sigmaObs = std::sqrt(2) * std::abs(statePos - lightSourceLoc) + 0.5;
  return sigmaObs;
}

double ContLightDark::reward(const State &state, const Action &action) const {
  auto cldState = dynamic_cast<const CLDState *>(&state);
  auto cldAction = static_cast<CLDAction>(action);

  double currentAbsPos = std::abs(cldState->get(0) - goalRegion);

  double reward = -1.0;
  if (cldAction == CLDAction::ZERO) {
    if (currentAbsPos <= 1.0) {
      reward = 100;
    } else if (1.0 < currentAbsPos && currentAbsPos <= 1.5) {
      reward = 200 - 100 * currentAbsPos;
    } else if (1.5 < currentAbsPos && currentAbsPos <= 2.0) {
      reward = 500 - 300 * currentAbsPos;
    } else if (2.0 < currentAbsPos) {
      reward = -100;
    }
  }
  return reward;
}

bool ContLightDark::terminalState(const State &statePosterior,
                                  const Action &action) const {
  auto cldAction = static_cast<CLDAction>(action);
  return cldAction == CLDAction::ZERO;
}

int ContLightDark::numDimStateSpace() const { return 1; }

std::vector<State *> ContLightDark::similarStates(const State &state,
                                                  int count) const {
  assert(count > 0);
  std::vector<State *> simStates;
  State *s = this->copyState(&state);
  auto cldState = dynamic_cast<CLDState *>(s);

  double mu = cldState->get(0);
  double std = Globals::config.min_particle_std;
  double total_weight = 0.0;

  cldState->weight_ = NormalDistr::prob(cldState->get(0), mu, std);
  simStates.push_back(cldState);
  total_weight += cldState->weight_;

  for (int i = 0; i < (count - 1); i++) {
    double x = rand_->nextNormal(mu, std);
    double p = NormalDistr::prob(x, mu, std);
    CLDState *simState = state_memory_pool_.Allocate();
    simState->set(x, 0);
    simState->weight_ = p;
    simStates.push_back(simState);
    total_weight += p;
  }
  State::normalizeWeights(simStates, total_weight);

  return simStates;
}

void ContLightDark::newParticle(State *particle,
                                const std::vector<State *> &particleSet,
                                const Action &act,
                                const Observation &obs) const {
  CLDState s(obs.get(0));
  Observation *obsTemp = this->observation(s);

  particle->set(obsTemp->get(0), 0);
  particle->weight_ = 1.0 / particleSet.size(); // normalize weight

  this->freeObs(obsTemp);
}

/* ---------------------------- Display function ---------------------------- */
std::string ContLightDark::to_string(const State *state) const {
  std::stringstream ss;
  ss.precision(cldPrec);
  auto cldState = dynamic_cast<const CLDState *>(state);
  if (cldState != nullptr) {
    ss << "[" << *cldState << " State]";
  } else {
    ss << "[ NULL State]";
  }
  return ss.str();
}
std::string ContLightDark::to_string(const Observation *obs) const {
  std::stringstream ss;
  ss.precision(cldPrec);
  auto cldObs = dynamic_cast<const CLDObs *>(obs);
  if (cldObs != nullptr) {
    ss << "[" << *cldObs << " Obs]";
  } else {
    ss << "[ NULL Obs]";
  }
  return ss.str();
}
std::string ContLightDark::to_string(const Action &action) const {
  std::stringstream ss;
  ss.precision(1);
  CLDActionValue cldAct(action);
  ss << "[" << std::setfill(' ') << std::setw(2) << std::right << cldAct.get(0)
     << " Act]";
  return ss.str();
}
std::string ContLightDark::to_string(const Belief *belief) const {
  return belief->text();
}

/* ----------------------- Memory management functions ---------------------- */

Observation *ContLightDark::allocateObs() const {
  CLDObs *obs = obs_memory_pool_.Construct();
  return obs;
}

State *ContLightDark::allocateState() const {
  CLDState *state = state_memory_pool_.Construct();
  return state;
}

Observation *ContLightDark::copyObs(const Observation *obs) const {
  CLDObs *newObs = obs_memory_pool_.Allocate();
  *newObs =
      *dynamic_cast<const CLDObs *>(obs); // default copy constructor called
  newObs->SetAllocated();
  return newObs;
}

State *ContLightDark::copyState(const State *state) const {
  CLDState *newState = state_memory_pool_.Allocate();
  *newState =
      *dynamic_cast<const CLDState *>(state); // default copy constructor called
  newState->SetAllocated();
  return newState;
}

void ContLightDark::freeObs(Observation *obs) const {
  obs_memory_pool_.Destroy(dynamic_cast<CLDObs *>(obs));
}

void ContLightDark::freeState(State *state) const {
  state_memory_pool_.Destroy(dynamic_cast<CLDState *>(state));
}

int ContLightDark::numActiveObs() const {
  return obs_memory_pool_.num_allocated();
}

int ContLightDark::numActiveStates() const {
  return state_memory_pool_.num_allocated();
}
} // namespace cld
} // namespace solver_ipft