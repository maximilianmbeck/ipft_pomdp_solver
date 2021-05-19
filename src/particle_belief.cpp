#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <glog/logging.h>
#include <solver_ipft/core/globals.hpp>
#include <solver_ipft/core/particle_belief.hpp>
#include <solver_ipft/util/debug.hpp>
#include <solver_ipft/util/output.hpp>

namespace solver_ipft {

ParticleBelief::ParticleBelief(std::vector<State*> particles,
                               bool beliefTerminated,
                               std::shared_ptr<POMDP> model,
                               std::shared_ptr<Random> rand)
        : ParticleBelief(std::move(particles),
                         beliefTerminated,
                         std::move(model),
                         std::move(rand),
                         std::make_unique<NoReinvigoration>()) {
}

ParticleBelief::ParticleBelief(std::vector<State*> particles,
                               std::vector<State*> wp_particles,
                               bool beliefTerminated,
                               std::shared_ptr<POMDP> model,
                               std::shared_ptr<Random> rand,
                               std::unique_ptr<ParticleReinvigorator>&& afterResampleReinvigorator)
        : ParticleBelief(std::move(particles),
                         beliefTerminated,
                         std::move(model),
                         std::move(rand),
                         std::move(afterResampleReinvigorator)) {
    this->weighted_posterior_particles_ = std::move(wp_particles);
}

ParticleBelief::ParticleBelief(std::vector<State*> particles,
                               bool beliefTerminated,
                               std::shared_ptr<POMDP> model,
                               std::shared_ptr<Random> rand,
                               std::unique_ptr<ParticleReinvigorator>&& afterResampleReinvigorator)
        : Belief(std::move(model)), rand_(std::move(rand)), particles_(std::move(particles)),
          beliefTerminated_(beliefTerminated), afterResampleReinvigorator_(std::move(afterResampleReinvigorator)),
          num_particles_(particles.size()) {
    init();
}

void ParticleBelief::init() {
    if (std::abs(State::weightSum(particles_) - 1.0) > Globals::DOUBLEEQLIM) {
        LOG(FATAL) << "[ParticleBelief::ParticleBelief] Particle weights sum to " << State::weightSum(particles_)
                   << " instead of 1." << std::endl;
    }
    this->num_particles_ = this->particles_.size();
}

ParticleBelief::~ParticleBelief() {
    this->model_->freeStates(this->particles_);
    this->model_->freeStates(this->weighted_posterior_particles_);
}

std::unique_ptr<Belief> ParticleBelief::clone() const {
    return std::make_unique<ParticleBelief>(this->model_->copyStates(particles_),
                                            this->model_->copyStates(weighted_posterior_particles_),
                                            this->beliefTerminated_,
                                            this->model_,
                                            this->rand_,
                                            this->afterResampleReinvigorator_->clone());
}

std::vector<State*> ParticleBelief::particles() const {
    return this->model_->copyStates(this->particles_);
}

const State* ParticleBelief::particle(int i) const {
    if (i >= this->particles_.size()) {
        return nullptr;
    }
    return this->particles_[i];
}

int ParticleBelief::numParticles() const {
    int particle_size = this->particles_.size();
    CHECK(this->num_particles_ == particle_size) << "Numbers of particles do not match.";
    return this->num_particles_;
}

std::string ParticleBelief::text(const std::vector<State*>& particleSet) const {
    ParticleSetToString pbPrinter(this->model_);
    std::string description = pbPrinter.shortDescription(particleSet, this->beliefTerminated_);
    std::stringstream ss;
    ss << description;
    return ss.str();
}

std::string ParticleBelief::text() const {
    return this->text(this->particles_);
}

std::string ParticleBelief::detailedText(const std::vector<State*>& particleSet) const {
    ParticleSetToString pbPrinter(this->model_);
    std::string description = pbPrinter.shortDescription(particleSet, this->beliefTerminated_);

    std::stringstream ss;
    std::string lineSep = std::string(description.length(), '=');
    ss << lineSep << std::endl;
    ss << description << std::endl;
    ss << pbPrinter.particleTable(particleSet);
    ss << lineSep << std::endl;

    return ss.str();
}

std::string ParticleBelief::detailedText() const {
    return this->detailedText(this->particles_);
}

std::ostream& operator<<(std::ostream& os, const ParticleBelief& partBelief) {
    os << partBelief.text();
    return os;
}

bool ParticleBelief::isTerminalBelief() const {
    return this->beliefTerminated_;
}

double ParticleBelief::particleWeightSum() const {
    return State::weightSum(this->particles_);
}

// core: Algorithm Particle_filter (Table 4.3 "Probabilistic robotics" p.78)
// should not be called if beliefTerminated_ is true (check before call)
double ParticleBelief::update(const Action& action, const Observation& obs) {
    if (this->beliefTerminated_) // fallback strategy if called anyway
    {
        // if belief is terminated this action should not be chosen again in
        // ucbActionSelection
        LOG(ERROR) << "Try to update terminal belief!";
        return Globals::NEG_INFTY;
    }

    // log belief update
    DLOG(INFO) << "[PF] " << std::setfill(' ') << std::setw(7) << std::left << "update " << this->text() << " with "
               << model_->to_string(action) << model_->to_string(&obs);

    std::vector<State*> predictedParticles;
    double total_state_reward = 0;
    double total_old_weight = 0;     // for reward normalisation
    double total_updated_weight = 0; // for weight normalisation
    double total_new_weight = 0;     // for error message (corresponds to weight sum in julia code)

    bool terminalBelief = false;
    for (auto& state : this->particles_) {

        // state posterior / next state
        State* stateP = this->model_->transition(*state, action);
        double prob = this->model_->obsProb(*stateP, obs);
        bool terminal = this->model_->terminalState(*stateP, action);

        // reward calculation (calculates the first term of equation (1) in IPFT
        // paper)
        double reward = this->model_->reward(*state, action, *stateP);
        total_old_weight += state->weight_;
        total_state_reward += reward * state->weight_;
        // update state weight
        // stateP->weight_ = state->weight_ * prob;  // use this state update, if
        // states are not resampled in every timestep
        stateP->weight_ = prob;
        total_new_weight += prob;

        // avoid round off to zero #1
        if (prob > Globals::DOUBLEWEIGHTEQLIM) // prob != 0.0
        {
            // new state is not a terminal state and observation probability is not
            // 0.0
            total_updated_weight += stateP->weight_;
            predictedParticles.push_back(stateP);
        } else {
            // LOG(WARNING) << "Particle " << *stateP << " rejected with weight " <<
            // prob << ". Observation was " << obs << ".";
            this->model_->freeState(stateP);
        }

        // avoid round off to zero #2 (see rlabbe online book particle filters)
        // stateP->weight_ += Globals::DOUBLEWEIGHTEQLIM;  // avoid round-off to
        // zero total_updated_weight += stateP->weight_;
        // predictedParticles.push_back(stateP);

        // Concept Terminal Action (see
        // https://github.com/johannes-fischer/icml2020_ipft)
        // --> terminal == true for either all or no particles
        if ((!terminalBelief) && terminal) {
            terminalBelief = terminal;
        }
    }

    // reward normalisation (in case total old weight > 1.0)
    total_state_reward = total_state_reward / total_old_weight;

    // Discard update ?
    // the probability of all updated particles is very small (should happen very
    // rarely) in this case just use the old particle set
    if (predictedParticles.empty()) {
        LOG(ERROR) << "[ParticleBelief] New particle set is empty. Observation "
                      "probability of all updated "
                      "states is 0.0. Observation was "
                   << obs << std::endl
                   << "Total new weight sum was: " << total_new_weight << std::endl
                   << "Total old weight sum was: " << total_old_weight << std::endl
                   << "Previous particle set was: " << std::endl
                   << this->detailedText();
        return total_state_reward;
    }

    // check if particles need to be reinvigorated
    bool reinvigorateParticles =
        this->afterResampleReinvigorator_->particleReinvigorationNeeded(predictedParticles, action, obs);

    // assign weighted posterior particle set
    this->model_->freeStates(this->weighted_posterior_particles_);
    State::normalizeWeights(predictedParticles, total_updated_weight);
    this->weighted_posterior_particles_ = predictedParticles;

    //? Particle reinvigoration point: before resampling (currently not supported)

    //! RESAMPLING in EVERY step
    std::vector<State*> resampledPart = this->sample(num_particles_, this->weighted_posterior_particles_);
    this->model_->freeStates(this->particles_);
    this->particles_ = resampledPart;

    // log belief update result
    DLOG(INFO) << "[PF] " << std::setfill(' ') << std::setw(7) << std::left << "new " << this->text();
    DLOG_IF(ERROR, debug::allParticlesEqual(this))
        << "[PF] After resampling: All particles equal to " << *(resampledPart[0]);

    // update belief terminated
    this->beliefTerminated_ = terminalBelief;

    //? particle reinvigoration point: after resampling
    if (reinvigorateParticles) {
        this->particles_ = this->afterResampleReinvigorator_->reinvigorate(this->particles_, action, obs);
        DLOG(INFO) << "[PF] " << std::setfill(' ') << std::setw(7) << std::left << "new " << this->text();
    }

    return total_state_reward;
}

void ParticleBelief::reinvigorateParticlesAfterResampling(const Action& act, const Observation& obs) {
    // need weighted posterior particles to determine if some particles need to be
    // reinvigorated / if noise needs to be added
    if (this->afterResampleReinvigorator_->particleReinvigorationNeeded(
            this->weighted_posterior_particles_, act, obs)) {
        this->particles_ = this->afterResampleReinvigorator_->reinvigorate(this->particles_, act, obs);
        DLOG(INFO) << "[PF] " << std::setfill(' ') << std::setw(7) << std::left << "new " << this->text();
    }
}

// Algorithm Low_variance_sampler (Table 4.4 "Probabilistic Robotics")
// weights must be normalized for this algorithm to work
std::vector<State*> ParticleBelief::sample(int num, const std::vector<State*>& particleSet) const {
    double weightSum = this->particleWeightSum();
    CHECK(std::abs(weightSum - 1.0) <= Globals::DOUBLEEQLIM);

    std::vector<State*> sampledParticles;
    double invNum = 1.0 / static_cast<double>(num);
    double r = this->rand_->nextUniform(0, invNum);
    double cur = particleSet[0]->weight_; // corresponds to c in alg
    int i = 0;
    for (int m = 0; m < num; m++) {
        double u = r + m * invNum;
        while (u > cur) {
            i++;
            if (i == particleSet.size()) {
                i = 0;
            }
            cur += particleSet[i]->weight_;
        }
        State* particle = this->model_->copyState(particleSet[i]);
        particle->weight_ = invNum;
        sampledParticles.push_back(particle);
    }
    return sampledParticles;
}

std::vector<State*> ParticleBelief::sample(int num) const {
    return this->sample(num, this->particles_);
}

State* ParticleBelief::sample() const {
    return this->sample(1).back();
}

std::unique_ptr<ParticleBelief> ParticleBelief::sampleParticleBelief(int num) const {
    return std::make_unique<ParticleBelief>(this->sample(num),
                                            this->beliefTerminated_,
                                            this->model_,
                                            this->rand_,
                                            this->afterResampleReinvigorator_->clone());
}

State* ParticleBelief::mean() const {
    return State::weightedMean(this->particles_, this->model_);
}

State* ParticleBelief::std() const {
    State* std = State::weightedVariance(this->particles_, this->model_);
    State::varToStd(std);
    return std;
}

void ParticleBelief::setReinvigorationStrategy(std::unique_ptr<ParticleReinvigorator>&& reinvigorator) {
    this->afterResampleReinvigorator_ = std::move(reinvigorator);
}

} // namespace solver_ipft