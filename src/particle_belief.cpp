#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <ipft/core/particle_belief.hpp>
#include <sstream>

#include "ipft/core/globals.hpp"
#include "ipft/util/debug.hpp"
#include "ipft/util/output.hpp"

namespace solver_ipft {

ParticleBelief::ParticleBelief(std::vector<State *> particles,
                               std::vector<State *> wp_particles,
                               bool beliefTerminated,
                               const POMDP *model,
                               const Random *rand,
                               const ParticleReinvigorator *afterResampleReinvigorator)
    : ParticleBelief(particles, beliefTerminated, model, rand, afterResampleReinvigorator) {
    this->weighted_posterior_particles_ = wp_particles;
}

ParticleBelief::ParticleBelief(std::vector<State *> particles,
                               bool beliefTerminated,
                               const POMDP *model,
                               const Random *rand,
                               const ParticleReinvigorator *afterResampleReinvigorator)
    : Belief(model), rand_(rand), particles_(particles), beliefTerminated_(beliefTerminated), afterResampleReinvigorator_(afterResampleReinvigorator) {
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
    delete afterResampleReinvigorator_;
}

Belief *ParticleBelief::clone() const {
    return new ParticleBelief(this->model_->copyStates(particles_),
                              this->model_->copyStates(weighted_posterior_particles_),
                              this->beliefTerminated_, this->model_, this->rand_, this->afterResampleReinvigorator_->clone());
}

std::vector<State *> ParticleBelief::particles() const {
    return this->model_->copyStates(this->particles_);
}

const State *ParticleBelief::particle(int i) const {
    if (i >= this->particles_.size()) {
        return nullptr;
    } else {
        return this->particles_[i];
    }
}

int ParticleBelief::numParticles() const {
    int particle_size = this->particles_.size();
    CHECK(this->num_particles_ == particle_size) << "Numbers of particles do not match.";
    return this->num_particles_;
}

std::string ParticleBelief::text(const std::vector<State *> &particleSet) const {
    ParticleSetToString pbPrinter(this->model_);
    std::string description = pbPrinter.shortDescription(particleSet, this->beliefTerminated_);
    std::stringstream ss;
    ss << description;
    return ss.str();
}

std::string ParticleBelief::text() const {
    return this->text(this->particles_);
}

std::string ParticleBelief::detailedText(const std::vector<State *> &particleSet) const {
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

std::ostream &operator<<(std::ostream &os, const ParticleBelief &partBelief) {
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
double ParticleBelief::update(const Action &action, const Observation &obs) {
    if (this->beliefTerminated_)  // fallback strategy if called anyway
    {
        // if belief is terminated this action should not be chosen again in ucbActionSelection
        LOG(ERROR) << "Try to update terminal belief!";
        return Globals::NEG_INFTY;
    }

    // log belief update
    DLOG(INFO) << "[PF] " << std::setfill(' ') << std::setw(7) << std::left << "update " << this->text()
               << " with " << model_->to_string(action) << model_->to_string(obs);

    std::vector<State *> predictedParticles;
    double total_state_reward = 0;
    double total_old_weight = 0;      // for reward normalisation
    double total_updated_weight = 0;  // for weight normalisation
    double total_new_weight = 0;      // for error message (corresponds to weight sum in julia code)

    bool terminalBelief = false;
    for (int i = 0; i < this->particles_.size(); i++) {
        State *state = this->particles_[i];
        double reward = this->model_->reward(*state, action);
        // state posterior / next state
        State *stateP = this->model_->transition(*state, action);
        double prob = this->model_->obsProb(*stateP, obs);
        bool terminal = this->model_->terminalState(*stateP, action);

        // reward calculation (calculates the first term of equation (1) in IPFT paper)
        total_old_weight += state->weight_;
        total_state_reward += reward * state->weight_;
        // update state weight
        // stateP->weight_ = state->weight_ * prob;  // TODO: (this is probably correct, mbeck) but: compare with weight update before reward calculation???
        stateP->weight_ = prob;
        total_new_weight += prob;

        // avoid round off to zero #1
        if (prob > Globals::DOUBLEWEIGHTEQLIM)  // prob != 0.0
        {
            // new state is not a terminal state and observation probability is not 0.0
            total_updated_weight += stateP->weight_;
            predictedParticles.push_back(stateP);
        } else {
            // LOG(WARNING) << "Particle " << *stateP << " rejected with weight " << prob << ". Observation was " << obs << ".";
            this->model_->freeState(stateP);
        }

        // avoid round off to zero #2 (see rlabbe online book particle filters)
        // stateP->weight_ += Globals::DOUBLEWEIGHTEQLIM;  // avoid round-off to zero
        // total_updated_weight += stateP->weight_;
        // predictedParticles.push_back(stateP);

        // Concept Terminal Action (see https://github.com/johannes-fischer/icml2020_ipft)
        // --> terminal == true for either all or no particles
        if ((!terminalBelief) && terminal) {
            terminalBelief = terminal;
        }
    }

    // reward normalisation (in case total old weight > 1.0)
    total_state_reward = total_state_reward / total_old_weight;

    // Discard update ?
    // the probability of all updated particles is very small (should happen very rarely)
    // in this case just use the old particle set
    if (predictedParticles.size() == 0) {
        LOG(ERROR) << "[ParticleBelief] New particle set is empty. Observation probability of all updated "
                      "states is 0.0. Observation was "
                   << obs << std::endl
                   << "Total new weight sum was: " << total_new_weight << std::endl
                   << "Total old weight sum was: " << total_old_weight << std::endl
                   << "Previous particle set was: " << std::endl
                   << this->detailedText();
        return total_state_reward;
    }

    // check if particles need to be reinvigorated
    bool reinvigorateParticles = this->afterResampleReinvigorator_->particleReinvigorationNeeded(predictedParticles, action, obs);

    // assign weighted posterior particle set
    this->model_->freeStates(this->weighted_posterior_particles_);
    State::normalizeWeights(predictedParticles, total_updated_weight);
    this->weighted_posterior_particles_ = predictedParticles;

    //? Particle reinvigoration point: before resampling (currently not supported)

    //! RESAMPLING in EVERY step
    std::vector<State *> resampledPart = this->sample(num_particles_, this->weighted_posterior_particles_);
    this->model_->freeStates(this->particles_);
    this->particles_ = resampledPart;

    // log belief update result
    DLOG(INFO) << "[PF] " << std::setfill(' ') << std::setw(7) << std::left << "new " << this->text();
    DLOG_IF(ERROR, debug::allParticlesEqual(this)) << "[PF] After resampling: All particles equal to " << *(resampledPart[0]);

    // Resampling needed?
    // if (this->particles_.size() <
    //     num_particles_) // Resample if particle set is smaller than the fixed particle set size
    // {
    //     State::normalizeWeights(this->particles_, total_updated_weight);
    //     std::vector<State *> resampledPart = this->sample(num_particles_);
    //     this->model_->freeStates(this->particles_);
    //     this->particles_ =
    //         resampledPart; //? effective number of particles could still be too small -> TODO: handle this case

    //     DLOG_IF(WARNING, debug::allParticlesEqual(this))
    //         << "After resampling due to particle deletion: All particles equal to " << *(resampledPart[0]);
    // }
    // else // Resample if effective particle size is small
    // {
    //     assert(this->particles_.size() == num_particles_);
    //     // Compute Neff (effective number of particles approximation) and normalize weights
    //     // see "Probabilistic Robotics" p.85/86: if variance of the weights is high resampling should be performed
    //     // see e.g. Arumpalam for computation of Neff
    //     double weight_square_sum = 0;
    //     for (int i = 0; i < this->particles_.size(); i++)
    //     {
    //         State *particle = this->particles_[i];
    //         // Normalize weights
    //         double newPartWeight = particle->weight_ / total_updated_weight;
    //         weight_square_sum += newPartWeight * newPartWeight;
    //         particle->weight_ = newPartWeight;
    //     }
    //     // Resample if the effective number of particles is "small"
    //     double num_effective_particles = 1.0 / weight_square_sum;
    //     if (num_effective_particles < this->num_particles_ / 2.0)
    //     {
    //         // number of effective particles smaller than half of the number of particles
    //         std::vector<State *> resampledPart = this->sample(num_particles_);

    //         DLOG_IF(WARNING, debug::allParticlesEqual(this))
    //             << "After resampling due to paricle deprivation: All particles equal to " << *(resampledPart[0]);

    //         // free old particles
    //         this->model_->freeStates(this->particles_);
    //         this->particles_ = resampledPart;
    //     }
    // }

    // update belief terminated
    this->beliefTerminated_ = terminalBelief;

    //? particle reinvigoration point: after resampling
    if (reinvigorateParticles) {
        this->particles_ = this->afterResampleReinvigorator_->reinvigorate(this->particles_, action, obs);
        DLOG(INFO) << "[PF] " << std::setfill(' ') << std::setw(7) << std::left << "new " << this->text();
    }

    return total_state_reward;
}

void ParticleBelief::reinvigorateParticlesAfterResampling(const Action &act, const Observation &obs) {
    // need weighted posterior particles to determine if some particles need to be reinvigorated / if noise needs to be added
    if (this->afterResampleReinvigorator_->particleReinvigorationNeeded(this->weighted_posterior_particles_, act, obs)) {
        this->particles_ = this->afterResampleReinvigorator_->reinvigorate(this->particles_, act, obs);
        DLOG(INFO) << "[PF] " << std::setfill(' ') << std::setw(7) << std::left << "new " << this->text();
    }
}

// Algorithm Low_variance_sampler (Table 4.4 "Probabilistic Robotics")
// weights must be normalized for this algorithm to work
std::vector<State *> ParticleBelief::sample(int num, const std::vector<State *> &particleSet) const {
    double weightSum = this->particleWeightSum();
    CHECK(std::abs(weightSum - 1.0) <= Globals::DOUBLEEQLIM);

    std::vector<State *> sampledParticles;
    double invNum = 1.0 / static_cast<double>(num);
    double r = this->rand_->nextUniform(0, invNum);
    double cur = particleSet[0]->weight_;  // corresponds to c in alg
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
        State *particle = this->model_->copyState(particleSet[i]);
        particle->weight_ = invNum;
        sampledParticles.push_back(particle);
    }
    // std::shuffle(sampledParticles.begin(), sampledParticles.end(), *(this->rand_->engine()));
    return sampledParticles;
}

std::vector<State *> ParticleBelief::sample(int num) const {
    return this->sample(num, this->particles_);  // TODO: maybe from weighted_posterior_particles_
}

State *ParticleBelief::sample() const {
    return this->sample(1).back();
}

// std::vector<double> ParticleBelief::weightedParticleMean() const {
//     return State::weightedMean(this->particles_);
// }

// std::vector<double> ParticleBelief::weightedParticleVariance() const {
//     return State::weightedVariance(this->particles_);
// }

State *ParticleBelief::mean() const {
    return State::weightedMean(this->particles_, this->model_);
}

State *ParticleBelief::std() const {
    State *std = State::weightedVariance(this->particles_, this->model_);
    State::varToStd(std);
    return std;
}

}  // namespace solver_ipft