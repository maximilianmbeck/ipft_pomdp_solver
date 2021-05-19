#pragma once

#include <memory>
#include <utility>
#include <solver_ipft/core/globals.hpp>
#include <solver_ipft/core/history.hpp>
#include <solver_ipft/interface/spaces.hpp>
namespace solver_ipft {

class POMDP;
class ParticleBelief;

/**
 * @brief A class representing an interface for the belief
 */
class Belief {
public:
    std::shared_ptr<POMDP> model_;

    explicit Belief(std::shared_ptr<POMDP> m) : model_(std::move(m)){};
    virtual ~Belief() = default;

    Belief(const Belief&) = delete;
    Belief(Belief&&) = delete;
    Belief& operator=(const Belief&) = delete;
    Belief& operator=(Belief&&) = delete;

    /**
     * @brief Sample states from the belief
     *
     * @param num   Number of states to be sampled
     * @return      std::vector<State *> Set of sampled states
     */
    virtual std::vector<State*> sample(int num) const = 0;

    /**
     * @brief Sample a state from the belief
     *
     * @return State* the sampled state
     */
    virtual State* sample() const = 0;

    /**
     * @brief Generates a particle-based representation of the belief consisting
     * of num particles
     *
     * @param num the number of particles
     * @return ParticleBelief* the samples particle belief / set
     */
    virtual std::unique_ptr<ParticleBelief> sampleParticleBelief(int num) const = 0;

    /**
     * @brief Update the belief
     * A call to this function requires that the belief is not a terminal belief.
     * @param action Action taken in the last step
     * @param obs Observation received in the last step
     * @return double reward received by taking action a
     */
    virtual double update(const Action& action, const Observation& obs) = 0;

    /**
     * @brief Check for terminal belief
     *
     * @return true
     * @return false
     */
    virtual bool isTerminalBelief() const = 0;

    /**
     * @brief Deepcopy current belief.
     *
     * @return std::unique_ptr<Belief>
     */
    virtual std::unique_ptr<Belief> clone() const = 0;

    /**
     * @brief Calculate the mean state of the belief
     */
    virtual State* mean() const = 0;

    /**
     * @brief Calculate the stddev of the state of the belief
     */
    virtual State* std() const = 0;

    /* --------------------------------------------------------------------------
     */
    /*                            Output/print function */
    /* --------------------------------------------------------------------------
     */

    virtual std::string text() const = 0;
    virtual std::string detailedText() const = 0;
    friend std::ostream& operator<<(std::ostream& os, const Belief& belief) {
        os << belief.text();
        return os;
    }
};

} // namespace solver_ipft
