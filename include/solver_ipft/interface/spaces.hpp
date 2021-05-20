#pragma once

// Definition of state, observation and action space in this file.

#include <memory>
#include <glog/logging.h>

#include <solver_ipft/core/globals.hpp>
#include <solver_ipft/interface/point.hpp>

namespace solver_ipft {

class POMDP;

/**
 * @brief Defines a class for the state
 * Must be implemented to meet the requirements of the use-case.
 */
class State : public Point {
public:
    /// weight of the state for the particle approximation of the belief
    double weight{1.0};

    State() = default;
    ~State() override = default;

    State(const State&) = default;
    State(State&&) = default;
    State& operator=(const State&) = default;
    State& operator=(State&&) = default;

    /**
     * @brief Cumulative sum of weights over vector of states
     */
    static double weightSum(const std::vector<State*>& states);

    /**
     * @brief Normalizes the weights of all the states by providing the weight sum
     */
    static void normalizeWeights(const std::vector<State*>& states, const double& weightSum);

    /**
     * @brief Normalizes the weights of all the states.
     */
    static void normalizeWeights(const std::vector<State*>& states);

    /**
     * @brief Calculates the mean state.
     */
    static State* weightedMean(const std::vector<State*>& states, const std::shared_ptr<POMDP>& model);

    /**
     * @brief Calculates the variance of the mean state.
     */
    static State* weightedVariance(const std::vector<State*>& states, const std::shared_ptr<POMDP>& model);

    /**
     * @brief Calculates the stddev of a state given its variance.
     */
    static void varToStd(State* varState);

    /**
     * @brief Check if state2 has a higher weight as state1
     */
    static bool hasBiggerWeight(State* state1, State* state2);
};

using Observation = Point;
// action space assumed to be discrete
using Action = int;
using ActionValue = Point;
} // namespace solver_ipft