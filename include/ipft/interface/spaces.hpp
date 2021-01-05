#pragma once

// Definition of state, observation and action space in this file.

#include <glog/logging.h>

#include <ipft/core/globals.hpp>
#include <ipft/interface/point.hpp>

namespace ipft {

class POMDP;

class State : public Point {
   public:
    State() : weight_(1.0){};
    virtual ~State(){};
    double weight_;

    static double weightSum(const std::vector<State*>& states);

    static void normalizeWeights(const std::vector<State*>& states, const double& total_weight);

    static void normalizeWeights(const std::vector<State*>& states);

    static State* weightedMean(const std::vector<State*>& states, const POMDP* model);

    static State* weightedVariance(const std::vector<State*>& states, const POMDP* model);

    static void varToStd(State* varState);

    static bool weightCompare(State* state1, State* state2);
};

typedef Point Observation;
// action space assumed to be discrete not continuous at first (later maybe point too)
typedef int Action;
typedef Point ActionValue;
}  // namespace ipft