#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/spaces.hpp>
namespace solver_ipft {

double State::weightSum(const std::vector<State*>& states) {
    double weight = 0;
    for (int i = 0; i < states.size(); i++) {
        weight += states[i]->weight_;
    }
    return weight;
}

void State::normalizeWeights(const std::vector<State*>& states, const double& total_weight) {
    for (int i = 0; i < states.size(); i++) {
        states[i]->weight_ = states[i]->weight_ / total_weight;
    }
}

void State::normalizeWeights(const std::vector<State*>& states) {
    double total_weight = 0.0;
    for (State* s : states) {
        total_weight += s->weight_;
    }
    normalizeWeights(states, total_weight);
}

// see wikipedia: "weighted_arithmetic_mean" https://en.wikipedia.org/wiki/Weighted_arithmetic_mean#Weighted_sample_variance
State* State::weightedMean(const std::vector<State*>& states, const POMDP* model) {
    double weightSum = State::weightSum(states);
    CHECK(std::abs(weightSum - 1.0) <= Globals::DOUBLEEQLIM);
    CHECK(states.size() > 0) << "State set empty";

    int dimensions = model->numDimStateSpace();
    State* meanState = model->allocateState();

    for (int dim = 0; dim < dimensions; dim++) {
        double dimValue = 0.0;
        for (int i = 0; i < states.size(); i++) {
            dimValue += states[i]->weight_ * states[i]->get(dim);
        }
        dimValue = dimValue / weightSum;
        meanState->set(dimValue, dim);
    }
    return meanState;
}

// see:
// https://numpy.org/doc/stable/reference/generated/numpy.average.html
// and
// https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
// for weighted variance estimate
State* State::weightedVariance(const std::vector<State*>& states, const POMDP* model) {
    State* meanState = State::weightedMean(states, model);
    CHECK(states.size() > 1) << "Need more than one particle to compute a variance";

    int dimensions = model->numDimStateSpace();
    State* varState = model->allocateState();

    for (int dim = 0; dim < dimensions; dim++) {
        double dimVarianceValue = 0.0;
        for (int i = 0; i < states.size(); i++) {
            double x = states[i]->get(dim) - meanState->get(dim);
            dimVarianceValue += states[i]->weight_ * x * x;
        }
        dimVarianceValue = dimVarianceValue / State::weightSum(states);
        varState->set(dimVarianceValue, dim);
    }
    model->freeState(meanState);
    return varState;
}

void State::varToStd(State* varState) {
    for (int dim = 0; dim < varState->dimensions(); dim++) {
        double dimValue = std::sqrt(varState->get(dim));
        varState->set(dimValue, dim);
    }
}

bool State::weightCompare(State* state1, State* state2) {
    return state1->weight_ < state2->weight_;
}

}  // namespace solver_ipft
