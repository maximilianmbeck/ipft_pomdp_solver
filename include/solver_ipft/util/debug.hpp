#pragma once

#include <solver_ipft/interface/belief.hpp>

namespace solver_ipft {

namespace debug {

// varType expected to be either int or double
template <typename varType>
bool variableHasIncreasedMoreThan(varType curVal, varType delta) {
    static varType oldVal = 0;
    // do not trigger on first call
    if (oldVal == 0) {
        oldVal = curVal;
        return false;
    }

    varType temp = curVal - oldVal;

    oldVal = curVal;

    return temp > delta;
}

bool allParticlesEqual(const Belief* belief);

bool allParticlesEqual(const std::vector<State*>& particles);

template <typename exp1DimState>
std::vector<State*> doubleVec2StateVec(const std::vector<double>& numbers) {
    std::vector<State*> states;
    // generate 1 dimensional states from number vector
    for (double n : numbers) {
        State* s = new exp1DimState();
        s->set(n, 0);
        s->weight_ = 1.0 / numbers.size();
        states.push_back(s);
    }

    return states;
}

template <typename oneDimType>
std::vector<Point*> doubleVec2PointVec(const std::vector<double>& numbers) {
    std::vector<Point*> points;
    // generate 1 dimensional states from number vector
    for (double n : numbers) {
        Point* s = new oneDimType();
        s->set(n, 0);
        points.push_back(s);
    }

    return points;
}

std::vector<State*> doubleVec2StateVec(const std::vector<double>& numbers, const std::shared_ptr<POMDP>& model);

void freeStateVec(std::vector<State*> states);

void freePointVec(std::vector<Point*> points);

} // namespace debug

} // namespace solver_ipft