#pragma once

// Definition of state, observation and action space in this file.

#include <glog/logging.h>
#include <memory>

#include <solver_ipft/core/globals.hpp>
#include <solver_ipft/interface/point.hpp>

namespace solver_ipft {

class POMDP;

class State : public Point {
public:
  double weight_{1.0};

public:
  State() = default;
  ~State() override = default;

  State(const State &) = default;
  State(State &&) = default;
  State &operator=(const State &) = default;
  State &operator=(State &&) = default;

  static double weightSum(const std::vector<State *> &states);

  static void normalizeWeights(const std::vector<State *> &states,
                               const double &total_weight);

  static void normalizeWeights(const std::vector<State *> &states);

  static State *weightedMean(const std::vector<State *> &states,
                             const std::shared_ptr<POMDP> &model);

  static State *weightedVariance(const std::vector<State *> &states,
                                 const std::shared_ptr<POMDP> &model);

  static void varToStd(State *varState);

  static bool weightCompare(State *state1, State *state2);
};

using Observation = Point;
// action space assumed to be discrete
using Action = int;
using ActionValue = Point;
} // namespace solver_ipft