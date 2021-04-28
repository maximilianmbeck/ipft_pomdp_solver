#pragma once

namespace solver_ipft {

/**
 * @brief Class representing a normal distribution
 *
 */
class NormalDistr {
public:
  static double prob(double x, double mu = 0.0, double std = 1.0);
};

} // namespace solver_ipft