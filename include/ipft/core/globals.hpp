#pragma once

//! helper functions, constants go here

#include <cmath>
#include <ipft/interface/spaces.hpp>
#include <limits>

#include "ipft/config.hpp"

namespace solver_ipft {

namespace Globals {

// double equals limit
constexpr double DOUBLEEQLIM = 1e-8;
constexpr double DOUBLEWEIGHTEQLIM = 1e-300;  // default 1e-160, MINIMUM DOUBLE VAL: 2.22507e-308

constexpr double INV_SQRT2PI = 0.3989422804014327; // 1/sqrt(2*PI)
extern Config config;

const double POS_INFTY = std::numeric_limits<double>::is_iec559 ? std::numeric_limits<double>::infinity()
                                                                : std::numeric_limits<double>::max();
const double NEG_INFTY = -POS_INFTY;

inline double discount(int d) {
    return std::pow(config.discount_gamma, d);
}

// no action
constexpr int noAction = -1;

namespace ucb {
// penalizing values for illegal action
constexpr int large_count = 1000000;
constexpr double neg_inf_val = -1e10;

// values for preferred actions
//* array or function also possible
constexpr int smart_count = 10;
constexpr double smart_value = 1.0;

} // namespace ucb

} // namespace Globals

} // namespace solver_ipft