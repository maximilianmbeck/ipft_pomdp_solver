
#include <cmath>
#include <solver_ipft/core/globals.hpp>
#include <solver_ipft/util/prob_densities.hpp>

namespace solver_ipft {

double NormalDistr::prob(double x, double mu, double std) {
  double zval = (x - mu) / std;
  double p = Globals::INV_SQRT2PI * std::exp(-(zval * zval) / 2) / std;
  return p;
}

} // namespace solver_ipft