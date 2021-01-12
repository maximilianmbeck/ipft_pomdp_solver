
#include <ipft/util/prob_densities.hpp>
#include <ipft/core/globals.hpp>
#include <cmath>

namespace solver_ipft
{

    double NormalDistr::prob(double x, double mu, double sigma)
    {
        double zval = (x - mu) / sigma;
        double p = Globals::INV_SQRT2PI * std::exp(-(zval * zval) / 2) / sigma;
        return p;
    }

} // namespace solver_ipft