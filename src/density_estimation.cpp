#include <cmath>
#include <glog/logging.h>
#include <solver_ipft/core/density_estimation.hpp>
#include <solver_ipft/util/util.hpp>

namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                        Class KernelDensityEstimator                        */
/* -------------------------------------------------------------------------- */

double KernelDensityEstimator::computeDensityValue(const State* s, const ParticleBelief* b) const {
    int dimensions = b->model->numDimStateSpace();
    double densityEstimate = 0.0;
    if (dimensions <= 1) {
        // univariate case
        double bandwidth = bwSelector_->computeBandwidth(b); // TODO(max): performance improvement! do this only once or
                                                             // store variance in particle belief
        densityEstimate = kernel_->univariateValue(s, b, bandwidth);
    } else {
        // multivariate case
        std::vector<double> bwMatrixDiagElmts =
            bwSelector_->computeBandwidthMatrixDiagElmts(b); // TODO(max): same as above
        densityEstimate = kernel_->multivariateValue(s, b, bwMatrixDiagElmts);
    }
    CHECK(!std::isnan(densityEstimate)) << "Density estimate is nan.";
    return densityEstimate;
}

/* -------------------------------------------------------------------------- */
/*                          Class RotBandwidthSelector                        */
/* -------------------------------------------------------------------------- */

// Rule-of-thumb bandwidth / Silvermans Rule (see IPFT paper supplemental
// equation (5))
double RotBandwidthSelector::computeBandwidth(const ParticleBelief* b) const {
    State* wSampleStd = b->std();
    double stdeviation = wSampleStd->get(0);
    b->model->freeState(wSampleStd);
    double minBandw = std::sqrt(std::numeric_limits<double>::epsilon()); //  = 1.49012e-08
    return std::max(1.0592238410488122 * std::pow(static_cast<double>(b->numParticles()), -0.2) * stdeviation,
                    minBandw);
}

std::vector<double> RotBandwidthSelector::computeBandwidthMatrixDiagElmts(const ParticleBelief* b) const {
    int dimensions = b->model->numDimStateSpace();
    State* wSampleStd = b->std();
    std::vector<double> bandwidthMatrixDiagElmts(dimensions, 0.0);
    for (int dim = 0; dim < dimensions; dim++) {
        bandwidthMatrixDiagElmts[dim] = wSampleStd->get(dim);
        bandwidthMatrixDiagElmts[dim] *= std::pow(4.0 / (b->numParticles() * (dimensions + 2)), 1.0 / (dimensions + 4));
    }
    b->model->freeState(wSampleStd);
    return bandwidthMatrixDiagElmts;
}

/* -------------------------------------------------------------------------- */
/*                             Class NormalKernel                             */
/* -------------------------------------------------------------------------- */

double NormalKernel::univariateValue(const State* s, const ParticleBelief* belief, const double& bandwidth) const {
    double value = 0.0;
    for (int i = 0; i < belief->numParticles(); i++) {
        const State* particle = belief->particle(i);
        double x = (s->get(0) - particle->get(0));
        double arg = -((x * x) / (2 * bandwidth * bandwidth));
        double expVal = std::exp(arg);
        value += particle->weight_ * expVal;
    }
    value *= Globals::INV_SQRT2PI / bandwidth;
    return value;
}

double NormalKernel::multivariateValue(const State* s,
                                       const ParticleBelief* belief,
                                       const std::vector<double>& bandwidth) const {
    int dimensions = s->dimensions();
    double value = 0.0;
    for (int i = 0; i < belief->numParticles(); i++) {
        double arg = 0.0;
        for (int dim = 0; dim < dimensions; dim++) {
            double x = (s->get(dim) - belief->particle(i)->get(dim));
            arg += (x * x) / bandwidth[dim];
        }
        arg *= -0.5;

        value += belief->particle(i)->weight_ * std::exp(arg);
    }

    double detBwMatrix = 1.0;
    for (int dim = 0; dim < dimensions; dim++) {
        detBwMatrix *= bandwidth[dim];
    }
    value *= 1.0 / std::sqrt(std::pow(2.0 * M_PIf64, dimensions) * detBwMatrix);
    return value;
}

} // namespace solver_ipft