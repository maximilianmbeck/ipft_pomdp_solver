#include <solver_ipft/util/util.hpp>

namespace solver_ipft {
namespace util {

std::vector<double> weightedSampleVarianceToStd(const std::vector<double>& sampleVariance) {
    std::vector<double> std(sampleVariance.size());
    for (int dim = 0; dim < sampleVariance.size(); dim++) {
        double dimValue = std::sqrt(sampleVariance[dim]);
        std[dim] = dimValue;
    }
    return std;
}
} // namespace util
} // namespace solver_ipft
