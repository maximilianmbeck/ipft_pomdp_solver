#include <ipft/util/util.hpp>

namespace ipft {
namespace util {

std::vector<double> weightedSampleVarianceToStd(const std::vector<double>& sampleVariance) {
    std::vector<double> std(sampleVariance.size());
    for (int dim = 0; dim < sampleVariance.size(); dim++) {
        double dimValue = std::sqrt(sampleVariance[dim]);
        std[dim] = dimValue;
    }
    return std;
}
}  // namespace util
}  // namespace ipft
