#pragma once
#include <cmath>
#include <vector>
namespace solver_ipft {

namespace util {

std::vector<double> weightedSampleVarianceToStd(const std::vector<double>& sampleVariance);
}  // namespace util

}  // namespace solver_ipft
