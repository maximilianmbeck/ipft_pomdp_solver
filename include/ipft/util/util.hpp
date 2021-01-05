#pragma once
#include <cmath>
#include <vector>
namespace ipft {

namespace util {

std::vector<double> weightedSampleVarianceToStd(const std::vector<double>& sampleVariance);
}  // namespace util

}  // namespace ipft
