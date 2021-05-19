#pragma once

#include <memory>
#include <random>
namespace solver_ipft {

/* -------------------------------------------------------------------------- */
/*                                 Definitions                                */
/* -------------------------------------------------------------------------- */

class Random {
public:
    Random();
    explicit Random(int seedOffset);

    static Random* gen();

    /**
     * @brief Returns a sample from normal distribution
     *
     * @param mu        mean
     * @param sigma     stddev (standarddeviation)
     * @return double   the sample
     */
    double nextNormal(const double& mu = 0, const double& sigma = 1.0) const;

    double nextUniform(const double& min = 0, const double& max = 1.0) const;

    unsigned nextUnsigned() const;
    int nextUniformInt(int n) const;
    int nextUniformInt(int min, int max) const;

    int nextDiscrProbDistrInt(const std::vector<double>& discreteProbDistrWeights) const;

    std::mt19937_64* engine() const;

private:
    static Random* _rand;
    mutable std::unique_ptr<std::mt19937_64> _randEngine;
};

} // namespace solver_ipft