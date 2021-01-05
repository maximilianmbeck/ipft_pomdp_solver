#pragma once

#include <random>

namespace ipft {

/* -------------------------------------------------------------------------- */
/*                                 Definitions                                */
/* -------------------------------------------------------------------------- */

class Random {
public:
    Random();
    Random(int seedOffset);
    // Destructor
    ~Random();

    static Random* gen();

    double nextNormal(const double& mu = 0, const double& sigma = 1.0) const;

    double nextUniform(const double& min = 0, const double& max = 1.0) const;

    unsigned nextUnsigned() const;
    int nextUniformInt(int n) const;
    int nextUniformInt(int min, int max) const;

    std::mt19937_64* engine() const;

    // template <class T>
    // T nextElement(const std::vector<T> &vec)
    // {
    //     return vec[NextInt(vec.size())];
    // }


private:
    static Random* _rand;
    mutable std::mt19937_64* _randEngine; // could also use mt19937_64 (maybe slightly faster, depends on compiler)
};

} // namespace ipft