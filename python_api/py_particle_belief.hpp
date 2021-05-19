#pragma once
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <solver_ipft/core/particle_belief.hpp>

namespace py = pybind11;

namespace solver_ipft {
namespace pyadapters {

class PyParticleBelief {
public:
    py::array_t<double> particles_;
    py::array_t<double> weights_;
    py::array_t<double> post_particles_; // particles before resampling
    py::array_t<double> post_weights_;   // weights before resampling
    int num_particles_;

public:
    PyParticleBelief() = default;
    virtual ~PyParticleBelief() = default;

    static PyParticleBelief createPyParticleBelief(const ParticleBelief& pb);
};

} // namespace pyadapters
} // namespace solver_ipft