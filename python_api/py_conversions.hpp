#pragma once
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <solver_ipft/core/solver.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/spaces.hpp>

namespace py = pybind11;

namespace solver_ipft {
namespace pyadapters {

/* -------------------------------------------------------------------------- */
/*            Conversion functions from ipft types to python types            */
/* -------------------------------------------------------------------------- */

py::array_t<double> point2NpArray(const Point& p);

py::array_t<double> action2NpArray(const Action& act, const POMDP* model);

py::array_t<double> value2NpArray(const Value& value);

// particle np array: one row corresponds to a state, columns correspond to the dimensions
void particleSet2NpArrays(const std::vector<State*>& particleSet, py::array_t<double>& particles, py::array_t<double>& weights);

// particle np array: one row corresponds to a state, columns correspond to the dimensions
std::vector<State*> npArray2ParticleSet(const py::array_t<double>& unweightedParticleArray, const POMDP* model);

}  // namespace pyadapters
}  // namespace solver_ipft