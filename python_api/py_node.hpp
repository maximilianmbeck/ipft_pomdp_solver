#pragma once

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "py_particle_belief.hpp"

// #include <ipft/core/node.hpp>
// #include <ipft/python_adapters/py_particle_belief.hpp>

namespace py = pybind11;

namespace solver_ipft {
namespace pyadapters {
// TODO maybe inherit from enable_shared_from_this
class PyNode {
   public:
    int count_;
    int treelevel_;
    py::array_t<double> values_;

   public:
    static void convertNode(std::shared_ptr<PyNode>& pynode, const Node* node);
};

// forward declarations
class PyQNode;

class PyVNode : public PyNode, public std::enable_shared_from_this<PyVNode> {
   public:
    std::shared_ptr<PyQNode> parent_;
    py::list children_;  // contains multiple std::shared_ptr<PyQNode>
    py::array_t<double> observation_;
    PyParticleBelief belief_;  // TODO support general belief

   public:
    static std::shared_ptr<PyVNode> convertTree(const VNode* vroot);
    static std::shared_ptr<PyVNode> convertVNode(const VNode* vnode, const std::shared_ptr<PyQNode>& qparent);
};

class PyQNode : public PyNode, public std::enable_shared_from_this<PyQNode> {
   public:
    std::shared_ptr<PyVNode> parent_;
    py::list children_;  // contains multiple std::shared_ptr<PyVNode>
    py::array_t<double> action_;

   public:
    static std::shared_ptr<PyQNode> convertQNode(const QNode* qnode, const std::shared_ptr<PyVNode>& vparent);
};

}  // namespace pyadapters
}  // namespace solver_ipft