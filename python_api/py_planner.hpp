#pragma once
#include <memory>
#include <solver_ipft/planner.hpp>
#include <vector>

#include "py_node.hpp"

namespace solver_ipft {
namespace pyadapters {

class PyPlanner : public Planner {
   protected:
    std::vector<std::shared_ptr<PyVNode>> search_trees_;

   public:
    PyPlanner() = default;
    virtual ~PyPlanner() = default;

    virtual void resetPlanner() override;

    virtual bool runStep() override;

    virtual std::shared_ptr<PyVNode> getSearchTreeOfStep(int i) const;

    int numSteps() const;
};

}  // namespace pyadapters
}  // namespace solver_ipft
