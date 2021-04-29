#pragma once
#include "py_planner.hpp"
#include <solver_ipft/core/pomdp_world.hpp>
#include <solver_ipft/problems/cont_lightdark.hpp>
namespace solver_ipft {
namespace pyadapters {

class PyPlannerCld : public PyPlanner {
   public:
    PyPlannerCld() {
        initializePlanner();
    }
    virtual ~PyPlannerCld() = default;
    virtual POMDP *initializeModel() override {
        POMDP *model = new cld::ContLightDark(this->rand_);
        return model;
    }

    virtual World *initializePOMDPWorld(POMDP *model) override {
        World *world = new POMDPWorld(model);
        return world;
    }

    virtual Solver *initializeSolver(POMDP *model, Belief *belief) override {
        Solver *solver = new Ipft(model, belief, this->rand_, new BeliefInformationPolicy(model, this->rand_));
        return solver;
    }
};

}  // namespace pyadapters
}  // namespace solver_ipft
