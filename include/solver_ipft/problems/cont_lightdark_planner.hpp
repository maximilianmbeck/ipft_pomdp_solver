#include <glog/logging.h>

#include <iostream>
#include <solver_ipft/core/pomdp_world.hpp>
#include <solver_ipft/evaluator.hpp>
#include <solver_ipft/planner.hpp>
#include <solver_ipft/problems/cont_lightdark.hpp>
#include <solver_ipft/solver/ipft.hpp>
#include <solver_ipft/util/random.hpp>

namespace solver_ipft {

namespace cld {

class MyPlanner : public Planner {
public:
  MyPlanner() {}

  POMDP *initializeModel() override {
    POMDP *model = new ContLightDark(this->rand_);
    return model;
  }

  World *initializePOMDPWorld(POMDP *model) override {
    World *world = new POMDPWorld(model);
    return world;
  }

  Solver *initializeSolver(POMDP *model, Belief *belief) override {
    Solver *solver = new Ipft(model, belief, this->rand_,
                              new BeliefInformationPolicy(model, this->rand_));
    return solver;
  }
};

} // namespace cld
} // namespace solver_ipft