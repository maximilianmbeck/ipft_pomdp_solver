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
  MyPlanner() = default;

  std::unique_ptr<POMDP> initializeModel() override {
    return std::make_unique<ContLightDark>(this->rand_);
  }

  std::unique_ptr<World>
  initializePOMDPWorld(std::shared_ptr<POMDP> model) override {
    return std::make_unique<POMDPWorld>(model);
  }

  std::unique_ptr<Solver>
  initializeSolver(std::shared_ptr<POMDP> model,
                   std::unique_ptr<Belief> &&belief) override {

    return std::make_unique<Ipft>(
        this->rand_, model, std::move(belief),
        std::make_unique<BeliefInformationPolicy>(model, this->rand_));
  }
};

} // namespace cld
} // namespace solver_ipft