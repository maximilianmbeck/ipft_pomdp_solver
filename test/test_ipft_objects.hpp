#pragma once
#include <gtest/gtest.h>

#include <solver_ipft/core/particle_belief.hpp>
#include <solver_ipft/core/solver.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/world.hpp>
#include <solver_ipft/problems/cont_lightdark.hpp>
#include <solver_ipft/util/debug.hpp>

namespace solver_ipft {
namespace test {

class IpftObjects : public ::testing::Test {
protected:
  Random *rand_;
  POMDP *model_;
  World *world_;
  Solver *solver_;

  IpftObjects() : model_(nullptr), world_(nullptr), solver_(nullptr) {
    this->rand_ = new Random();
  }

  virtual ~IpftObjects() {
    delete solver_;
    delete world_;
    delete model_;
    delete rand_;
  }

};

} // namespace test
} // namespace solver_ipft