#pragma once
#include <gtest/gtest.h>

#include <solver_ipft/core/particle_belief.hpp>
#include <solver_ipft/interface/pomdp.hpp>
#include <solver_ipft/interface/solver.hpp>
#include <solver_ipft/interface/world.hpp>
#include <solver_ipft/problems/cont_lightdark.hpp>
#include <solver_ipft/util/debug.hpp>

namespace solver_ipft {
namespace test {

class IpftObjects : public ::testing::Test {
protected:
    std::shared_ptr<Random> rand_;
    std::shared_ptr<POMDP> model_;
    std::shared_ptr<World> world_;
    std::shared_ptr<Solver> solver_;

    IpftObjects() : model_(nullptr), world_(nullptr), solver_(nullptr) {
        this->rand_ = std::make_shared<Random>();
    }

    ~IpftObjects() override = default;
};

} // namespace test
} // namespace solver_ipft