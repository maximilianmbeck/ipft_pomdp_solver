#pragma once
#include <gtest/gtest.h>

#include <ipft/core/particle_belief.hpp>
#include <ipft/core/solver.hpp>
#include <ipft/interface/pomdp.hpp>
#include <ipft/interface/world.hpp>
#include <ipft/problems/cont_lightdark.hpp>
#include <ipft/util/debug.hpp>

namespace ipft {
namespace test {

class IpftObjects : public ::testing::Test {
   protected:
    Random* rand_;
    POMDP* model_;
    World* world_;
    Solver* solver_;

    IpftObjects() : model_(nullptr), world_(nullptr), solver_(nullptr) {
        this->rand_ = new Random();
    }

    virtual ~IpftObjects() {
        if (solver_ != nullptr)
            delete solver_;
        if (world_ != nullptr)
            delete world_;
        if (model_ != nullptr)
            delete model_;
        if (rand_ != nullptr)
            delete rand_;
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    void SetUp() override {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
};

}  // namespace test
}  // namespace ipft