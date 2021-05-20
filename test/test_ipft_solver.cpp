#include <gtest/gtest.h>

#include <limits>
#include <solver_ipft/core/pomdp_world.hpp>
#include <solver_ipft/util/util.hpp>

#include "test_ipft_objects.hpp"

namespace solver_ipft {
namespace test {

// The fixture for testing class Ipft.
class IpftSolverTest : public IpftObjects {
protected:
    IpftSolverTest() {
        this->model_ = std::make_shared<cld::ContLightDark>(this->rand_);
    }
};

TEST_F(IpftSolverTest, TestSingleSearch) { // NOLINT
    std::vector<double> numbers = {3.9433,  3.1378, 0.2911, -1.6948, 1.1413, 3.2896, 2.1801, 2.9403, 3.0115,  4.5241,
                                   -0.1786, 3.6636, 3.2806, 3.5872,  2.1970, 4.3526, 3.2905, 3.4663, -3.0771, 2.1328};

    std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
    auto initBel = std::make_unique<ParticleBelief>(states, false, this->model_, this->rand_);

    this->solver_ = std::make_shared<Ipft>(this->rand_,
                                           this->model_,
                                           std::move(initBel),
                                           std::make_unique<BeliefInformationPolicy>(this->model_, this->rand_));

    ValuedAction valuedAct = this->solver_->search();
    auto ss = this->solver_->getSearchStatistics();
    LOG(INFO) << ss->text();
    LOG(INFO) << valuedAct;

    // Expect action -3 (corresponds to action index 0)(from julia IPFT)
    EXPECT_EQ(0, valuedAct.action_);

    //? Result:
    // on multiple runs the solver always chooses -3
}

} // namespace test
} // namespace solver_ipft

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // INIT GLOG
    google::InitGoogleLogging(argv[0]); // NOLINT
    google::InstallFailureSignalHandler();
    // ::testing::InitGoogleTest(&argc, argv);

    FLAGS_colorlogtostderr = true;
    FLAGS_logtostderr = true;
    return RUN_ALL_TESTS();
}