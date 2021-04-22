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
    // You can remove any or all of the following functions if their bodies would
    // be empty.

    ParticleBelief* belErr1Step;
    ParticleBelief* belErr2Step;
    ParticleBelief* belErr3Initial;

    IpftSolverTest() {
        // You can do set-up work for each test here.

        //! run all these tests with CLD::createStartState() = -5.0
        this->model_ = new cld::ContLightDark(this->rand_);
        this->world_ = new POMDPWorld(model_);

        // belErr1Step
        std::vector<double> numbers = {9.3953,
                                       9.8427,
                                       10.1268,
                                       9.6946,
                                       9.2815,
                                       10.4726,
                                       10.7598,
                                       3.0242,  // this is an outlier
                                       10.6167,
                                       10.2247,
                                       9.0881,
                                       11.5099,
                                       9.6757,
                                       9.6167,
                                       8.9347,
                                       10.1384,
                                       9.6376,
                                       9.6736,
                                       10.7702,
                                       9.7005};

        std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
        belErr1Step = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());

        // belErr2Step
        numbers = {-1.2199,
                   2.5795,
                   1.7870,
                   -1.1903,
                   -0.2870,
                   -4.4878,
                   2.1491,
                   0.8276,
                   -0.6552,
                   -7.8526,
                   -5.9127,
                   0.5438,
                   0.6360,
                   -3.9846,
                   -1.0641,
                   -10.0696,
                   1.1477,
                   3.0037,
                   -4.2193,
                   -0.2440};
        states = debug::doubleVec2StateVec(numbers, this->model_);
        belErr2Step = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());
    }

    ~IpftSolverTest() override {
        // You can do clean-up work that doesn't throw exceptions here.
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

    // Class members declared here can be used by all tests in the test suite
};

// TEST_F(IpftSolverTest, TestErr1ParticleFilterStep8) {
//     State* worldState = this->model_->allocateState();
//     worldState->set(9.2871, 0);
//     Action act = static_cast<Action>(cld::CLDAction::ZERO);
//     Observation* obs = this->model_->observation(*worldState);
//     this->model_->freeState(worldState);
//     LOG(INFO) << "observation: " << *obs;
//     double stateReward = belErr1Step->update(act, *obs);
//     LOG(INFO) << belErr1Step->detailedText();
//     LOG(INFO) << "reward: " << stateReward;
//     delete belErr1Step;
// }

//* Results of julia IPFT (one exemplary round respectively)
// IF action 1 taken
// [ Info: [UCB] [-3 Act]: [UCB1: -5.362977163442629] = [V: -30.950524221577886 (-69.39759599840215|38.44707177682427)] + [ucbT: 25.587547058135257][C:92]
// [ Info: [UCB] [-1 Act]: [UCB1: -4.913023781499213] = [V: -30.50057083963447 (-78.79408451265692|48.29351367302245)] + [ucbT: 25.587547058135257][C:92]
// [ Info: [UCB] [0 Act]: [UCB1: -6.125738863583621] = [V: -76.97444852579879 (-100.0|23.025551474201205)] + [ucbT: 70.84870966221517][C:12]
// [ Info: [UCB] [1 Act]: [UCB1: -4.980325243756798] = [V: -23.073453282494135 (-76.4204620320547|53.347008749560565)] + [ucbT: 18.093128038737337][C:184]
// [ Info: [UCB] [3 Act]: [UCB1: -4.838663971375539] = [V: -47.56204339508372 (-82.67045436776557|35.10841097268185)] + [ucbT: 42.72337942370818][C:33]
// IF action -1 taken
// [ Info: [UCB] [-3 Act]: [UCB1: -7.045679060638424] = [V: -29.62343742902221 (-69.21884795036513|39.59541052134292)] + [ucbT: 22.577758368383787][C:127]
// [ Info: [UCB] [-1 Act]: [UCB1: -0.9924029603134557] = [V: -19.549231571470166 (-71.01825335899031|51.46902178752014)] + [ucbT: 18.55682861115671][C:188]
// [ Info: [UCB] [0 Act]: [UCB1: -6.81535960243319] = [V: -72.51107583189153 (-100.0|27.488924168108465)] + [ucbT: 65.69571622945834][C:15]
// [ Info: [UCB] [1 Act]: [UCB1: -6.903366530566842] = [V: -22.246580246737345 (-76.12335979683704|53.87677955009969)] + [ucbT: 15.343213716170503][C:275]
// [ Info: [UCB] [3 Act]: [UCB1: -6.894436157727014] = [V: -45.69593066368487 (-82.59169495595067|36.8957642922658)] + [ucbT: 38.801494505957855][C:43]
// TEST_F(IpftSolverTest, TestErr1SolverSearchStep8) {
//     this->solver_ = new Ipft(this->model_, belErr1Step, this->rand_, new BeliefInformationPolicy(this->model_, this->rand_));

//     ValuedAction valuedAct = this->solver_->search();
//     SearchStatistics* ss = this->solver_->getSearchStatistics();
//     LOG(INFO) << ss->text();
//     LOG(INFO) << belErr1Step->detailedText();
//     LOG(INFO) << valuedAct;
//     delete ss;

//     // EXPECT_NE(static_cast<int>(cld::CLDAction::NEG3), valuedAct.action_);
//     EXPECT_NE(static_cast<int>(cld::CLDAction::ZERO), valuedAct.action_);
//     EXPECT_NE(static_cast<int>(cld::CLDAction::POS3), valuedAct.action_);

//     //? Result:
//     // the solver switches between -1 and 1 action on multiple runs
// }

//! In this example c++ implementation performs equal to julia code!
//* Results of julia IPFT (one exemplary round)
// [ Info: [UCB] [-3 Act]: [UCB1: 47.2069704106324] = [V: 38.2247928321901 (28.11229761744563|10.112495214744468)] + [ucbT: 8.982177578442304][C:843]
// [ Info: [UCB] [-1 Act]: [UCB1: 43.433565651862516] = [V: -19.8179539042697 (-43.456906967632825|23.638953063363125)] + [ucbT: 63.251519556132216][C:17]
// [ Info: [UCB] [0 Act]: [UCB1: 45.09829477531684] = [V: -61.369877577360306 (-71.48178467631043|10.11190709895012)] + [ucbT: 106.46817235267714][C:6]
// [ Info: [UCB] [1 Act]: [UCB1: 41.92745259898567] = [V: -17.90249516734541 (-40.6119701352586|22.70947496791319)] + [ucbT: 59.82994776633108][C:19]
// [ Info: [UCB] [3 Act]: [UCB1: 43.55983176119865] = [V: -26.139948083051006 (-56.522833889549844|30.382885806498837)] + [ucbT: 69.69977984424966][C:14]
// [ Info: [SIM] select [-3 Act]
TEST_F(IpftSolverTest, TestEval1txtRound10Step4) {
    std::vector<double> numbers = {3.9433,
                                   3.1378,
                                   0.2911,
                                   -1.6948,
                                   1.1413,
                                   3.2896,
                                   2.1801,
                                   2.9403,
                                   3.0115,
                                   4.5241,
                                   -0.1786,
                                   3.6636,
                                   3.2806,
                                   3.5872,
                                   2.1970,
                                   4.3526,
                                   3.2905,
                                   3.4663,
                                   -3.0771,
                                   2.1328};
    std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
    ParticleBelief* initBel = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());

    this->solver_ = new Ipft(this->model_, initBel, this->rand_, new BeliefInformationPolicy(this->model_, this->rand_));

    ValuedAction valuedAct = this->solver_->search();
    SearchStatistics* ss = this->solver_->getSearchStatistics();
    LOG(INFO) << ss->text();
    LOG(INFO) << valuedAct;
    delete ss;

    // Expect action -3 (corresponds to action index 0)(from julia IPFT)
    EXPECT_EQ(static_cast<int>(cld::CLDAction::NEG3), valuedAct.action_);

    //? Result:
    // on multiple runs the solver always chooses -3
}

//* Results of julia IPFT (one exemplary round)
// [ Info: [UCB] [-3 Act]: [UCB1: 50.93705900385379] = [V: -29.000914474164897 (-27.087096475102317|-1.9138179990625785)] + [ucbT: 79.93797347801869][C:11]
// [ Info: [UCB] [-1 Act]: [UCB1: 53.39682830292425] = [V: -26.541145175094442 (-24.635740197761624|-1.9054049773328186)] + [ucbT: 79.93797347801869][C:11]
// [ Info: [UCB] [0 Act]: [UCB1: 53.439197496392936] = [V: 45.43812313538836 (45.6177156909735|-0.1795925555851369)] + [ucbT: 8.001074361004576][C:1098]
// [ Info: [UCB] [1 Act]: [UCB1: 43.63427478417502] = [V: -74.93290080966007 (-77.87741675175168|2.944515942091611)] + [ucbT: 118.56717559383509][C:5]
// [ Info: [UCB] [3 Act]: [UCB1: 41.38137882637069] = [V: -91.18075343761394 (-93.1975|2.0167465623860616)] + [ucbT: 132.56213226398464][C:4]
// If the c++ implementation also takes action 0, the action values on treelevel 0 are comparable
// TEST_F(IpftSolverTest, TestAction0TakenWrong) {
//     std::vector<double> numbers = {
//         1.3683,
//         0.7596,
//         1.7421,
//         1.0755,
//         1.2364,
//         1.4864,
//         1.3303,
//         0.0859,
//         1.8493,
//         2.2353,
//         0.8299,
//         1.2075,
//         1.8157,
//         1.8321,
//         0.2949,
//         1.4529,
//         1.2495,
//         1.1323,
//         1.2289,
//         1.4052};

//     std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
//     ParticleBelief* initBel = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());

//     this->solver_ = new Ipft(this->model_, initBel, this->rand_, new BeliefInformationPolicy(this->model_, this->rand_));

//     ValuedAction valuedAct = this->solver_->search();
//     SearchStatistics* ss = this->solver_->getSearchStatistics();
//     LOG(INFO) << initBel->detailedText();
//     LOG(INFO) << ss->text();
//     LOG(INFO) << valuedAct;
//     delete ss;

//     // Expect action 0 (corresponds to action index 2) or -1 (action index 1) (from julia IPFT)
//     EXPECT_NE(static_cast<int>(cld::CLDAction::NEG3), valuedAct.action_);
//     EXPECT_NE(static_cast<int>(cld::CLDAction::POS1), valuedAct.action_);
//     EXPECT_NE(static_cast<int>(cld::CLDAction::POS3), valuedAct.action_);

//     //? Result:
//     // on multiple runs action -1 (which is actually the correct one, from looking at the particles) is taken slightly more often than action 0
// }


}  // namespace test
}  // namespace solver_ipft

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // INIT GLOG
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    // ::testing::InitGoogleTest(&argc, argv);

    FLAGS_colorlogtostderr = true;
    FLAGS_logtostderr = true;
    return RUN_ALL_TESTS();
}