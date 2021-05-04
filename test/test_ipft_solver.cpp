// #include <gtest/gtest.h>

// #include <limits>
// #include <solver_ipft/core/pomdp_world.hpp>
// #include <solver_ipft/util/util.hpp>

// #include "test_ipft_objects.hpp"

// namespace solver_ipft {
// namespace test {

// // The fixture for testing class Ipft.
// class IpftSolverTest : public IpftObjects {
// protected:
//   // You can remove any or all of the following functions if their bodies
//   would
//   // be empty.

//   ParticleBelief *belErr1Step;
//   ParticleBelief *belErr2Step;
//   ParticleBelief *belErr3Initial;

//   IpftSolverTest() {
//     // You can do set-up work for each test here.

//     //! run all these tests with CLD::createStartState() = -5.0
//     this->model_ = new cld::ContLightDark(this->rand_);
//     this->world_ = new POMDPWorld(model_);

//     // belErr1Step
//     std::vector<double> numbers = {
//         9.3953,  9.8427,  10.1268, 9.6946,  9.2815, 10.4726, 10.7598,
//         3.0242, // this is an outlier
//         10.6167, 10.2247, 9.0881,  11.5099, 9.6757, 9.6167,  8.9347,
//         10.1384, 9.6376,  9.6736,  10.7702, 9.7005};

//     std::vector<State *> states =
//         debug::doubleVec2StateVec(numbers, this->model_);
//     belErr1Step = new ParticleBelief(states, false, this->model_,
//     this->rand_,
//                                      new NoReinvigoration());

//     // belErr2Step
//     numbers = {-1.2199, 2.5795,   1.7870,  -1.1903, -0.2870, -4.4878, 2.1491,
//                0.8276,  -0.6552,  -7.8526, -5.9127, 0.5438,  0.6360, -3.9846,
//                -1.0641, -10.0696, 1.1477,  3.0037,  -4.2193, -0.2440};
//     states = debug::doubleVec2StateVec(numbers, this->model_);
//     belErr2Step = new ParticleBelief(states, false, this->model_,
//     this->rand_,
//                                      new NoReinvigoration());
//   }

//   ~IpftSolverTest() override {
//     // You can do clean-up work that doesn't throw exceptions here.
//   }

//   // If the constructor and destructor are not enough for setting up
//   // and cleaning up each test, you can define the following methods:

//   void SetUp() override {
//     // Code here will be called immediately after the constructor (right
//     // before each test).
//   }

//   void TearDown() override {
//     // Code here will be called immediately after each test (right
//     // before the destructor).
//   }

//   // Class members declared here can be used by all tests in the test suite
// };

// //* Results of julia IPFT (one exemplary round)
// // [ Info: [UCB] [-3 Act]: [UCB1: 47.2069704106324] = [V: 38.2247928321901
// // (28.11229761744563|10.112495214744468)] + [ucbT: 8.982177578442304][C:843]
// [
// // Info: [UCB] [-1 Act]: [UCB1: 43.433565651862516] = [V: -19.8179539042697
// // (-43.456906967632825|23.638953063363125)] +
// [ucbT: 63.251519556132216][C:17]
// // [ Info: [UCB] [0 Act]: [UCB1: 45.09829477531684] = [V: -61.369877577360306
// // (-71.48178467631043|10.11190709895012)] + [ucbT: 106.46817235267714][C:6]
// [
// // Info: [UCB] [1 Act]: [UCB1: 41.92745259898567] = [V: -17.90249516734541
// // (-40.6119701352586|22.70947496791319)] + [ucbT: 59.82994776633108][C:19] [
// // Info: [UCB] [3 Act]: [UCB1: 43.55983176119865] = [V: -26.139948083051006
// // (-56.522833889549844|30.382885806498837)] +
// [ucbT: 69.69977984424966][C:14]
// // [ Info: [SIM] select [-3 Act]
// TEST_F(IpftSolverTest, TestEval1txtRound10Step4) {
//   std::vector<double> numbers = {3.9433,  3.1378, 0.2911, -1.6948, 1.1413,
//                                  3.2896,  2.1801, 2.9403, 3.0115,  4.5241,
//                                  -0.1786, 3.6636, 3.2806, 3.5872,  2.1970,
//                                  4.3526,  3.2905, 3.4663, -3.0771, 2.1328};
//   std::vector<State *> states =
//       debug::doubleVec2StateVec(numbers, this->model_);
//   ParticleBelief *initBel = new ParticleBelief(
//       states, false, this->model_, this->rand_, new NoReinvigoration());

//   this->solver_ =
//       new Ipft(this->model_, initBel, this->rand_,
//                new BeliefInformationPolicy(this->model_, this->rand_));

//   ValuedAction valuedAct = this->solver_->search();
//   SearchStatistics *ss = this->solver_->getSearchStatistics();
//   LOG(INFO) << ss->text();
//   LOG(INFO) << valuedAct;
//   delete ss;

//   // Expect action -3 (corresponds to action index 0)(from julia IPFT)
//   EXPECT_EQ(static_cast<int>(cld::CLDAction::NEG3), valuedAct.action_);

//   //? Result:
//   // on multiple runs the solver always chooses -3
// }

// } // namespace test
// } // namespace solver_ipft

// int main(int argc, char **argv) {
//   ::testing::InitGoogleTest(&argc, argv);
//   // INIT GLOG
//   google::InitGoogleLogging(argv[0]);
//   google::InstallFailureSignalHandler();
//   // ::testing::InitGoogleTest(&argc, argv);

//   FLAGS_colorlogtostderr = true;
//   FLAGS_logtostderr = true;
//   return RUN_ALL_TESTS();
// }