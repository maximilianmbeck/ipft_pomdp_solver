// #include <gtest/gtest.h>

// #include "test_particle_data.hpp"

// namespace solver_ipft {

// namespace test {

// // The fixture for testing class RolloutPolicy and its child classes.
// class RolloutTest : public ParticleBeliefData {
// protected:
//   const int depth = 20;
//   const int N_r = 10000; // number of rollouts
//   const double deltaStateReward = 3.0;
//   const double deltaInfReward = 2.0;
// };

// TEST_F(RolloutTest, BeliefInformationPolicyTest) {
//     // Number of rollouts: 10000
//     // Belief: particle set of 20 equally weighted particles at 5.0
//     // expected results:
//     //                              state reward      information reward
//     //  mean(qvalVec, dims=2) = [-71.10636623102988; -16.65686187957277]
//     //   std(qvalVec, dims=2) = [46.98322507221085;    0.369056989054344]

//     RolloutPolicy* rp = new BeliefInformationPolicy(this->model_,
//     this->rand_); IpftValue qval; for (int i = 0; i < N_r; i++) {
//         IpftValue v = rp->rollout(allEq5->clone(), depth);
//         qval += v;
//     }

//     // expect somewhere around mean
//     double meanStateR = qval.getComponent(0) / N_r;
//     double meanInfR = qval.getComponent(1) / N_r;
//     LOG(INFO) << "mean state reward: " << meanStateR;
//     LOG(INFO) << "mean inf reward: " << meanInfR;
//     EXPECT_LE(std::abs(-71.11 - meanStateR), deltaStateReward);
//     EXPECT_LE(std::abs(-16.66 - meanInfR), deltaInfReward);

//     delete rp;
// }

// }  // namespace test
// }  // namespace solver_ipft

// int main(int argc, char** argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     // INIT GLOG
//     google::InitGoogleLogging(argv[0]);
//     google::InstallFailureSignalHandler();
//     // ::testing::InitGoogleTest(&argc, argv);

//     FLAGS_colorlogtostderr = true;
//     FLAGS_logtostderr = true;
//     return RUN_ALL_TESTS();
// }