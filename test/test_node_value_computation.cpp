#include <gtest/gtest.h>

#include "test_ipft_objects.hpp"

namespace ipft {
namespace test {

// The fixture for testing class CldModel.
class NodeValueTest : public IpftObjects {
   protected:
    // You can remove any or all of the following functions if their bodies would
    // be empty.

    NodeValueTest() {
        // You can do set-up work for each test here.
        this->model_ = new cld::ContLightDark(this->rand_);
    }

    ~NodeValueTest() override {
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

// Test for IpftValue operations
TEST_F(NodeValueTest, IPFTValueOperations) {
    IpftValue zeroVal;
    EXPECT_DOUBLE_EQ(0.0, zeroVal.total());

    IpftValue oneOneVal(1.0, 1.0);
    // test total()
    EXPECT_DOUBLE_EQ(1.0 + Globals::config.inf_gather_constant_lambda * 1.0, oneOneVal.total());

    // test opertor+=
    IpftValue tempVal;  // == zeroVal
    tempVal += oneOneVal;
    EXPECT_DOUBLE_EQ(1.0, tempVal.getRawComponent(0));
    EXPECT_DOUBLE_EQ(1.0, tempVal.getRawComponent(1));

    // test operator* and copyconstructor
    IpftValue tempVal2;
    tempVal2 = oneOneVal * 5.0;
    EXPECT_DOUBLE_EQ(5.0, tempVal2.getRawComponent(0));
    EXPECT_DOUBLE_EQ(5.0, tempVal2.getRawComponent(1));

    // test update()
    // we have value1 = (stateVal, infVal) = (20,0.5)
    // update value1 with value2 (10,1.5) and count 3
    // expected result: stateVal = 20+(10-20)/(3+1) = 17.5, infVal = 0.5+(1.5-0.5)/(3+1) = 0.75
    IpftValue value1(20, 0.5);
    IpftValue value2(10, 1.5);
    value1.update(value2, 3);
    EXPECT_DOUBLE_EQ(17.5, value1.getRawComponent(0));
    EXPECT_DOUBLE_EQ(0.75, value1.getRawComponent(1));
}

}  // namespace test
}  // namespace ipft

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // INIT GLOG
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    // ::testing::InitGoogleTest(&argc, argv);

    FLAGS_colorlogtostderr = true;
    FLAGS_logtostderr = true;
    return RUN_ALL_TESTS();
}