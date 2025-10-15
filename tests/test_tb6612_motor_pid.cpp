#include <gtest/gtest.h>
#include <cmath>
#include "mocks/mock_tb6612_motor_mgr.h"
#include "../src/TB6612MotorPID.h"

class TB6612MotorPIDTest : public ::testing::Test {
protected:
    void SetUp() override {
        motor_pid = new TB6612MotorPID(1, 2, 3, 4, 5);
        motor_pid->configPID(0.1f, 0.01f, 0.005f);
    }
    void TearDown() override { delete motor_pid; }
    TB6612MotorPID* motor_pid;
};

TEST_F(TB6612MotorPIDTest, ConstructorWorks) {
    ASSERT_NE(motor_pid, nullptr);
}

TEST_F(TB6612MotorPIDTest, ForwardDirectionSetsCorrectPins) {
    motor_pid->setSpeedRPM(100.0f, true);
    motor_pid->setMovingAvgRPM(0.0f);
    motor_pid->doPID();
    EXPECT_TRUE(motor_pid->getIN1State());
    EXPECT_FALSE(motor_pid->getIN2State());
}

TEST_F(TB6612MotorPIDTest, BrakeModeWorks) {
    motor_pid->brake();
    EXPECT_TRUE(motor_pid->getIN1State());
    EXPECT_TRUE(motor_pid->getIN2State());
    EXPECT_FLOAT_EQ(motor_pid->getThrottle(), 0.0f);
}
