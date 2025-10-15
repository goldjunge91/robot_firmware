#include <gtest/gtest.h>
#include <cmath>
#include "mocks/mock_motor_mgr.h"
#include "../src/MotorPID.h"

class MotorPIDTest : public ::testing::Test {
protected:
    void SetUp() override {
        motor_pid = new MotorPID(1, 2, 3, 4);
        motor_pid->configPID(0.1f, 0.01f, 0.005f);
    }
    void TearDown() override { delete motor_pid; }
    MotorPID* motor_pid;
};

TEST_F(MotorPIDTest, ConstructorWorks) {
    ASSERT_NE(motor_pid, nullptr);
}

TEST_F(MotorPIDTest, SetSpeedRPMWorks) {
    EXPECT_NO_THROW(motor_pid->setSpeedRPM(100.0f, true));
}

TEST_F(MotorPIDTest, PIDRespondsToError) {
    motor_pid->setSpeedRPM(100.0f, true);
    motor_pid->setMovingAvgRPM(0.0f);
    float throttle1 = motor_pid->doPID();
    motor_pid->setMovingAvgRPM(50.0f);
    float throttle2 = motor_pid->doPID();
    EXPECT_NE(throttle1, throttle2);
}
