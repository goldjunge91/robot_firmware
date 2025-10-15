/**
 * @file test_mecanum_kinematics.cpp
 * @brief Comprehensive unit tests for mecanum wheel inverse kinematics
 * @date 2025-10-11
 * 
 * Tests the mecanum wheel kinematics calculations used in RobotController
 * to convert Twist commands (linear x, y and angular z) into individual
 * wheel velocities for a 4-wheel mecanum drive system.
 * 
 * Key formulas tested:
 * - Front Left:  vx - vy - ω*L
 * - Front Right: vx + vy + ω*L
 * - Rear Left:   vx + vy - ω*L
 * - Rear Right:  vx - vy + ω*L
 * 
 * where L = sqrt((wheelbase/2)² + (track/2)²)
 */

#include <gtest/gtest.h>
#include <cmath>
#include <array>

// Physical constants matching RobotController.h
namespace MecanumConstants {
    constexpr double kWheelRadius = 0.065;      // meters
    constexpr double kWheelsSeparation = 0.204; // meters (left-right distance)
    constexpr double kWheelsOffset = 0.010;     // meters
    constexpr double kEpsilon = 1e-6;           // for float comparisons
}

/**
 * @class MecanumKinematics
 * @brief Mock implementation of mecanum wheel kinematics for testing
 * 
 * This class implements the inverse kinematics transformation used
 * by the robot controller to convert body-frame velocities into
 * wheel velocities.
 */
class MecanumKinematics {
public:
    /**
     * @brief Wheel velocity result structure
     */
    struct WheelVelocities {
        double front_left;   ///< Front left wheel velocity (rad/s)
        double front_right;  ///< Front right wheel velocity (rad/s)
        double rear_left;    ///< Rear left wheel velocity (rad/s)
        double rear_right;   ///< Rear right wheel velocity (rad/s
    };

    /**
     * @brief Calculate wheel velocities from body twist
     * 
     * @param vx Linear velocity in x direction (m/s)
     * @param vy Linear velocity in y direction (m/s)
     * @param omega Angular velocity around z axis (rad/s)
     * @return WheelVelocities Calculated wheel speeds
     */
    static WheelVelocities inverseMecanum(double vx, double vy, double omega) {
        using namespace MecanumConstants;
        
        // Calculate L: distance from robot center to wheel
        const double wheelbase = kWheelsSeparation;
        const double track = kWheelsSeparation; // assuming square footprint
        const double L = std::sqrt((wheelbase/2.0) * (wheelbase/2.0) + 
                                   (track/2.0) * (track/2.0));
        
        WheelVelocities wheels;
        
        // Mecanum inverse kinematics
        wheels.front_left  = (vx - vy - omega * L) / kWheelRadius;
        wheels.front_right = (vx + vy + omega * L) / kWheelRadius;
        wheels.rear_left   = (vx + vy - omega * L) / kWheelRadius;
        wheels.rear_right  = (vx - vy + omega * L) / kWheelRadius;
        
        return wheels;
    }

    /**
     * @brief Calculate body twist from wheel velocities (forward kinematics)
     * 
     * @param wheels Wheel velocities in rad/s
     * @return std::array<double, 3> Body velocities [vx, vy, omega]
     */
    static std::array<double, 3> forwardMecanum(const WheelVelocities& wheels) {
        using namespace MecanumConstants;
        
        const double wheelbase = kWheelsSeparation;
        const double track = kWheelsSeparation;
        const double L = std::sqrt((wheelbase/2.0) * (wheelbase/2.0) + 
                                   (track/2.0) * (track/2.0));
        
        // Convert wheel speeds back to linear velocities
        const double vfl = wheels.front_left  * kWheelRadius;
        const double vfr = wheels.front_right * kWheelRadius;
        const double vrl = wheels.rear_left   * kWheelRadius;
        const double vrr = wheels.rear_right  * kWheelRadius;
        
        // Forward kinematics (inverse of the transformation matrix)
        const double vx = (vfl + vfr + vrl + vrr) / 4.0;
        const double vy = (-vfl + vfr + vrl - vrr) / 4.0;
        const double omega = (-vfl + vfr - vrl + vrr) / (4.0 * L);
        
        return {vx, vy, omega};
    }
};

/**
 * @class MecanumKinematicsTest
 * @brief Test fixture for mecanum kinematics tests
 */
class MecanumKinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Test setup if needed
    }

    void TearDown() override {
        // Cleanup if needed
    }

    /**
     * @brief Helper to compare doubles with epsilon tolerance
     */
    void expectNear(double actual, double expected, double epsilon = MecanumConstants::kEpsilon) {
        EXPECT_NEAR(actual, expected, epsilon);
    }
};

// ============================================================================
// BASIC MOTION TESTS
// ============================================================================

/**
 * @test Forward motion should produce equal positive velocities on all wheels
 */
TEST_F(MecanumKinematicsTest, ForwardMotionProducesEqualWheelSpeeds) {
    const double vx = 0.5;  // 0.5 m/s forward
    const double vy = 0.0;
    const double omega = 0.0;

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // All wheels should have same speed for forward motion
    expectNear(wheels.front_left, wheels.front_right);
    expectNear(wheels.front_left, wheels.rear_left);
    expectNear(wheels.front_left, wheels.rear_right);
    
    // Wheels should be rotating forward (positive)
    EXPECT_GT(wheels.front_left, 0.0);
}

/**
 * @test Backward motion should produce equal negative velocities on all wheels
 */
TEST_F(MecanumKinematicsTest, BackwardMotionProducesEqualNegativeWheelSpeeds) {
    const double vx = -0.3;  // 0.3 m/s backward
    const double vy = 0.0;
    const double omega = 0.0;

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // All wheels should have same speed for backward motion
    expectNear(wheels.front_left, wheels.front_right);
    expectNear(wheels.front_left, wheels.rear_left);
    expectNear(wheels.front_left, wheels.rear_right);
    
    // Wheels should be rotating backward (negative)
    EXPECT_LT(wheels.front_left, 0.0);
}

/**
 * @test Leftward strafing should produce diagonal wheel pattern
 */
TEST_F(MecanumKinematicsTest, LeftwardStrafeProducesCorrectPattern) {
    const double vx = 0.0;
    const double vy = 0.4;  // 0.4 m/s left
    const double omega = 0.0;

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // For left strafe: FL and RR should be negative, FR and RL should be positive
    EXPECT_LT(wheels.front_left, 0.0);
    EXPECT_GT(wheels.front_right, 0.0);
    EXPECT_GT(wheels.rear_left, 0.0);
    EXPECT_LT(wheels.rear_right, 0.0);
    
    // Opposite wheels should have equal magnitude
    expectNear(std::abs(wheels.front_left), std::abs(wheels.rear_right));
    expectNear(std::abs(wheels.front_right), std::abs(wheels.rear_left));
}

/**
 * @test Rightward strafing should produce opposite diagonal pattern
 */
TEST_F(MecanumKinematicsTest, RightwardStrafeProducesCorrectPattern) {
    const double vx = 0.0;
    const double vy = -0.4;  // 0.4 m/s right
    const double omega = 0.0;

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // For right strafe: FL and RR positive, FR and RL negative
    EXPECT_GT(wheels.front_left, 0.0);
    EXPECT_LT(wheels.front_right, 0.0);
    EXPECT_LT(wheels.rear_left, 0.0);
    EXPECT_GT(wheels.rear_right, 0.0);
}

/**
 * @test Rotation in place (counter-clockwise) produces correct wheel pattern
 */
TEST_F(MecanumKinematicsTest, CounterClockwiseRotationProducesCorrectPattern) {
    const double vx = 0.0;
    const double vy = 0.0;
    const double omega = 1.0;  // 1 rad/s CCW

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // For CCW rotation: left wheels negative, right wheels positive
    EXPECT_LT(wheels.front_left, 0.0);
    EXPECT_GT(wheels.front_right, 0.0);
    EXPECT_LT(wheels.rear_left, 0.0);
    EXPECT_GT(wheels.rear_right, 0.0);
    
    // Front and rear wheels should have equal magnitude on same side
    expectNear(std::abs(wheels.front_left), std::abs(wheels.rear_left));
    expectNear(std::abs(wheels.front_right), std::abs(wheels.rear_right));
}

/**
 * @test Rotation in place (clockwise) produces opposite pattern
 */
TEST_F(MecanumKinematicsTest, ClockwiseRotationProducesCorrectPattern) {
    const double vx = 0.0;
    const double vy = 0.0;
    const double omega = -0.8;  // 0.8 rad/s CW

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // For CW rotation: left wheels positive, right wheels negative
    EXPECT_GT(wheels.front_left, 0.0);
    EXPECT_LT(wheels.front_right, 0.0);
    EXPECT_GT(wheels.rear_left, 0.0);
    EXPECT_LT(wheels.rear_right, 0.0);
}

// ============================================================================
// COMBINED MOTION TESTS
// ============================================================================

/**
 * @test Combined forward and left strafe (diagonal motion)
 */
TEST_F(MecanumKinematicsTest, DiagonalForwardLeftMotion) {
    const double vx = 0.3;   // Forward
    const double vy = 0.3;   // Left
    const double omega = 0.0;

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // Front-right and rear-left should have highest speeds
    EXPECT_GT(wheels.front_right, wheels.front_left);
    EXPECT_GT(wheels.rear_left, wheels.rear_right);
}

/**
 * @test Forward motion with rotation (arc turn)
 */
TEST_F(MecanumKinematicsTest, ForwardWithRotation) {
    const double vx = 0.5;    // Forward
    const double vy = 0.0;
    const double omega = 0.5; // CCW rotation

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // Right wheels should spin faster than left (turning left)
    EXPECT_GT(wheels.front_right, wheels.front_left);
    EXPECT_GT(wheels.rear_right, wheels.rear_left);
    
    // All wheels should still be moving forward
    EXPECT_GT(wheels.front_left, 0.0);
    EXPECT_GT(wheels.front_right, 0.0);
    EXPECT_GT(wheels.rear_left, 0.0);
    EXPECT_GT(wheels.rear_right, 0.0);
}

/**
 * @test Complex motion: forward + left strafe + rotation
 */
TEST_F(MecanumKinematicsTest, ComplexThreeComponentMotion) {
    const double vx = 0.4;    // Forward
    const double vy = 0.2;    // Left
    const double omega = 0.3; // CCW rotation

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // Verify no NaN or infinity values
    EXPECT_FALSE(std::isnan(wheels.front_left));
    EXPECT_FALSE(std::isnan(wheels.front_right));
    EXPECT_FALSE(std::isnan(wheels.rear_left));
    EXPECT_FALSE(std::isnan(wheels.rear_right));
    
    EXPECT_FALSE(std::isinf(wheels.front_left));
    EXPECT_FALSE(std::isinf(wheels.front_right));
    EXPECT_FALSE(std::isinf(wheels.rear_left));
    EXPECT_FALSE(std::isinf(wheels.rear_right));
}

// ============================================================================
// EDGE CASE TESTS
// ============================================================================

/**
 * @test Zero velocity command produces zero wheel speeds
 */
TEST_F(MecanumKinematicsTest, ZeroVelocityProducesZeroWheelSpeeds) {
    const double vx = 0.0;
    const double vy = 0.0;
    const double omega = 0.0;

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    expectNear(wheels.front_left, 0.0);
    expectNear(wheels.front_right, 0.0);
    expectNear(wheels.rear_left, 0.0);
    expectNear(wheels.rear_right, 0.0);
}

/**
 * @test Maximum safe velocity doesn't produce unrealistic wheel speeds
 */
TEST_F(MecanumKinematicsTest, MaximumVelocityProducesReasonableWheelSpeeds) {
    const double vx = 1.0;  // 1 m/s (fast for indoor robot)
    const double vy = 0.0;
    const double omega = 0.0;

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // Wheel speeds should be reasonable (< 100 rad/s for typical motors)
    EXPECT_LT(std::abs(wheels.front_left), 100.0);
    EXPECT_LT(std::abs(wheels.front_right), 100.0);
    EXPECT_LT(std::abs(wheels.rear_left), 100.0);
    EXPECT_LT(std::abs(wheels.rear_right), 100.0);
}

/**
 * @test Extreme rotation rate
 */
TEST_F(MecanumKinematicsTest, ExtremeRotationRateIsHandled) {
    const double vx = 0.0;
    const double vy = 0.0;
    const double omega = 5.0;  // 5 rad/s is quite fast

    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    // Should not produce NaN or infinity
    EXPECT_FALSE(std::isnan(wheels.front_left));
    EXPECT_FALSE(std::isinf(wheels.front_left));
}

// ============================================================================
// FORWARD KINEMATICS CONSISTENCY TESTS
// ============================================================================

/**
 * @test Inverse followed by forward kinematics returns original velocities
 */
TEST_F(MecanumKinematicsTest, InverseForwardConsistency) {
    const double vx_orig = 0.5;
    const double vy_orig = 0.3;
    const double omega_orig = 0.4;

    // Inverse kinematics: body twist -> wheel velocities
    auto wheels = MecanumKinematics::inverseMecanum(vx_orig, vy_orig, omega_orig);
    
    // Forward kinematics: wheel velocities -> body twist
    auto body = MecanumKinematics::forwardMecanum(wheels);

    // Should get back original velocities (within epsilon)
    expectNear(body[0], vx_orig, 1e-5);
    expectNear(body[1], vy_orig, 1e-5);
    expectNear(body[2], omega_orig, 1e-5);
}

/**
 * @test Multiple round-trip conversions maintain consistency
 */
TEST_F(MecanumKinematicsTest, MultipleRoundTripConsistency) {
    double vx = 0.6;
    double vy = -0.2;
    double omega = 0.5;

    for (int i = 0; i < 10; ++i) {
        auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);
        auto body = MecanumKinematics::forwardMecanum(wheels);
        
        vx = body[0];
        vy = body[1];
        omega = body[2];
    }

    // After 10 round trips, should still be close to original
    expectNear(vx, 0.6, 1e-4);
    expectNear(vy, -0.2, 1e-4);
    expectNear(omega, 0.5, 1e-4);
}

// ============================================================================
// PARAMETERIZED TESTS FOR MULTIPLE SCENARIOS
// ============================================================================

/**
 * @class MecanumKinematicsParamTest
 * @brief Parameterized test fixture for testing multiple velocity combinations
 */
class MecanumKinematicsParamTest : public ::testing::TestWithParam<std::tuple<double, double, double>> {
protected:
    void SetUp() override {
        auto params = GetParam();
        vx = std::get<0>(params);
        vy = std::get<1>(params);
        omega = std::get<2>(params);
    }

    double vx, vy, omega;
};

/**
 * @test Parameterized test ensuring no NaN/Inf for various inputs
 */
TEST_P(MecanumKinematicsParamTest, NoNaNOrInfProduced) {
    auto wheels = MecanumKinematics::inverseMecanum(vx, vy, omega);

    EXPECT_FALSE(std::isnan(wheels.front_left));
    EXPECT_FALSE(std::isnan(wheels.front_right));
    EXPECT_FALSE(std::isnan(wheels.rear_left));
    EXPECT_FALSE(std::isnan(wheels.rear_right));
    
    EXPECT_FALSE(std::isinf(wheels.front_left));
    EXPECT_FALSE(std::isinf(wheels.front_right));
    EXPECT_FALSE(std::isinf(wheels.rear_left));
    EXPECT_FALSE(std::isinf(wheels.rear_right));
}

/**
 * @brief Test data for parameterized tests
 * Format: {vx, vy, omega}
 */
INSTANTIATE_TEST_SUITE_P(
    VariousVelocityCombinations,
    MecanumKinematicsParamTest,
    ::testing::Values(
        std::make_tuple(0.5, 0.0, 0.0),    // Forward only
        std::make_tuple(0.0, 0.5, 0.0),    // Left strafe only
        std::make_tuple(0.0, 0.0, 0.5),    // Rotation only
        std::make_tuple(0.3, 0.3, 0.0),    // Diagonal
        std::make_tuple(0.5, 0.0, 0.3),    // Arc turn
        std::make_tuple(0.3, 0.3, 0.3),    // All three
        std::make_tuple(-0.5, 0.0, 0.0),   // Backward
        std::make_tuple(0.0, -0.5, 0.0),   // Right strafe
        std::make_tuple(0.0, 0.0, -0.5)    // CW rotation
    )
);

// /**
//  * @test Main function for running all tests
//  */
// int main(int argc, char **argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
