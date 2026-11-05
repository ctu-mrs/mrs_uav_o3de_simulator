#include <AzTest/AzTest.h>

#include <Clients/FreeCameraComponent.h>
#include <Clients/MultirotorController.h>

namespace UnitTest
{
    using Multirotor::MultirotorConfiguration;
    using Multirotor::MultirotorController;

    class MultirotorControllerTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            m_config.m_rotors = MultirotorConfiguration::DefaultRotors();
            ASSERT_TRUE(m_controller.Configure(m_config, AZ::Vector3(0.02f, 0.02f, 0.04f)));
        }

        float Sum(const AZStd::vector<float>& values)
        {
            float sum = 0.0f;
            for (float v : values)
            {
                sum += v;
            }
            return sum;
        }

        MultirotorConfiguration m_config;
        MultirotorController m_controller;
    };

    TEST_F(MultirotorControllerTest, MixPureThrust_SplitsEvenly)
    {
        const auto thrusts = m_controller.Mix(20.0f, AZ::Vector3::CreateZero());
        ASSERT_EQ(thrusts.size(), 4);
        for (float f : thrusts)
        {
            EXPECT_NEAR(f, 5.0f, 1e-4f);
        }
    }

    TEST_F(MultirotorControllerTest, MixRollTorque_LoadsLeftRotors)
    {
        // Positive roll torque (about +X, FLU) lifts the left side (+Y).
        const auto thrusts = m_controller.Mix(20.0f, AZ::Vector3(0.5f, 0.0f, 0.0f));
        EXPECT_NEAR(Sum(thrusts), 20.0f, 1e-3f);
        for (size_t i = 0; i < thrusts.size(); ++i)
        {
            const bool left = m_config.m_rotors[i].m_position.GetY() > 0.0f;
            EXPECT_EQ(thrusts[i] > 5.0f, left);
        }
    }

    TEST_F(MultirotorControllerTest, MixYawTorque_LoadsClockwiseRotors)
    {
        const auto thrusts = m_controller.Mix(20.0f, AZ::Vector3(0.0f, 0.0f, 0.1f));
        EXPECT_NEAR(Sum(thrusts), 20.0f, 1e-3f);
        for (size_t i = 0; i < thrusts.size(); ++i)
        {
            EXPECT_EQ(thrusts[i] > 5.0f, m_config.m_rotors[i].m_clockwise);
        }
    }

    TEST_F(MultirotorControllerTest, ThrottleToThrust_QuadraticInThrottle)
    {
        const float full = m_controller.ThrottleToThrust(1.0f);
        const auto& motor = m_config.m_motor;
        EXPECT_NEAR(full, 4.0f * motor.m_thrustCoefficient * motor.m_maxRotorSpeed * motor.m_maxRotorSpeed, 1e-3f);
        EXPECT_NEAR(m_controller.ThrottleToThrust(0.5f), 0.25f * full, 1e-3f);
        EXPECT_NEAR(m_controller.ThrottleToThrust(2.0f), full, 1e-3f);
    }

    TEST_F(MultirotorControllerTest, Step_ConvergesToCommandedThrustWithZeroYawTorque)
    {
        MultirotorController::Command command;
        command.m_mode = MultirotorController::Mode::AttitudeRate;
        command.m_throttle = 0.5f;
        for (int i = 0; i < 1000; ++i)
        {
            m_controller.Step(command, MultirotorController::State{}, 0.002f);
        }
        EXPECT_NEAR(Sum(m_controller.GetRotorThrusts()), m_controller.ThrottleToThrust(0.5f), 1e-2f);
        EXPECT_NEAR(m_controller.GetYawReactionTorque(), 0.0f, 1e-5f);
    }

    TEST_F(MultirotorControllerTest, Step_NoCommandStopsMotors)
    {
        m_controller.Step(MultirotorController::Command{}, MultirotorController::State{}, 0.002f);
        EXPECT_EQ(Sum(m_controller.GetRotorThrusts()), 0.0f);
    }

    TEST_F(MultirotorControllerTest, AttitudeControl_RollErrorGivesPositiveRollRate)
    {
        const AZ::Quaternion desired = AZ::Quaternion::CreateRotationX(0.1f);
        const AZ::Vector3 rate = m_controller.AttitudeControl(desired, AZ::Quaternion::CreateIdentity());
        EXPECT_GT(rate.GetX(), 0.0f);
        EXPECT_NEAR(rate.GetY(), 0.0f, 1e-6f);
        EXPECT_NEAR(rate.GetZ(), 0.0f, 1e-6f);
    }

    TEST_F(MultirotorControllerTest, AttitudeControl_TakesShortestPath)
    {
        // q and -q are the same rotation; the result must not depend on the sign.
        const AZ::Quaternion desired = AZ::Quaternion::CreateRotationZ(0.2f);
        const AZ::Vector3 a = m_controller.AttitudeControl(desired, AZ::Quaternion::CreateIdentity());
        const AZ::Vector3 b = m_controller.AttitudeControl(-desired, AZ::Quaternion::CreateIdentity());
        EXPECT_TRUE(a.IsClose(b));
        EXPECT_GT(a.GetZ(), 0.0f);
    }

    TEST(MultirotorConfiguration, ConstructorLeavesRotorsEmpty)
    {
        // Deserialization appends to the vector, so presetting it here multiplies rotors and thrust.
        EXPECT_TRUE(MultirotorConfiguration().m_rotors.empty());
        EXPECT_EQ(MultirotorConfiguration::DefaultRotors().size(), 4);
    }

    TEST(FreeCamera, LookAtFacesTheTarget)
    {
        using Multirotor::FreeCameraComponent;
        float yaw = 0.0f;
        float pitch = 0.0f;

        // Y ahead needs no rotation.
        FreeCameraComponent::LookAt(AZ::Vector3(1.0f, 1.0f, 1.0f), AZ::Vector3(1.0f, 5.0f, 1.0f), yaw, pitch);
        EXPECT_NEAR(yaw, 0.0f, 1e-5f);
        EXPECT_NEAR(pitch, 0.0f, 1e-5f);

        // +X is to the right, a negative rotation about Z.
        FreeCameraComponent::LookAt(AZ::Vector3::CreateZero(), AZ::Vector3(3.0f, 0.0f, 0.0f), yaw, pitch);
        EXPECT_NEAR(yaw, -AZ::Constants::HalfPi, 1e-5f);

        // Aiming along the result reaches the target, also above it.
        const AZ::Vector3 from(1.0f, -2.0f, 0.5f);
        const AZ::Vector3 to(4.0f, 2.0f, 3.0f);
        FreeCameraComponent::LookAt(from, to, yaw, pitch);
        const AZ::Quaternion rotation = AZ::Quaternion::CreateRotationZ(yaw) * AZ::Quaternion::CreateRotationX(pitch);
        const AZ::Vector3 forward = rotation.TransformVector(AZ::Vector3::CreateAxisY());
        EXPECT_TRUE(forward.IsClose((to - from).GetNormalized(), 1e-5f));

        // No direction keeps the current aim.
        yaw = 0.3f;
        FreeCameraComponent::LookAt(from, from, yaw, pitch);
        EXPECT_FLOAT_EQ(yaw, 0.3f);
    }

    TEST(MultirotorControllerConfigure, RejectsTooFewRotors)
    {
        MultirotorConfiguration config;
        config.m_rotors.resize(3);
        MultirotorController controller;
        EXPECT_FALSE(controller.Configure(config, AZ::Vector3::CreateOne()));
    }
} // namespace UnitTest

AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
