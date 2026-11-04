#include <AzTest/AzTest.h>

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

    TEST(MultirotorControllerConfigure, RejectsTooFewRotors)
    {
        MultirotorConfiguration config;
        config.m_rotors.resize(3);
        MultirotorController controller;
        EXPECT_FALSE(controller.Configure(config, AZ::Vector3::CreateOne()));
    }
} // namespace UnitTest

AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
