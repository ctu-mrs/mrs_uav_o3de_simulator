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

    //! Closed-loop step responses of the controller on the x500's rotational dynamics, stepped as PhysX steps the rigid
    //! body: the controller runs once per physics step, and the rotor thrusts act as impulses over that step. Compares
    //! the PhysX step the project uses (2 ms) with a coarser one (4 ms), which halves the physics cost per rendered frame.
    //! In this model the responses stay within the limits up to a 64 ms step (checked 2026-11-17), so it guards the
    //! response shape, not the margin: sensor delay, contacts, and the MRS outer loops are not modelled; fly to check those.
    class MultirotorClosedLoopTest : public ::testing::TestWithParam<float>
    {
    protected:
        struct Response
        {
            float m_overshoot = 0.0f; //!< Peak beyond the target, relative to the step.
            float m_settlingTime = 0.0f; //!< Last time the error exceeded 5% of the step [s].
            float m_finalError = 0.0f; //!< Relative to the step, at the end.
        };

        void SetUp() override
        {
            // x500 as in Assets/Drone/Multirotor.prefab and Tools/create_drone_prefab.py.
            constexpr float d = 0.1812f;
            constexpr float h = 0.057f;
            m_config.m_rotors = {
                { AZ::Vector3(d, -d, h), false },
                { AZ::Vector3(-d, d, h), false },
                { AZ::Vector3(d, d, h), true },
                { AZ::Vector3(-d, -d, h), true },
            };
            ASSERT_TRUE(m_controller.Configure(m_config, m_inertia));
        }

        //! Advances the body by one physics step of dt with the controller's current rotor thrusts.
        void StepBody(const MultirotorController& controller, MultirotorController::State& state, float dt) const
        {
            AZ::Vector3 torque(0.0f, 0.0f, controller.GetYawReactionTorque());
            const auto& thrusts = controller.GetRotorThrusts();
            for (size_t i = 0; i < thrusts.size(); ++i)
            {
                torque += m_config.m_rotors[i].m_position.Cross(AZ::Vector3(0.0f, 0.0f, thrusts[i]));
            }
            state.m_bodyRate += torque / m_inertia * dt;
            const AZ::Vector3 w = state.m_bodyRate;
            const AZ::Quaternion spin(w.GetX(), w.GetY(), w.GetZ(), 0.0f);
            state.m_orientation = (state.m_orientation + state.m_orientation * spin * (0.5f * dt)).GetNormalized();
        }

        //! value(state) is the controlled quantity; target its commanded value.
        template<typename Value>
        Response Simulate(const MultirotorController::Command& command, float target, Value value, float dt) const
        {
            MultirotorController controller = m_controller;
            MultirotorController::State state;
            Response response;
            float peak = 0.0f;
            const int steps = static_cast<int>(3.0f / dt);
            for (int i = 0; i < steps; ++i)
            {
                controller.Step(command, state, dt);
                StepBody(controller, state, dt);
                const float v = value(state);
                peak = AZ::GetMax(peak, v);
                if (AZ::GetAbs(v - target) > 0.05f * AZ::GetAbs(target))
                {
                    response.m_settlingTime = (i + 1) * dt;
                }
                response.m_finalError = AZ::GetAbs(v - target) / AZ::GetAbs(target);
            }
            response.m_overshoot = AZ::GetMax(0.0f, peak - target) / AZ::GetAbs(target);
            return response;
        }

        Response RollRateStep(float dt) const
        {
            MultirotorController::Command command;
            command.m_mode = MultirotorController::Mode::AttitudeRate;
            command.m_throttle = 0.5f;
            command.m_bodyRate = AZ::Vector3(1.0f, 0.0f, 0.0f);
            return Simulate(command, 1.0f, [](const MultirotorController::State& s) { return s.m_bodyRate.GetX(); }, dt);
        }

        Response RollAttitudeStep(float dt) const
        {
            MultirotorController::Command command;
            command.m_mode = MultirotorController::Mode::Attitude;
            command.m_throttle = 0.5f;
            command.m_attitude = AZ::Quaternion::CreateRotationX(0.2f);
            return Simulate(
                command, 0.2f,
                [](const MultirotorController::State& s)
                {
                    const AZ::Quaternion q = s.m_orientation;
                    return 2.0f * AZStd::atan2(q.GetX(), q.GetW()); // roll, as the step is about X only
                },
                dt);
        }

        MultirotorConfiguration m_config;
        MultirotorController m_controller;
        AZ::Vector3 m_inertia = AZ::Vector3(0.03167f, 0.03167f, 0.0625f);
    };

    TEST_P(MultirotorClosedLoopTest, RollRateAndAttitudeStepsMatchThe2msReference)
    {
        const float dt = GetParam();
        const Response referenceRate = RollRateStep(0.002f);
        const Response referenceAttitude = RollAttitudeStep(0.002f);
        const Response rate = RollRateStep(dt);
        const Response attitude = RollAttitudeStep(dt);
        printf("dt %.3f s: rate step overshoot %.3f settling %.3f s final %.4f (2 ms: %.3f, %.3f s); "
               "attitude step overshoot %.3f settling %.3f s final %.4f (2 ms: %.3f, %.3f s)\n",
               dt, rate.m_overshoot, rate.m_settlingTime, rate.m_finalError, referenceRate.m_overshoot,
               referenceRate.m_settlingTime, attitude.m_overshoot, attitude.m_settlingTime, attitude.m_finalError,
               referenceAttitude.m_overshoot, referenceAttitude.m_settlingTime);

        // Converges, and within 0.05 overshoot and 20% settling time of the 2 ms reference.
        EXPECT_LT(rate.m_finalError, 0.02f);
        EXPECT_LT(attitude.m_finalError, 0.02f);
        EXPECT_LT(rate.m_overshoot, referenceRate.m_overshoot + 0.05f);
        EXPECT_LT(attitude.m_overshoot, referenceAttitude.m_overshoot + 0.05f);
        EXPECT_LT(rate.m_settlingTime, 1.2f * referenceRate.m_settlingTime + 0.01f);
        EXPECT_LT(attitude.m_settlingTime, 1.2f * referenceAttitude.m_settlingTime + 0.01f);
    }

    INSTANTIATE_TEST_SUITE_P(PhysicsSteps, MultirotorClosedLoopTest, ::testing::Values(0.002f, 0.004f));
} // namespace UnitTest

AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
