#pragma once

#include "MultirotorConfiguration.h"

#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Vector4.h>
#include <AzCore/std/containers/vector.h>

namespace Multirotor
{
    //! Onboard flight controller and motor model: attitude P loop -> rate PID -> mixer -> first-order motors.
    //! Runs at the physics rate. Free of physics and ROS dependencies so it can be unit tested.
    //! All vectors are in the body frame (FLU, Z up) unless stated otherwise.
    class MultirotorController
    {
    public:
        enum class Mode
        {
            None, //!< No command received yet: motors are stopped.
            Attitude,
            AttitudeRate,
        };

        struct Command
        {
            Mode m_mode = Mode::None;
            AZ::Quaternion m_attitude = AZ::Quaternion::CreateIdentity(); //!< Desired orientation, world <- body.
            AZ::Vector3 m_bodyRate = AZ::Vector3::CreateZero(); //!< Desired body rate [rad/s].
            float m_throttle = 0.0f; //!< Collective throttle in [0, 1], proportional to rotor speed.
        };

        struct State
        {
            AZ::Quaternion m_orientation = AZ::Quaternion::CreateIdentity(); //!< world <- body.
            AZ::Vector3 m_bodyRate = AZ::Vector3::CreateZero(); //!< [rad/s].
        };

        //! @param inertiaDiagonal principal moments of inertia of the body [kg m^2].
        //! @return false if the rotor layout cannot produce independent thrust, roll, pitch, and yaw.
        bool Configure(const MultirotorConfiguration& config, const AZ::Vector3& inertiaDiagonal);

        void Reset();

        //! Advance the controller and motors by dt. Results are available through the getters below.
        void Step(const Command& command, const State& state, float dt);

        const AZStd::vector<float>& GetRotorSpeeds() const { return m_rotorSpeeds; } //!< [rad/s]
        const AZStd::vector<float>& GetRotorThrusts() const { return m_rotorThrusts; } //!< Along body +Z [N].
        float GetYawReactionTorque() const { return m_yawReactionTorque; } //!< Total about body +Z [Nm].

        //! Total thrust the rotors produce in steady state at the given collective throttle [N].
        float ThrottleToThrust(float throttle) const;

        //! Per-rotor thrust that realizes the given total thrust and body torque, clamped to [0, max].
        AZStd::vector<float> Mix(float collectiveThrust, const AZ::Vector3& torque) const;

        //! Body torque requested by the rate loop. Updates the integrator state.
        AZ::Vector3 RateControl(const AZ::Vector3& desiredRate, const AZ::Vector3& bodyRate, bool integrate, float dt);

        //! Desired body rate from the attitude error.
        AZ::Vector3 AttitudeControl(const AZ::Quaternion& desired, const AZ::Quaternion& current) const;

    private:
        MotorConfiguration m_motor;
        ControllerConfiguration m_gains;
        AZ::Vector3 m_inertia = AZ::Vector3::CreateOne();

        AZStd::vector<float> m_yawSign; //!< +1 for clockwise rotors, -1 for counter-clockwise.
        AZStd::vector<AZ::Vector4> m_allocation; //!< Row i maps (thrust, torque x, y, z) to rotor i thrust.

        AZ::Vector3 m_rateIntegral = AZ::Vector3::CreateZero();
        AZ::Vector3 m_previousRateError = AZ::Vector3::CreateZero();
        bool m_hasPreviousRateError = false;

        AZStd::vector<float> m_rotorSpeeds;
        AZStd::vector<float> m_rotorThrusts;
        float m_yawReactionTorque = 0.0f;
    };
} // namespace Multirotor
