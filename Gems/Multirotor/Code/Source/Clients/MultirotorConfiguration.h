#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/containers/vector.h>
#include <ROS2/Communication/TopicConfiguration.h>

namespace AZ
{
    class ReflectContext;
}

namespace Multirotor
{
    //! A single rotor, expressed in the body frame (FLU, Z up) relative to the entity origin.
    struct RotorConfiguration
    {
        AZ_TYPE_INFO(RotorConfiguration, "{6B0E3B53-4D7A-4C6E-9E39-1B0C6A9F2E11}");
        static void Reflect(AZ::ReflectContext* context);

        RotorConfiguration() = default;
        RotorConfiguration(const AZ::Vector3& position, bool clockwise)
            : m_position(position)
            , m_clockwise(clockwise)
        {
        }

        AZ::Vector3 m_position = AZ::Vector3::CreateZero(); //!< Rotor hub position [m].
        bool m_clockwise = false; //!< Spin direction viewed from above. Determines the sign of the yaw reaction torque.
    };

    //! Motor/propeller model shared by all rotors.
    struct MotorConfiguration
    {
        AZ_TYPE_INFO(MotorConfiguration, "{0C5F7C1E-2F7B-4B0D-8B7B-6A3B2C1D9E21}");
        static void Reflect(AZ::ReflectContext* context);

        float m_thrustCoefficient = 1.96e-5f; //!< k_f: thrust = k_f * omega^2 [N/(rad/s)^2].
        float m_torqueCoefficient = 3.1e-7f; //!< k_m: reaction torque = k_m * omega^2 [Nm/(rad/s)^2].
        float m_maxRotorSpeed = 1000.0f; //!< omega at throttle 1.0 [rad/s].
        float m_timeConstant = 0.03f; //!< First-order motor lag [s].
    };

    //! Onboard attitude and attitude-rate controller gains.
    //! Rate gains are normalized by inertia, so they are in units of angular acceleration per error.
    struct ControllerConfiguration
    {
        AZ_TYPE_INFO(ControllerConfiguration, "{9D7E2A64-1C3B-4F5E-A8D9-3E2F1B0C7A31}");
        static void Reflect(AZ::ReflectContext* context);

        AZ::Vector3 m_attitudeP = AZ::Vector3(8.0f, 8.0f, 4.0f); //!< Attitude error to desired body rate [1/s].
        AZ::Vector3 m_maxBodyRate = AZ::Vector3(6.0f, 6.0f, 3.0f); //!< Limit of the attitude loop output [rad/s].
        AZ::Vector3 m_rateP = AZ::Vector3(20.0f, 20.0f, 8.0f); //!< [1/s].
        AZ::Vector3 m_rateI = AZ::Vector3(5.0f, 5.0f, 2.0f); //!< [1/s^2].
        AZ::Vector3 m_rateD = AZ::Vector3(0.2f, 0.2f, 0.0f); //!< [-].
        AZ::Vector3 m_rateIntegralLimit = AZ::Vector3(1.0f, 1.0f, 1.0f); //!< Anti-windup limit on the integrated rate error [rad].
        float m_integratorThrottleThreshold = 0.05f; //!< The integrator is reset while throttle is below this (e.g. landed).
    };

    struct MultirotorConfiguration
    {
        AZ_TYPE_INFO(MultirotorConfiguration, "{3A8C1F02-7E4D-4B6A-9C5E-2D1F0A9B8C41}");
        static void Reflect(AZ::ReflectContext* context);

        MultirotorConfiguration();

        AZStd::vector<RotorConfiguration> m_rotors; //!< Defaults to a 0.4 m quad in X configuration (PX4 motor order).
        MotorConfiguration m_motor;
        ControllerConfiguration m_controller;

        ROS2::TopicConfiguration m_attitudeCmdTopic; //!< mrs_msgs/HwApiAttitudeCmd
        ROS2::TopicConfiguration m_attitudeRateCmdTopic; //!< mrs_msgs/HwApiAttitudeRateCmd
        float m_commandTimeout = 0.5f; //!< Throttle is cut when no command arrives for this long [s, sim time]. 0 disables.
    };
} // namespace Multirotor
