#include "MultirotorConfiguration.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace Multirotor
{
    void RotorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RotorConfiguration>()
                ->Version(1)
                ->Field("Position", &RotorConfiguration::m_position)
                ->Field("Clockwise", &RotorConfiguration::m_clockwise)
                ->Field("Visual", &RotorConfiguration::m_visual);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RotorConfiguration>("Rotor", "A single rotor")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RotorConfiguration::m_position, "Position", "Hub position in the body frame [m]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RotorConfiguration::m_clockwise, "Clockwise", "Spin direction viewed from above")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RotorConfiguration::m_visual, "Visual", "Child entity to spin, with its origin at the hub");
            }
        }
    }

    void MotorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MotorConfiguration>()
                ->Version(1)
                ->Field("ThrustCoefficient", &MotorConfiguration::m_thrustCoefficient)
                ->Field("TorqueCoefficient", &MotorConfiguration::m_torqueCoefficient)
                ->Field("MaxRotorSpeed", &MotorConfiguration::m_maxRotorSpeed)
                ->Field("TimeConstant", &MotorConfiguration::m_timeConstant)
                ->Field("VisualSlowdown", &MotorConfiguration::m_visualSlowdown);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<MotorConfiguration>("Motor", "Motor and propeller model")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfiguration::m_thrustCoefficient, "Thrust coefficient", "k_f [N/(rad/s)^2]")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfiguration::m_torqueCoefficient, "Torque coefficient", "k_m [Nm/(rad/s)^2]")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfiguration::m_maxRotorSpeed, "Max rotor speed", "Rotor speed at full throttle [rad/s]")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfiguration::m_timeConstant, "Time constant", "First-order motor lag [s]")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorConfiguration::m_visualSlowdown, "Visual slowdown", "Rotor visuals spin this many times slower")
                    ->Attribute(AZ::Edit::Attributes::Min, 1.0f);
            }
        }
    }

    void ControllerConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ControllerConfiguration>()
                ->Version(1)
                ->Field("AttitudeP", &ControllerConfiguration::m_attitudeP)
                ->Field("MaxBodyRate", &ControllerConfiguration::m_maxBodyRate)
                ->Field("RateP", &ControllerConfiguration::m_rateP)
                ->Field("RateI", &ControllerConfiguration::m_rateI)
                ->Field("RateD", &ControllerConfiguration::m_rateD)
                ->Field("RateIntegralLimit", &ControllerConfiguration::m_rateIntegralLimit)
                ->Field("IntegratorThrottleThreshold", &ControllerConfiguration::m_integratorThrottleThreshold);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ControllerConfiguration>("Controller", "Onboard attitude and rate controller")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ControllerConfiguration::m_attitudeP, "Attitude P", "[1/s]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ControllerConfiguration::m_maxBodyRate, "Max body rate", "[rad/s]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ControllerConfiguration::m_rateP, "Rate P", "Inertia-normalized [1/s]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ControllerConfiguration::m_rateI, "Rate I", "Inertia-normalized [1/s^2]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ControllerConfiguration::m_rateD, "Rate D", "Inertia-normalized [-]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ControllerConfiguration::m_rateIntegralLimit, "Rate integral limit", "[rad]")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ControllerConfiguration::m_integratorThrottleThreshold,
                        "Integrator throttle threshold",
                        "Integrator is held at zero below this throttle");
            }
        }
    }

    AZStd::vector<RotorConfiguration> MultirotorConfiguration::DefaultRotors()
    {
        constexpr float d = 0.1414f;
        return {
            { AZ::Vector3(d, -d, 0.0f), false },
            { AZ::Vector3(-d, d, 0.0f), false },
            { AZ::Vector3(d, d, 0.0f), true },
            { AZ::Vector3(-d, -d, 0.0f), true },
        };
    }

    MultirotorConfiguration::MultirotorConfiguration()
    {
        m_attitudeCmdTopic.m_type = "geometry_msgs::msg::PoseStamped";
        m_attitudeCmdTopic.m_topic = "cmd/attitude";
        m_attitudeRateCmdTopic.m_type = "geometry_msgs::msg::TwistStamped";
        m_attitudeRateCmdTopic.m_topic = "cmd/attitude_rate";
    }

    void MultirotorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        RotorConfiguration::Reflect(context);
        MotorConfiguration::Reflect(context);
        ControllerConfiguration::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MultirotorConfiguration>()
                ->Version(1)
                ->Field("Rotors", &MultirotorConfiguration::m_rotors)
                ->Field("Motor", &MultirotorConfiguration::m_motor)
                ->Field("Controller", &MultirotorConfiguration::m_controller)
                ->Field("AttitudeCmdTopic", &MultirotorConfiguration::m_attitudeCmdTopic)
                ->Field("AttitudeRateCmdTopic", &MultirotorConfiguration::m_attitudeRateCmdTopic)
                ->Field("CommandTimeout", &MultirotorConfiguration::m_commandTimeout);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<MultirotorConfiguration>("Multirotor configuration", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultirotorConfiguration::m_rotors, "Rotors", "Rotor layout. Empty selects a 0.4 m quad X.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultirotorConfiguration::m_motor, "Motor", "Motor model")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultirotorConfiguration::m_controller, "Controller", "Onboard controller")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultirotorConfiguration::m_attitudeCmdTopic, "Attitude command", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultirotorConfiguration::m_attitudeRateCmdTopic, "Attitude rate command", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultirotorConfiguration::m_commandTimeout, "Command timeout", "[s], 0 disables")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f);
            }
        }
    }
} // namespace Multirotor
