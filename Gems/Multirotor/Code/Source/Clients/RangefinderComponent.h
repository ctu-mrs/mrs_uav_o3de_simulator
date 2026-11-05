#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <Multirotor/MultirotorTypeIds.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace Multirotor
{
    //! A single-beam rangefinder, publishing sensor_msgs/Range. The beam is the entity's X axis, cast on the physics
    //! step against colliders. A range above max_range means no return.
    class RangefinderComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(RangefinderComponent, RangefinderComponentTypeId, AZ::Component);

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void Activate() override;
        void Deactivate() override;

    private:
        void OnSceneSimulationFinish(AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime);

        AZStd::string m_topic = "rangefinder";
        AZ::EntityId m_ignoredEntity; //!< Colliders of this entity are not hit, usually the UAV's own.
        float m_rate = 100.0f; //!< [Hz, sim time]
        float m_maxRange = 40.0f; //!< [m]
        float m_fieldOfView = 0.01f; //!< Only reported, the beam is a single ray [rad].

        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneSimulationFinishHandler;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr m_publisher;
        AZStd::string m_frameId;
        float m_timeSincePublish = 0.0f; //!< [s, sim time]
    };
} // namespace Multirotor
