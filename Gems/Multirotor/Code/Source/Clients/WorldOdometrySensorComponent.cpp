#include "WorldOdometrySensorComponent.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace Multirotor
{
    namespace
    {
        constexpr const char* OdometryMessageType = "nav_msgs::msg::Odometry";
        constexpr const char* WorldFrame = "world";
    } // namespace

    void WorldOdometrySensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WorldOdometrySensorComponent, SensorBaseType>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<WorldOdometrySensorComponent>("World Odometry Sensor", "Ground truth odometry in the world frame")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
            }
        }
    }

    void WorldOdometrySensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("PhysicsDynamicRigidBodyService"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    WorldOdometrySensorComponent::WorldOdometrySensorComponent()
    {
        ROS2::TopicConfiguration topic;
        topic.m_type = OdometryMessageType;
        topic.m_topic = "odom";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(AZStd::string(OdometryMessageType), topic));
    }

    void WorldOdometrySensorComponent::Activate()
    {
        SensorBaseType::Activate();

        m_message.header.frame_id = WorldFrame;
        m_message.child_frame_id = GetNamespacedFrameID().c_str();

        const auto& publisherConfiguration = m_sensorConfiguration.m_publishersConfigurations[OdometryMessageType];
        AZStd::string topic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            topic, &ROS2::ROS2NamesRequests::GetNamespacedName, GetNamespace(), publisherConfiguration.m_topic);
        m_publisher = ROS2::ROS2Interface::Get()->GetNode()->create_publisher<nav_msgs::msg::Odometry>(
            topic.c_str(), publisherConfiguration.GetQoS());

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] float sensorDeltaTime, [[maybe_unused]] AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float physicsDeltaTime)
            {
                if (m_sensorConfiguration.m_publishingEnabled)
                {
                    Publish();
                }
            });
    }

    void WorldOdometrySensorComponent::Deactivate()
    {
        StopSensor();
        m_publisher.reset();
        SensorBaseType::Deactivate();
    }

    void WorldOdometrySensorComponent::Publish()
    {
        AzPhysics::RigidBody* body = nullptr;
        Physics::RigidBodyRequestBus::EventResult(body, GetEntityId(), &Physics::RigidBodyRequests::GetRigidBody);
        if (!body)
        {
            return;
        }

        const AZ::Transform pose = body->GetTransform();
        const AZ::Transform worldToBody = pose.GetInverse();

        m_message.header.stamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
        m_message.pose.pose = ROS2::ROS2Conversions::ToROS2Pose(pose);
        m_message.twist.twist.linear = ROS2::ROS2Conversions::ToROS2Vector3(worldToBody.TransformVector(body->GetLinearVelocity()));
        m_message.twist.twist.angular = ROS2::ROS2Conversions::ToROS2Vector3(worldToBody.TransformVector(body->GetAngularVelocity()));
        m_publisher->publish(m_message);
    }
} // namespace Multirotor
