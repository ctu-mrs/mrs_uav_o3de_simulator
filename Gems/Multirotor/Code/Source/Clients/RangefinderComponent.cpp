#include "RangefinderComponent.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>

namespace Multirotor
{
    void RangefinderComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RangefinderComponent, AZ::Component>()
                ->Version(1)
                ->Field("Topic", &RangefinderComponent::m_topic)
                ->Field("IgnoredEntity", &RangefinderComponent::m_ignoredEntity)
                ->Field("Rate", &RangefinderComponent::m_rate)
                ->Field("MaxRange", &RangefinderComponent::m_maxRange)
                ->Field("FieldOfView", &RangefinderComponent::m_fieldOfView);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RangefinderComponent>("Rangefinder", "Single-beam rangefinder along the entity's X axis")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RangefinderComponent::m_topic, "Topic", "sensor_msgs/Range")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RangefinderComponent::m_ignoredEntity, "Ignored entity", "Its colliders are not hit")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RangefinderComponent::m_rate, "Rate", "[Hz, sim time]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RangefinderComponent::m_maxRange, "Max range", "[m]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RangefinderComponent::m_fieldOfView, "Field of view", "Only reported [rad]");
            }
        }
    }

    void RangefinderComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void RangefinderComponent::Activate()
    {
        AZStd::string ns;
        ROS2::ROS2FrameComponentBus::EventResult(ns, GetEntityId(), &ROS2::ROS2FrameComponentRequests::GetNamespace);
        ROS2::ROS2FrameComponentBus::EventResult(m_frameId, GetEntityId(), &ROS2::ROS2FrameComponentRequests::GetNamespacedFrameID);

        AZStd::string topic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(topic, &ROS2::ROS2NamesRequests::GetNamespacedName, ns, m_topic);
        m_publisher = ROS2::ROS2Interface::Get()->GetNode()->create_publisher<sensor_msgs::msg::Range>(topic.c_str(), rclcpp::QoS(10));

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (!sceneInterface)
        {
            return;
        }

        m_timeSincePublish = 0.0f;
        m_sceneSimulationFinishHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle handle, float fixedDeltaTime)
            {
                OnSceneSimulationFinish(handle, fixedDeltaTime);
            });
        sceneInterface->RegisterSceneSimulationFinishHandler(
            sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName), m_sceneSimulationFinishHandler);
    }

    void RangefinderComponent::Deactivate()
    {
        m_sceneSimulationFinishHandler.Disconnect();
        m_publisher.reset();
    }

    void RangefinderComponent::OnSceneSimulationFinish(AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
    {
        m_timeSincePublish += fixedDeltaTime;
        if (m_rate <= 0.0f || m_timeSincePublish < 1.0f / m_rate)
        {
            return;
        }
        m_timeSincePublish -= 1.0f / m_rate;

        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, GetEntityId(), &AZ::TransformInterface::GetWorldTM);

        AzPhysics::RayCastRequest request;
        request.m_start = transform.GetTranslation();
        request.m_direction = transform.GetBasisX().GetNormalized();
        request.m_distance = m_maxRange;
        request.m_filterCallback = [ignored = m_ignoredEntity](const AzPhysics::SimulatedBody* body, [[maybe_unused]] const Physics::Shape* shape)
        {
            return body->GetEntityId() == ignored ? AzPhysics::SceneQuery::QueryHitType::None : AzPhysics::SceneQuery::QueryHitType::Block;
        };

        const AzPhysics::SceneQueryHits hits = AZ::Interface<AzPhysics::SceneInterface>::Get()->QueryScene(sceneHandle, &request);

        sensor_msgs::msg::Range message;
        message.header.stamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
        message.header.frame_id = m_frameId.c_str();
        message.radiation_type = sensor_msgs::msg::Range::INFRARED;
        message.field_of_view = m_fieldOfView;
        message.min_range = 0.0f;
        message.max_range = m_maxRange;
        message.range = hits ? hits.m_hits[0].m_distance : m_maxRange + 1.0f;
        m_publisher->publish(message);
    }
} // namespace Multirotor
