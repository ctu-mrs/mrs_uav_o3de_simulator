
#include "DroneSimSampleComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <imgui/imgui.h>

#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace DroneSim
{

    DroneSimSampleComponent::DroneSimSampleComponent()
    {
        m_goalTopicConfiguration.m_topic = "/goal_pose";
        m_goalTopicConfiguration.m_type = "geometry_msgs::msg::PoseStamped";
    }

    void DroneSimSampleComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_goalPublisher = ros2Node->create_publisher<geometry_msgs::msg::PoseStamped>(m_goalTopicConfiguration.m_topic.data(), m_goalTopicConfiguration.GetQoS());
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();

    }

    void DroneSimSampleComponent::Deactivate()
    {
        m_goalPublisher.reset();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void DroneSimSampleComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<DroneSimSampleComponent, AZ::Component>()
                ->Version(1)
                ->Field("goals", &DroneSimSampleComponent::m_goalEntities)
                ->Field("goalMessageTopic", &DroneSimSampleComponent::m_goalTopicConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<DroneSimSampleComponent>("DroneSimSampleComponent", "A sample component that sends goal")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "DroneSimSampleComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Sample ROS2 Project")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &DroneSimSampleComponent::m_goalEntities, "Goals", "Entities goals")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &DroneSimSampleComponent::m_goalTopicConfiguration,
                        "Topic for goal message",
                        "Configuration for ROS2 topic to send goal message to");
            }
        }
    }

    void DroneSimSampleComponent::OnImGuiUpdate()
    {
        ImGui::Begin("DroneSimSampleComponent");
        for (const auto& entityId : m_goalEntities)
        {
            AZStd::string entityName;
            AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, entityId);
            const AZStd::string buttonName = AZStd::string::format("Send goal %s", entityName.c_str());

            if (ImGui::Button(buttonName.c_str()))
            {
                AZ::Transform transform;
                AZ::TransformBus::EventResult(transform, entityId, &AZ::TransformBus::Events::GetWorldTM);
                geometry_msgs::msg::PoseStamped poseStamped;
                poseStamped.header.frame_id = "odom";
                poseStamped.pose = ROS2::ROS2Conversions::ToROS2Pose(transform);
                m_goalPublisher->publish(poseStamped);
            }
        }
        ImGui::End();
    }
} // namespace DroneSim
