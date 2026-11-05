#include "MultirotorComponent.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace Multirotor
{
    void MultirotorComponent::Reflect(AZ::ReflectContext* context)
    {
        MultirotorConfiguration::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MultirotorComponent, AZ::Component>()->Version(1)->Field("Config", &MultirotorComponent::m_config);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<MultirotorComponent>("Multirotor", "Multirotor dynamics and onboard attitude controller")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultirotorComponent::m_config, "Configuration", "")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void MultirotorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("MultirotorService"));
    }

    void MultirotorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("PhysicsDynamicRigidBodyService"));
    }

    void MultirotorComponent::Activate()
    {
        m_simTime = 0.0;
        m_command = {};
        m_commandTime = 0.0;
        if (m_config.m_rotors.empty())
        {
            m_config.m_rotors = MultirotorConfiguration::DefaultRotors();
        }

        AZStd::string ns;
        ROS2::ROS2FrameComponentBus::EventResult(ns, GetEntityId(), &ROS2::ROS2FrameComponentRequests::GetNamespace);
        auto namespaced = [&ns](const AZStd::string& topic)
        {
            AZStd::string result;
            ROS2::ROS2NamesRequestBus::BroadcastResult(result, &ROS2::ROS2NamesRequests::GetNamespacedName, ns, topic);
            return result;
        };

        auto node = ROS2::ROS2Interface::Get()->GetNode();
        m_attitudeSubscription = node->create_subscription<mrs_msgs::msg::HwApiAttitudeCmd>(
            namespaced(m_config.m_attitudeCmdTopic.m_topic).c_str(),
            m_config.m_attitudeCmdTopic.GetQoS(),
            [this](const mrs_msgs::msg::HwApiAttitudeCmd& msg)
            {
                MultirotorController::Command command;
                command.m_mode = MultirotorController::Mode::Attitude;
                command.m_attitude = ROS2::ROS2Conversions::FromROS2Quaternion(msg.orientation).GetNormalized();
                command.m_throttle = aznumeric_cast<float>(msg.throttle);
                SetCommand(command);
            });
        m_attitudeRateSubscription = node->create_subscription<mrs_msgs::msg::HwApiAttitudeRateCmd>(
            namespaced(m_config.m_attitudeRateCmdTopic.m_topic).c_str(),
            m_config.m_attitudeRateCmdTopic.GetQoS(),
            [this](const mrs_msgs::msg::HwApiAttitudeRateCmd& msg)
            {
                MultirotorController::Command command;
                command.m_mode = MultirotorController::Mode::AttitudeRate;
                command.m_bodyRate = ROS2::ROS2Conversions::FromROS2Vector3(msg.body_rate);
                command.m_throttle = aznumeric_cast<float>(msg.throttle);
                SetCommand(command);
            });

        // The rigid body is created by the physics component's activation, which may come after ours.
        AZ::TickBus::Handler::BusConnect();
    }

    void MultirotorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_sceneSimulationStartHandler.Disconnect();
        m_attitudeSubscription.reset();
        m_attitudeRateSubscription.reset();
        m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
    }

    void MultirotorComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        if (!sceneInterface)
        {
            return;
        }
        const AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);

        AzPhysics::RigidBody* rigidBody = nullptr;
        Physics::RigidBodyRequestBus::EventResult(rigidBody, GetEntityId(), &Physics::RigidBodyRequests::GetRigidBody);
        if (sceneHandle == AzPhysics::InvalidSceneHandle || !rigidBody)
        {
            return;
        }
        AZ::TickBus::Handler::BusDisconnect();

        const AZ::Matrix3x3 inertia = rigidBody->GetInertiaLocal();
        const AZ::Vector3 inertiaDiagonal(inertia.GetElement(0, 0), inertia.GetElement(1, 1), inertia.GetElement(2, 2));
        if (!m_controller.Configure(m_config, inertiaDiagonal))
        {
            AZ_Error(
                "Multirotor",
                false,
                "Entity %s: the rotor layout cannot control thrust, roll, pitch, and yaw independently. "
                "At least 4 rotors with non-degenerate positions and a positive thrust coefficient are required.",
                GetEntity()->GetName().c_str());
            return;
        }

        m_bodyHandle = rigidBody->m_bodyHandle;
        m_sceneSimulationStartHandler = AzPhysics::SceneEvents::OnSceneSimulationStartHandler(
            [this](AzPhysics::SceneHandle handle, float fixedDeltaTime)
            {
                OnSceneSimulationStart(handle, fixedDeltaTime);
            },
            aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
        sceneInterface->RegisterSceneSimulationStartHandler(sceneHandle, m_sceneSimulationStartHandler);
    }

    void MultirotorComponent::SetCommand(const MultirotorController::Command& command)
    {
        AZStd::lock_guard<AZStd::mutex> lock(m_commandMutex);
        m_command = command;
        m_command.m_throttle = AZ::GetClamp(m_command.m_throttle, 0.0f, 1.0f);
        m_commandTime = m_simTime;
    }

    void MultirotorComponent::OnSceneSimulationStart(AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto* body = azdynamic_cast<AzPhysics::RigidBody*>(sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, m_bodyHandle));
        if (!body)
        {
            return;
        }

        MultirotorController::Command command;
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_commandMutex);
            m_simTime += fixedDeltaTime;
            const bool timedOut = m_config.m_commandTimeout > 0.0f && m_simTime - m_commandTime > m_config.m_commandTimeout;
            if (!timedOut)
            {
                command = m_command;
            }
        }

        const AZ::Transform transform = body->GetTransform();
        const AZ::Quaternion orientation = transform.GetRotation();

        MultirotorController::State state;
        state.m_orientation = orientation;
        state.m_bodyRate = orientation.GetConjugate().TransformVector(body->GetAngularVelocity());

        m_controller.Step(command, state, fixedDeltaTime);

        // Forces are constant over the sub-step, so the impulse is force * dt.
        const AZ::Vector3 bodyUp = orientation.TransformVector(AZ::Vector3::CreateAxisZ());
        const auto& thrusts = m_controller.GetRotorThrusts();
        for (size_t i = 0; i < thrusts.size(); ++i)
        {
            const AZ::Vector3 rotorWorld = transform.TransformPoint(m_config.m_rotors[i].m_position);
            body->ApplyLinearImpulseAtWorldPoint(bodyUp * (thrusts[i] * fixedDeltaTime), rotorWorld);
        }
        body->ApplyAngularImpulse(bodyUp * (m_controller.GetYawReactionTorque() * fixedDeltaTime));
    }
} // namespace Multirotor
