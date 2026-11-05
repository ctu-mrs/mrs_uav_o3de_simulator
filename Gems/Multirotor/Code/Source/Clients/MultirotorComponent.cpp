#include "MultirotorComponent.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Time/ITime.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace Multirotor
{
    namespace
    {
        //! A contact is on the ground if its normal is within 45 deg of vertical. The sign of the normal is not used.
        constexpr float MinGroundContactNormalZ = 0.7071f;

        auto ShapePair(const AzPhysics::CollisionEvent& event)
        {
            const Physics::Shape* first = event.m_shape1;
            const Physics::Shape* second = event.m_shape2;
            return AZStd::make_pair(AZStd::min(first, second), AZStd::max(first, second));
        }
    } // namespace

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
        m_attitudeSubscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            namespaced(m_config.m_attitudeCmdTopic.m_topic).c_str(),
            m_config.m_attitudeCmdTopic.GetQoS(),
            [this](const geometry_msgs::msg::PoseStamped& msg)
            {
                MultirotorController::Command command;
                command.m_mode = MultirotorController::Mode::Attitude;
                command.m_attitude = ROS2::ROS2Conversions::FromROS2Quaternion(msg.pose.orientation).GetNormalized();
                command.m_throttle = aznumeric_cast<float>(msg.pose.position.z);
                SetCommand(command);
            });
        m_attitudeRateSubscription = node->create_subscription<geometry_msgs::msg::TwistStamped>(
            namespaced(m_config.m_attitudeRateCmdTopic.m_topic).c_str(),
            m_config.m_attitudeRateCmdTopic.GetQoS(),
            [this](const geometry_msgs::msg::TwistStamped& msg)
            {
                MultirotorController::Command command;
                command.m_mode = MultirotorController::Mode::AttitudeRate;
                command.m_bodyRate = ROS2::ROS2Conversions::FromROS2Vector3(msg.twist.angular);
                command.m_throttle = aznumeric_cast<float>(msg.twist.linear.z);
                SetCommand(command);
            });
        m_onGroundPublisher =
            node->create_publisher<std_msgs::msg::Bool>(namespaced(m_config.m_onGroundTopic.m_topic).c_str(), m_config.m_onGroundTopic.GetQoS());

        // The rigid body is created by the physics component's activation, which may come after ours.
        AZ::TickBus::Handler::BusConnect();
    }

    void MultirotorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_sceneSimulationStartHandler.Disconnect();
        m_collisionBeginHandler.Disconnect();
        m_collisionPersistHandler.Disconnect();
        m_collisionEndHandler.Disconnect();
        m_groundContacts.clear();
        m_attitudeSubscription.reset();
        m_attitudeRateSubscription.reset();
        m_onGroundPublisher.reset();
        m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        m_waitingForGround = false;
    }

    void MultirotorComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_waitingForGround)
        {
            WaitForGround();
            return;
        }

        if (m_bodyHandle != AzPhysics::InvalidSimulatedBodyHandle)
        {
            SpinRotorVisuals(deltaTime);
            return;
        }

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
            AZ::TickBus::Handler::BusDisconnect();
            return;
        }

        m_rotorAngles.assign(m_config.m_rotors.size(), 0.0f);
        m_bodyHandle = rigidBody->m_bodyHandle;
        m_sceneHandle = sceneHandle;

        if (m_config.m_groundWaitTimeout > 0.0f)
        {
            rigidBody->SetKinematic(true);
            m_groundWaitTime = 0.0f;
            m_waitingForGround = true;
            return;
        }

        Release();
    }

    void MultirotorComponent::WaitForGround()
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto* body = azdynamic_cast<AzPhysics::RigidBody*>(sceneInterface->GetSimulatedBodyFromHandle(m_sceneHandle, m_bodyHandle));
        if (!body)
        {
            return;
        }

        AzPhysics::RayCastRequest request;
        request.m_start = body->GetPosition();
        request.m_direction = -AZ::Vector3::CreateAxisZ();
        request.m_distance = 10000.0f;
        request.m_filterCallback = [handle = m_bodyHandle](const AzPhysics::SimulatedBody* hitBody, [[maybe_unused]] const Physics::Shape* shape)
        {
            return hitBody->m_bodyHandle == handle ? AzPhysics::SceneQuery::QueryHitType::None : AzPhysics::SceneQuery::QueryHitType::Block;
        };

        m_groundWaitTime += AZ::TimeUsToSeconds(AZ::GetRealTickDeltaTimeUs());
        const AzPhysics::SceneQueryHits hits = sceneInterface->QueryScene(m_sceneHandle, &request);
        if (hits)
        {
            AZ_Printf("Multirotor", "ground found %.2f m below after %.1f s\n", hits.m_hits[0].m_distance, m_groundWaitTime);
        }
        else if (m_groundWaitTime < m_config.m_groundWaitTimeout)
        {
            return;
        }
        else
        {
            AZ_Warning("Multirotor", false, "no ground found below the UAV after %.1f s, releasing it", m_groundWaitTime);
        }

        body->SetKinematic(false);
        m_waitingForGround = false;
        Release();
    }

    void MultirotorComponent::Release()
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        m_sceneSimulationStartHandler = AzPhysics::SceneEvents::OnSceneSimulationStartHandler(
            [this](AzPhysics::SceneHandle handle, float fixedDeltaTime)
            {
                OnSceneSimulationStart(handle, fixedDeltaTime);
            },
            aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
        sceneInterface->RegisterSceneSimulationStartHandler(m_sceneHandle, m_sceneSimulationStartHandler);

        auto onCollision = [this]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, const AzPhysics::CollisionEvent& event)
        {
            OnCollision(event);
        };
        m_collisionBeginHandler = AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler(onCollision);
        m_collisionPersistHandler = AzPhysics::SimulatedBodyEvents::OnCollisionPersist::Handler(onCollision);
        m_collisionEndHandler = AzPhysics::SimulatedBodyEvents::OnCollisionEnd::Handler(onCollision);
        AzPhysics::SimulatedBodyEvents::RegisterOnCollisionBeginHandler(m_sceneHandle, m_bodyHandle, m_collisionBeginHandler);
        AzPhysics::SimulatedBodyEvents::RegisterOnCollisionPersistHandler(m_sceneHandle, m_bodyHandle, m_collisionPersistHandler);
        AzPhysics::SimulatedBodyEvents::RegisterOnCollisionEndHandler(m_sceneHandle, m_bodyHandle, m_collisionEndHandler);
    }

    void MultirotorComponent::OnCollision(const AzPhysics::CollisionEvent& event)
    {
        if (event.m_type == AzPhysics::CollisionEvent::Type::End)
        {
            m_groundContacts.erase(ShapePair(event));
        }
        else if (IsGroundContact(event))
        {
            m_groundContacts.insert(ShapePair(event));
        }
        else if (!event.m_contacts.empty())
        {
            m_groundContacts.erase(ShapePair(event));
        }
    }

    bool MultirotorComponent::IsGroundContact(const AzPhysics::CollisionEvent& event) const
    {
        // The other end of a near-vertical contact below the UAV is the ground. One above it is a ceiling.
        const float bodyHeight = event.m_body1->GetPosition().GetZ();
        for (const AzPhysics::Contact& contact : event.m_contacts)
        {
            if (AZStd::abs(contact.m_normal.GetZ()) >= MinGroundContactNormalZ && contact.m_position.GetZ() < bodyHeight)
            {
                return true;
            }
        }
        return false;
    }

    void MultirotorComponent::PublishOnGround(float fixedDeltaTime)
    {
        m_timeSinceOnGroundPublish += fixedDeltaTime;
        if (m_config.m_onGroundRate <= 0.0f || m_timeSinceOnGroundPublish < 1.0f / m_config.m_onGroundRate)
        {
            return;
        }
        m_timeSinceOnGroundPublish -= 1.0f / m_config.m_onGroundRate;

        std_msgs::msg::Bool message;
        message.data = !m_groundContacts.empty();
        m_onGroundPublisher->publish(message);
    }

    void MultirotorComponent::SpinRotorVisuals(float deltaTime)
    {
        const auto& speeds = m_controller.GetRotorSpeeds();
        for (size_t i = 0; i < speeds.size(); ++i)
        {
            const RotorConfiguration& rotor = m_config.m_rotors[i];
            if (!rotor.m_visual.IsValid())
            {
                continue;
            }

            // A clockwise rotor, viewed from above, spins about -Z.
            const float direction = rotor.m_clockwise ? -1.0f : 1.0f;
            m_rotorAngles[i] = AZStd::fmod(m_rotorAngles[i] + direction * speeds[i] / m_config.m_motor.m_visualSlowdown * deltaTime, AZ::Constants::TwoPi);
            AZ::TransformBus::Event(
                rotor.m_visual, &AZ::TransformInterface::SetLocalRotationQuaternion, AZ::Quaternion::CreateRotationZ(m_rotorAngles[i]));
        }
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

        PublishOnGround(fixedDeltaTime);
    }
} // namespace Multirotor
