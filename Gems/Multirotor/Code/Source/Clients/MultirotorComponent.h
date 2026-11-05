#pragma once

#include "MultirotorConfiguration.h"
#include "MultirotorController.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <Multirotor/MultirotorTypeIds.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace Physics
{
    class Shape;
}

namespace Multirotor
{
    //! Simulates a multirotor on the entity's dynamic rigid body.
    //! Receives attitude or attitude-rate + collective throttle commands over ROS 2 (see MultirotorConfiguration)
    //! and runs the onboard controller and motor model on every physics sub-step, applying rotor forces to the body.
    //! Publishes whether the UAV stands on something, from the contacts of its rigid body.
    //! Sensors (IMU, odometry, cameras, ...) are separate ROS 2 sensor components on the same entity or its children.
    class MultirotorComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(MultirotorComponent, MultirotorComponentTypeId, AZ::Component);

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TickBus::Handler. Finds the rigid body after physics has created it, then spins the rotor visuals.
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void OnSceneSimulationStart(AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime);
        void SetCommand(const MultirotorController::Command& command);
        void SpinRotorVisuals(float deltaTime);
        void WaitForGround();
        void Release();
        void OnCollision(const AzPhysics::CollisionEvent& event);
        bool IsGroundContact(const AzPhysics::CollisionEvent& event) const;
        void PublishOnGround(float fixedDeltaTime);

        MultirotorConfiguration m_config;
        MultirotorController m_controller;
        AZStd::vector<float> m_rotorAngles; //!< Of the rotor visuals [rad].

        AzPhysics::SceneHandle m_sceneHandle = AzPhysics::InvalidSceneHandle;
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        bool m_waitingForGround = false;
        float m_groundWaitTime = 0.0f; //!< [s, real time]
        AzPhysics::SceneEvents::OnSceneSimulationStartHandler m_sceneSimulationStartHandler;
        AzPhysics::SimulatedBodyEvents::OnCollisionBegin::Handler m_collisionBeginHandler;
        AzPhysics::SimulatedBodyEvents::OnCollisionPersist::Handler m_collisionPersistHandler;
        AzPhysics::SimulatedBodyEvents::OnCollisionEnd::Handler m_collisionEndHandler;

        //! Pairs of shapes touching in a ground contact. Kept between events because they stop while the body sleeps.
        //! Ordered by address, because the pointers are only compared.
        AZStd::set<AZStd::pair<const Physics::Shape*, const Physics::Shape*>> m_groundContacts;
        float m_timeSinceOnGroundPublish = 0.0f; //!< [s, sim time]

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_attitudeSubscription;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_attitudeRateSubscription;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_onGroundPublisher;

        AZStd::mutex m_commandMutex;
        MultirotorController::Command m_command; //!< Guarded by m_commandMutex.
        double m_commandTime = 0.0; //!< Sim time the command arrived. Guarded by m_commandMutex.
        double m_simTime = 0.0; //!< Accumulated physics time since activation [s].
    };
} // namespace Multirotor
