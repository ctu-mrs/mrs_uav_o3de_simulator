#pragma once

#include "MultirotorConfiguration.h"
#include "MultirotorController.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <Multirotor/MultirotorTypeIds.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace Multirotor
{
    //! Simulates a multirotor on the entity's dynamic rigid body.
    //! Receives attitude or attitude-rate + collective throttle commands over ROS 2 (see MultirotorConfiguration)
    //! and runs the onboard controller and motor model on every physics sub-step, applying rotor forces to the body.
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

        MultirotorConfiguration m_config;
        MultirotorController m_controller;
        AZStd::vector<float> m_rotorAngles; //!< Of the rotor visuals [rad].

        AzPhysics::SceneHandle m_sceneHandle = AzPhysics::InvalidSceneHandle;
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        bool m_waitingForGround = false;
        float m_groundWaitTime = 0.0f; //!< [s, real time]
        AzPhysics::SceneEvents::OnSceneSimulationStartHandler m_sceneSimulationStartHandler;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_attitudeSubscription;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_attitudeRateSubscription;

        AZStd::mutex m_commandMutex;
        MultirotorController::Command m_command; //!< Guarded by m_commandMutex.
        double m_commandTime = 0.0; //!< Sim time the command arrived. Guarded by m_commandMutex.
        double m_simTime = 0.0; //!< Accumulated physics time since activation [s].
    };
} // namespace Multirotor
