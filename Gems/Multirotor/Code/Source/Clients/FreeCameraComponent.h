#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>
#include <Multirotor/MultirotorTypeIds.h>

namespace Multirotor
{
    //! Switches the game view between a follow camera and the camera on this entity.
    //! C toggles the view, which starts from the follow camera's pose. WASD moves, Q and E go down and up,
    //! the right mouse button looks around, and left shift moves faster.
    class FreeCameraComponent
        : public AZ::Component
        , private AZ::TickBus::Handler
        , private AzFramework::InputChannelEventListener
    {
    public:
        AZ_COMPONENT(FreeCameraComponent, FreeCameraComponentTypeId, AZ::Component);

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TickBus::Handler
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AzFramework::InputChannelEventListener
        bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;

        void ToggleView();

        AZ::EntityId m_followCamera;
        float m_speed = 3.0f; //!< [m/s]
        float m_fastMultiplier = 4.0f;
        float m_lookSensitivity = 0.003f; //!< [rad/pixel]

        bool m_active = false;
        bool m_looking = false;
        bool m_fast = false;
        AZ::Vector3 m_moveInput = AZ::Vector3::CreateZero(); //!< Right, forward, up, each in [-1, 1].
        bool m_keys[6] = {}; //!< Held movement keys, in the order of the key table in the source file.
        AZ::Vector3 m_position = AZ::Vector3::CreateZero(); //!< In the world [m].
        float m_yaw = 0.0f; //!< [rad]
        float m_pitch = 0.0f; //!< [rad]
    };
} // namespace Multirotor
