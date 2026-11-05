#include "FreeCameraComponent.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Time/ITime.h>
#include <AzFramework/Components/CameraBus.h>
#include <AzFramework/Input/Channels/InputChannel.h>
#include <AzFramework/Input/Devices/Keyboard/InputDeviceKeyboard.h>
#include <AzFramework/Input/Devices/Mouse/InputDeviceMouse.h>

namespace Multirotor
{
    namespace
    {
        using Key = AzFramework::InputDeviceKeyboard::Key;
        using Mouse = AzFramework::InputDeviceMouse;

        constexpr float MaxPitch = 1.5f; //!< Keeps the view away from straight up and down [rad].

        //! A key held down moves along this direction: right, forward, up.
        struct MoveKey
        {
            const AzFramework::InputChannelId& m_channel;
            float m_right;
            float m_forward;
            float m_up;
        };

        const MoveKey MoveKeys[] = {
            { Key::AlphanumericD, 1.0f, 0.0f, 0.0f },  { Key::AlphanumericA, -1.0f, 0.0f, 0.0f },
            { Key::AlphanumericW, 0.0f, 1.0f, 0.0f },  { Key::AlphanumericS, 0.0f, -1.0f, 0.0f },
            { Key::AlphanumericE, 0.0f, 0.0f, 1.0f },  { Key::AlphanumericQ, 0.0f, 0.0f, -1.0f },
        };
    } // namespace

    void FreeCameraComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<FreeCameraComponent, AZ::Component>()
                ->Version(1)
                ->Field("FollowCamera", &FreeCameraComponent::m_followCamera)
                ->Field("Speed", &FreeCameraComponent::m_speed)
                ->Field("FastMultiplier", &FreeCameraComponent::m_fastMultiplier)
                ->Field("LookSensitivity", &FreeCameraComponent::m_lookSensitivity);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<FreeCameraComponent>("Free Camera", "Toggles the game view between a follow camera and this camera")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Camera")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FreeCameraComponent::m_followCamera, "Follow camera", "Camera to return to")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FreeCameraComponent::m_speed, "Speed", "[m/s]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FreeCameraComponent::m_fastMultiplier, "Fast multiplier", "With left shift held")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &FreeCameraComponent::m_lookSensitivity, "Look sensitivity", "[rad/pixel]");
            }
        }
    }

    void FreeCameraComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("CameraService"));
    }

    void FreeCameraComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        AzFramework::InputChannelEventListener::Connect();
    }

    void FreeCameraComponent::Deactivate()
    {
        AzFramework::InputChannelEventListener::Disconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void FreeCameraComponent::ToggleView()
    {
        if (m_active)
        {
            Camera::CameraRequestBus::Event(m_followCamera, &Camera::CameraRequestBus::Events::MakeActiveView);
            m_active = false;
            return;
        }

        AZ::Transform followTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(followTransform, m_followCamera, &AZ::TransformInterface::GetWorldTM);
        const AZ::Vector3 forward = followTransform.GetBasisY().GetNormalized();

        m_position = followTransform.GetTranslation();
        m_yaw = AZ::Atan2(-forward.GetX(), forward.GetY());
        m_pitch = AZ::GetClamp(AZStd::asin(forward.GetZ()), -MaxPitch, MaxPitch);

        Camera::CameraRequestBus::Event(GetEntityId(), &Camera::CameraRequestBus::Events::MakeActiveView);
        m_active = true;
    }

    bool FreeCameraComponent::OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputChannelId& channel = inputChannel.GetInputChannelId();

        if (channel == Key::AlphanumericC && inputChannel.IsStateBegan())
        {
            ToggleView();
            return false;
        }

        // Tracked while inactive too, so that a key held across the toggle is not stuck.
        if (channel == Mouse::Button::Right)
        {
            m_looking = inputChannel.IsActive();
        }
        else if (channel == Key::ModifierShiftL)
        {
            m_fast = inputChannel.IsActive();
        }
        else if (m_active && m_looking && channel == Mouse::Movement::X)
        {
            m_yaw -= inputChannel.GetValue() * m_lookSensitivity;
        }
        else if (m_active && m_looking && channel == Mouse::Movement::Y)
        {
            m_pitch = AZ::GetClamp(m_pitch - inputChannel.GetValue() * m_lookSensitivity, -MaxPitch, MaxPitch);
        }

        AZ::Vector3 moveInput = AZ::Vector3::CreateZero();
        for (size_t i = 0; i < AZ_ARRAY_SIZE(MoveKeys); ++i)
        {
            const MoveKey& key = MoveKeys[i];
            if (channel == key.m_channel)
            {
                m_keys[i] = inputChannel.IsActive();
            }
            if (m_keys[i])
            {
                moveInput += AZ::Vector3(key.m_right, key.m_forward, key.m_up);
            }
        }
        m_moveInput = moveInput;
        return false;
    }

    void FreeCameraComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_active)
        {
            return;
        }

        // Real time, so that the camera does not slow down with the simulation.
        const float realDeltaTime = AZ::TimeUsToSeconds(AZ::GetRealTickDeltaTimeUs());
        const AZ::Quaternion rotation = AZ::Quaternion::CreateRotationZ(m_yaw) * AZ::Quaternion::CreateRotationX(m_pitch);
        const float speed = m_fast ? m_speed * m_fastMultiplier : m_speed;

        m_position += rotation.TransformVector(m_moveInput) * speed * realDeltaTime;

        AZ::TransformBus::Event(
            GetEntityId(), &AZ::TransformInterface::SetWorldTM, AZ::Transform::CreateFromQuaternionAndTranslation(rotation, m_position));
    }
} // namespace Multirotor
