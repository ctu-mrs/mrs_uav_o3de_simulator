#include "CameraResponseComponent.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace Multirotor
{
    void CameraResponseComponent::Reflect(AZ::ReflectContext* context)
    {
        CameraResponseConfiguration::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<CameraResponseComponent, AZ::Component>()->Version(1)->Field("Config", &CameraResponseComponent::m_config);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<CameraResponseComponent>("Camera response", "Low dynamic range response and auto exposure of a ROS 2 camera")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseComponent::m_config, "Configuration", "")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void CameraResponseComponent::Activate()
    {
        m_response.Configure(m_config);
        ROS2Sensors::CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void CameraResponseComponent::Deactivate()
    {
        ROS2Sensors::CameraPostProcessingRequestBus::Handler::BusDisconnect();
    }

    void CameraResponseComponent::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        // The bus also carries other images of this camera, such as depth, and the handler must leave those alone.
        if (image.encoding != "rgba8")
        {
            return;
        }

        const double time = image.header.stamp.sec + image.header.stamp.nanosec * 1e-9;
        m_response.Process(image.data.data(), image.width, image.height, image.step, time);
    }

    AZ::u8 CameraResponseComponent::GetPriority() const
    {
        // Before encoding conversions, which may change the format this expects.
        return ROS2Sensors::CameraPostProcessingRequests::MAX_PRIORITY;
    }
} // namespace Multirotor
