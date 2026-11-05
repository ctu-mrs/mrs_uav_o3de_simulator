#pragma once

#include "CameraResponse.h"

#include <AzCore/Component/Component.h>
#include <Multirotor/MultirotorTypeIds.h>
#include <ROS2Sensors/Camera/CameraPostProcessingRequestBus.h>

namespace Multirotor
{
    //! Makes the images of the ROS 2 camera on this entity look like those of a low dynamic range camera (see CameraResponse).
    //! It works on the published image, so it does not affect the game view, and it also reaches what is drawn after the
    //! engine's post-processing, such as splats.
    class CameraResponseComponent
        : public AZ::Component
        , private ROS2Sensors::CameraPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(CameraResponseComponent, CameraResponseComponentTypeId, AZ::Component);

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    private:
        // ROS2Sensors::CameraPostProcessingRequestBus::Handler
        void ApplyPostProcessing(sensor_msgs::msg::Image& image) override;
        AZ::u8 GetPriority() const override;

        CameraResponseConfiguration m_config;
        CameraResponse m_response;
    };
} // namespace Multirotor
