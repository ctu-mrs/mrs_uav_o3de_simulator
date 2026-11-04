
#include <AzCore/Serialization/SerializeContext.h>
#include "DroneSimEditorSystemComponent.h"

namespace DroneSim
{
    void DroneSimEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<DroneSimEditorSystemComponent, DroneSimSystemComponent>()
                ->Version(0);
        }
    }

    DroneSimEditorSystemComponent::DroneSimEditorSystemComponent() = default;

    DroneSimEditorSystemComponent::~DroneSimEditorSystemComponent() = default;

    void DroneSimEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("DroneSimEditorService"));
    }

    void DroneSimEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("DroneSimEditorService"));
    }

    void DroneSimEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void DroneSimEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void DroneSimEditorSystemComponent::Activate()
    {
        DroneSimSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void DroneSimEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        DroneSimSystemComponent::Deactivate();
    }

} // namespace DroneSim
