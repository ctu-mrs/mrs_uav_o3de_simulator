
#include <AzCore/Serialization/SerializeContext.h>
#include "MultirotorEditorSystemComponent.h"

#include <Multirotor/MultirotorTypeIds.h>

namespace Multirotor
{
    AZ_COMPONENT_IMPL(MultirotorEditorSystemComponent, "MultirotorEditorSystemComponent",
        MultirotorEditorSystemComponentTypeId, BaseSystemComponent);

    void MultirotorEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MultirotorEditorSystemComponent, MultirotorSystemComponent>()
                ->Version(0);
        }
    }

    MultirotorEditorSystemComponent::MultirotorEditorSystemComponent() = default;

    MultirotorEditorSystemComponent::~MultirotorEditorSystemComponent() = default;

    void MultirotorEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("MultirotorEditorService"));
    }

    void MultirotorEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("MultirotorEditorService"));
    }

    void MultirotorEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void MultirotorEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void MultirotorEditorSystemComponent::Activate()
    {
        MultirotorSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void MultirotorEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        MultirotorSystemComponent::Deactivate();
    }

} // namespace Multirotor
