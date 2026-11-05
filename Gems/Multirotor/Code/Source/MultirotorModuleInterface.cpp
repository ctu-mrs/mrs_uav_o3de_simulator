
#include "MultirotorModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <Multirotor/MultirotorTypeIds.h>

#include <Clients/FreeCameraComponent.h>
#include <Clients/MultirotorComponent.h>
#include <Clients/MultirotorSystemComponent.h>

namespace Multirotor
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(MultirotorModuleInterface,
        "MultirotorModuleInterface", MultirotorModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(MultirotorModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(MultirotorModuleInterface, AZ::SystemAllocator);

    MultirotorModuleInterface::MultirotorModuleInterface()
    {
        // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
        // Add ALL components descriptors associated with this gem to m_descriptors.
        // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
        // This happens through the [MyComponent]::Reflect() function.
        m_descriptors.insert(m_descriptors.end(), {
            MultirotorSystemComponent::CreateDescriptor(),
            MultirotorComponent::CreateDescriptor(),
            FreeCameraComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList MultirotorModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<MultirotorSystemComponent>(),
        };
    }
} // namespace Multirotor
