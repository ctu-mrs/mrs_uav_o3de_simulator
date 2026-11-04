
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <DroneSimSystemComponent.h>

namespace DroneSim
{
    class DroneSimModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(DroneSimModuleInterface, "{E87AC1F9-1384-4165-9354-D6E3822925CE}", AZ::Module);
        AZ_CLASS_ALLOCATOR(DroneSimModuleInterface, AZ::SystemAllocator, 0);

        DroneSimModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                DroneSimSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<DroneSimSystemComponent>(),
            };
        }
    };
}// namespace DroneSim
