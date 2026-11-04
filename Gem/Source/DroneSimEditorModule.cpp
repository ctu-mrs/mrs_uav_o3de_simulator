
#include <DroneSimModuleInterface.h>
#include "DroneSimEditorSystemComponent.h"

#include "DroneSimSampleComponent.h"
namespace DroneSim
{
    class DroneSimEditorModule
        : public DroneSimModuleInterface
    {
    public:
        AZ_RTTI(DroneSimEditorModule, "{1AF9D0C6-300E-4EA6-8E1F-86C5BE76C413}", DroneSimModuleInterface);
        AZ_CLASS_ALLOCATOR(DroneSimEditorModule, AZ::SystemAllocator, 0);

        DroneSimEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                DroneSimEditorSystemComponent::CreateDescriptor(),
                DroneSimSampleComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<DroneSimEditorSystemComponent>(),
            };
        }
    };
}// namespace DroneSim

AZ_DECLARE_MODULE_CLASS(Gem_DroneSim, DroneSim::DroneSimEditorModule)
