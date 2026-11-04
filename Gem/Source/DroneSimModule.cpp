
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "DroneSimSystemComponent.h"
#include "DroneSimSampleComponent.h"

namespace DroneSim
{
    class DroneSimModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(DroneSimModule, "{1AF9D0C6-300E-4EA6-8E1F-86C5BE76C413}", AZ::Module);
        AZ_CLASS_ALLOCATOR(DroneSimModule, AZ::SystemAllocator);

        DroneSimModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                DroneSimSystemComponent::CreateDescriptor(),
                DroneSimSampleComponent::CreateDescriptor(),
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

AZ_DECLARE_MODULE_CLASS(Gem_DroneSim, DroneSim::DroneSimModule)
