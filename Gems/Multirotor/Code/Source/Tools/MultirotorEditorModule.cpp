
#include <Multirotor/MultirotorTypeIds.h>
#include <MultirotorModuleInterface.h>
#include "MultirotorEditorSystemComponent.h"

namespace Multirotor
{
    class MultirotorEditorModule
        : public MultirotorModuleInterface
    {
    public:
        AZ_RTTI(MultirotorEditorModule, MultirotorEditorModuleTypeId, MultirotorModuleInterface);
        AZ_CLASS_ALLOCATOR(MultirotorEditorModule, AZ::SystemAllocator);

        MultirotorEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                MultirotorEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<MultirotorEditorSystemComponent>(),
            };
        }
    };
}// namespace Multirotor

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), Multirotor::MultirotorEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_Multirotor_Editor, Multirotor::MultirotorEditorModule)
#endif
