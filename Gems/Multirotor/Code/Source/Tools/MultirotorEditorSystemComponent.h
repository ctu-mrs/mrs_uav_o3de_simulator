
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/MultirotorSystemComponent.h>

namespace Multirotor
{
    /// System component for Multirotor editor
    class MultirotorEditorSystemComponent
        : public MultirotorSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = MultirotorSystemComponent;
    public:
        AZ_COMPONENT_DECL(MultirotorEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        MultirotorEditorSystemComponent();
        ~MultirotorEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace Multirotor
