
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <DroneSimSystemComponent.h>

namespace DroneSim
{
    /// System component for DroneSim editor
    class DroneSimEditorSystemComponent
        : public DroneSimSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = DroneSimSystemComponent;
    public:
        AZ_COMPONENT(DroneSimEditorSystemComponent, "{DA155A9F-289A-40B9-8876-C593DD2758CF}", BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        DroneSimEditorSystemComponent();
        ~DroneSimEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace DroneSim
