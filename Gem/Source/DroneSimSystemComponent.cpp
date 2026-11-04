

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "DroneSimSystemComponent.h"

namespace DroneSim
{
    void DroneSimSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<DroneSimSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<DroneSimSystemComponent>("DroneSim", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void DroneSimSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("DroneSimService"));
    }

    void DroneSimSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("DroneSimService"));
    }

    void DroneSimSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void DroneSimSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    DroneSimSystemComponent::DroneSimSystemComponent()
    {
        if (DroneSimInterface::Get() == nullptr)
        {
            DroneSimInterface::Register(this);
        }
    }

    DroneSimSystemComponent::~DroneSimSystemComponent()
    {
        if (DroneSimInterface::Get() == this)
        {
            DroneSimInterface::Unregister(this);
        }
    }

    void DroneSimSystemComponent::Init()
    {
    }

    void DroneSimSystemComponent::Activate()
    {
        DroneSimRequestBus::Handler::BusConnect();
    }

    void DroneSimSystemComponent::Deactivate()
    {
        DroneSimRequestBus::Handler::BusDisconnect();
    }
}
