
#include "MultirotorSystemComponent.h"

#include <Multirotor/MultirotorTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace Multirotor
{
    AZ_COMPONENT_IMPL(MultirotorSystemComponent, "MultirotorSystemComponent",
        MultirotorSystemComponentTypeId);

    void MultirotorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MultirotorSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void MultirotorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("MultirotorService"));
    }

    void MultirotorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("MultirotorService"));
    }

    void MultirotorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void MultirotorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    MultirotorSystemComponent::MultirotorSystemComponent()
    {
        if (MultirotorInterface::Get() == nullptr)
        {
            MultirotorInterface::Register(this);
        }
    }

    MultirotorSystemComponent::~MultirotorSystemComponent()
    {
        if (MultirotorInterface::Get() == this)
        {
            MultirotorInterface::Unregister(this);
        }
    }

    void MultirotorSystemComponent::Init()
    {
    }

    void MultirotorSystemComponent::Activate()
    {
        MultirotorRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void MultirotorSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        MultirotorRequestBus::Handler::BusDisconnect();
    }

    void MultirotorSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace Multirotor
