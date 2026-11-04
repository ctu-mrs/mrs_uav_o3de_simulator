
#pragma once

#include <Multirotor/MultirotorTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace Multirotor
{
    class MultirotorRequests
    {
    public:
        AZ_RTTI(MultirotorRequests, MultirotorRequestsTypeId);
        virtual ~MultirotorRequests() = default;
        // Put your public methods here
    };

    class MultirotorBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using MultirotorRequestBus = AZ::EBus<MultirotorRequests, MultirotorBusTraits>;
    using MultirotorInterface = AZ::Interface<MultirotorRequests>;

} // namespace Multirotor
