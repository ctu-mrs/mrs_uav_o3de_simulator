/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace DroneSim
{
    class DroneSimRequests
    {
    public:
        AZ_RTTI(DroneSimRequests, "{F04749B7-F599-4E1E-92D7-A4A4D8F12F9D}");
        virtual ~DroneSimRequests() = default;
        // Put your public methods here
    };

    class DroneSimBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using DroneSimRequestBus = AZ::EBus<DroneSimRequests, DroneSimBusTraits>;
    using DroneSimInterface = AZ::Interface<DroneSimRequests>;

} // namespace DroneSim
