
#pragma once

namespace Multirotor
{
    // System Component TypeIds
    inline constexpr const char* MultirotorSystemComponentTypeId = "{73F60590-9A75-4558-8065-D45E01A4C36F}";
    inline constexpr const char* MultirotorEditorSystemComponentTypeId = "{38203D39-D11D-4C72-AA3B-54A9E9E946FE}";

    // Component TypeIds
    inline constexpr const char* MultirotorComponentTypeId = "{E4B7C2A9-5D16-4F38-9A0B-7C3E1D2F4A51}";
    inline constexpr const char* FreeCameraComponentTypeId = "{2F6D8A14-93C7-4E1B-B5A2-6C0E7D1F3B92}";
    inline constexpr const char* RangefinderComponentTypeId = "{7C1E4D90-2B3A-4F6E-8D15-A94C0B7E6F21}";
    inline constexpr const char* WorldOdometrySensorComponentTypeId = "{72BD2D17-DA24-43AC-B50B-4ED43D455152}";

    // Module derived classes TypeIds
    inline constexpr const char* MultirotorModuleInterfaceTypeId = "{BCC94CE4-5401-4DC0-8838-5AA40FE4DACF}";
    inline constexpr const char* MultirotorModuleTypeId = "{2CD7EFF6-42A8-4378-9CEE-B4E12CAD01E1}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* MultirotorEditorModuleTypeId = MultirotorModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* MultirotorRequestsTypeId = "{2C3F794A-43D0-4E43-A2CC-AA36F91EFDCF}";
} // namespace Multirotor
