
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/string/string.h>
#include <Multirotor/MultirotorBus.h>

#include <cstdio>

namespace Multirotor
{
    class MultirotorSystemComponent
        : public AZ::Component
        , protected MultirotorRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(MultirotorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        MultirotorSystemComponent();
        ~MultirotorSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // MultirotorRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        ////////////////////////////////////////////////////////////////////////

    private:
        AZStd::string m_frameLogPath; //!< --drone_frame_log, empty: off
        FILE* m_frameLog = nullptr; //!< Opened on the first tick.
        double m_previousFrame = 0.0; //!< Steady clock [s].

        void ProfileTick();
        AZStd::string m_profileDirectory; //!< --drone_profile_directory, empty: off
        float m_profileAfter = 60.0f; //!< [s] after the first tick
        int m_profileCount = 5;
        int m_profileTaken = 0;
        double m_profileStart = 0.0; //!< Steady clock [s].
    };

} // namespace Multirotor
