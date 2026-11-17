
#include "MultirotorSystemComponent.h"

#include <Multirotor/MultirotorTypeIds.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Debug/ProfilerBus.h>
#include <AzCore/std/string/conversions.h>
#include <AzCore/Settings/CommandLine.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <chrono>
#include <cstdio>


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

        // --drone_frame_log=<file> writes every frame's wall-clock interval as CSV (unix_s,frame_ms), for benchmarks.
        const AZ::CommandLine* commandLine = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(commandLine, &AZ::ComponentApplicationRequests::GetAzCommandLine);
        if (commandLine && commandLine->GetNumSwitchValues("drone_frame_log") > 0)
        {
            m_frameLogPath = commandLine->GetSwitchValue("drone_frame_log", 0);
        }
        // --drone_profile_directory=<dir> saves CPU profiler captures of single frames, one per second from
        // --drone_profile_after seconds (default 60) for --drone_profile_count frames (default 5). Needs the Profiler Gem.
        if (commandLine && commandLine->GetNumSwitchValues("drone_profile_directory") > 0)
        {
            m_profileDirectory = commandLine->GetSwitchValue("drone_profile_directory", 0);
            if (commandLine->GetNumSwitchValues("drone_profile_after") > 0)
            {
                m_profileAfter = AZStd::stof(commandLine->GetSwitchValue("drone_profile_after", 0));
            }
            if (commandLine->GetNumSwitchValues("drone_profile_count") > 0)
            {
                m_profileCount = AZStd::stoi(commandLine->GetSwitchValue("drone_profile_count", 0));
            }
        }
    }

    void MultirotorSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        MultirotorRequestBus::Handler::BusDisconnect();
        if (m_frameLog)
        {
            fclose(m_frameLog);
            m_frameLog = nullptr;
        }
    }

    void MultirotorSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        ProfileTick();

        // Raw wall-clock intervals between frames, not the simulation's tick delta, which drone users often override.
        if (m_frameLogPath.empty())
        {
            return;
        }
        if (!m_frameLog)
        {
            m_frameLog = fopen(m_frameLogPath.c_str(), "w");
            if (!m_frameLog)
            {
                AZ_Error("Multirotor", false, "Cannot open the drone_frame_log file %s", m_frameLogPath.c_str());
                m_frameLogPath.clear();
                return;
            }
            fputs("unix_s,frame_ms\n", m_frameLog);
        }
        const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (m_previousFrame > 0.0)
        {
            const double utc = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            fprintf(m_frameLog, "%.6f,%.6f\n", utc, (now - m_previousFrame) * 1000.0);
            fflush(m_frameLog);
        }
        m_previousFrame = now;
    }

    void MultirotorSystemComponent::ProfileTick()
    {
        if (m_profileDirectory.empty() || m_profileTaken >= m_profileCount)
        {
            return;
        }
        const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (m_profileStart <= 0.0)
        {
            m_profileStart = now;
        }
        if (now - m_profileStart < m_profileAfter + m_profileTaken)
        {
            return;
        }
        auto* profiler = AZ::Debug::ProfilerSystemInterface::Get();
        if (!profiler)
        {
            AZ_Error("Multirotor", false, "--drone_profile_directory needs the Profiler Gem");
            m_profileDirectory.clear();
            return;
        }
        profiler->CaptureFrame(AZStd::string::format("%s/cpu-%02d.json", m_profileDirectory.c_str(), ++m_profileTaken));
    }

} // namespace Multirotor
