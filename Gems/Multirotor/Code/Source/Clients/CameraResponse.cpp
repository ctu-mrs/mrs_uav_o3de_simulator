#include "CameraResponse.h"

#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace Multirotor
{
    namespace
    {
        constexpr AZ::u32 MeteringStride = 8; //!< Every n-th pixel in each direction is enough to measure the brightness.
        constexpr size_t NoiseTableSize = 1 << 16;
        constexpr float MinMeasuredSignal = 1e-4f; //!< Keeps the exposure of a black image finite.

        AZ::u64 NextRandom(AZ::u64& state)
        {
            // xorshift64: only picks where each row starts in the noise table.
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            return state;
        }
    } // namespace

    void CameraResponseConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<CameraResponseConfiguration>()
                ->Version(1)
                ->Field("BlackLevel", &CameraResponseConfiguration::m_blackLevel)
                ->Field("WhiteLevel", &CameraResponseConfiguration::m_whiteLevel)
                ->Field("Contrast", &CameraResponseConfiguration::m_contrast)
                ->Field("Gamma", &CameraResponseConfiguration::m_gamma)
                ->Field("NoiseStdDev", &CameraResponseConfiguration::m_noiseStdDev)
                ->Field("AutoExposure", &CameraResponseConfiguration::m_autoExposure)
                ->Field("Exposure", &CameraResponseConfiguration::m_exposure)
                ->Field("TargetBrightness", &CameraResponseConfiguration::m_targetBrightness)
                ->Field("ExposureTimeConstant", &CameraResponseConfiguration::m_exposureTimeConstant)
                ->Field("MinExposure", &CameraResponseConfiguration::m_minExposure)
                ->Field("MaxExposure", &CameraResponseConfiguration::m_maxExposure);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<CameraResponseConfiguration>("Camera response", "Low dynamic range sensor")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_blackLevel, "Black level", "Linear signal clipped to black")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_whiteLevel, "White level", "Linear signal clipped to white")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_contrast, "Contrast", "Slope about mid grey")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_gamma, "Gamma", "Of the input and the output")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_noiseStdDev, "Noise", "Standard deviation [8-bit levels]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_autoExposure, "Auto exposure", "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_exposure, "Exposure", "Without auto exposure [EV]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_targetBrightness, "Target brightness", "Mean output level")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_exposureTimeConstant, "Exposure lag", "[s, sim time]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_minExposure, "Min exposure", "[EV]")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraResponseConfiguration::m_maxExposure, "Max exposure", "[EV]");
            }
        }
    }

    void CameraResponse::Configure(const CameraResponseConfiguration& config)
    {
        m_config = config;
        m_adapted = false;

        for (int level = 0; level < 256; ++level)
        {
            m_linear[level] = std::pow(aznumeric_cast<float>(level) / 255.0f, m_config.m_gamma);
        }

        m_noise.clear();
        if (m_config.m_noiseStdDev > 0.0f)
        {
            std::mt19937 generator(1);
            std::normal_distribution<float> distribution(0.0f, m_config.m_noiseStdDev);
            m_noise.resize(NoiseTableSize);
            for (AZ::s8& sample : m_noise)
            {
                sample = aznumeric_cast<AZ::s8>(std::lround(AZ::GetClamp(distribution(generator), -127.0f, 127.0f)));
            }
        }
    }

    float CameraResponse::MeasureExposure(const AZ::u8* data, AZ::u32 width, AZ::u32 height, AZ::u32 step) const
    {
        float sum = 0.0f;
        AZ::u32 count = 0;
        for (AZ::u32 row = 0; row < height; row += MeteringStride)
        {
            const AZ::u8* pixels = data + size_t{ row } * step;
            for (AZ::u32 column = 0; column < width; column += MeteringStride)
            {
                const AZ::u8* pixel = pixels + size_t{ column } * 4;
                sum += 0.2126f * m_linear[pixel[0]] + 0.7152f * m_linear[pixel[1]] + 0.0722f * m_linear[pixel[2]];
                ++count;
            }
        }

        const float mean = AZ::GetMax(sum / aznumeric_cast<float>(AZ::GetMax(count, 1u)), MinMeasuredSignal);
        const float target = std::pow(m_config.m_targetBrightness, m_config.m_gamma);
        return AZ::GetClamp(std::log2(target / mean), m_config.m_minExposure, m_config.m_maxExposure);
    }

    float CameraResponse::UpdateExposure(float target, double time)
    {
        AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
        if (!m_adapted)
        {
            m_currentExposure = target;
            m_lastTime = time;
            m_adapted = true;
        }
        else if (time > m_lastTime)
        {
            const double dt = time - m_lastTime;
            const float tau = m_config.m_exposureTimeConstant;
            const float blend = tau > 0.0f ? 1.0f - aznumeric_cast<float>(std::exp(-dt / tau)) : 1.0f;
            m_currentExposure += blend * (target - m_currentExposure);
            m_lastTime = time;
        }
        return m_currentExposure;
    }

    void CameraResponse::Process(AZ::u8* data, AZ::u32 width, AZ::u32 height, AZ::u32 step, double time)
    {
        const float exposure =
            m_config.m_autoExposure ? UpdateExposure(MeasureExposure(data, width, height, step), time) : UpdateExposure(m_config.m_exposure, time);

        // Rebuilt for every image, as the exposure changes.
        AZ::u8 table[256];
        const float gain = std::pow(2.0f, exposure);
        const float range = AZ::GetMax(m_config.m_whiteLevel - m_config.m_blackLevel, 1e-6f);
        for (int level = 0; level < 256; ++level)
        {
            const float signal = AZ::GetClamp((m_linear[level] * gain - m_config.m_blackLevel) / range, 0.0f, 1.0f);
            const float encoded = std::pow(signal, 1.0f / m_config.m_gamma);
            const float contrasted = 0.5f + (encoded - 0.5f) * m_config.m_contrast;
            table[level] = aznumeric_cast<AZ::u8>(std::lround(AZ::GetClamp(contrasted, 0.0f, 1.0f) * 255.0f));
        }

        AZ::u64 random;
        {
            AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
            random = 0x9E3779B97F4A7C15ull * ++m_frame;
        }

        for (AZ::u32 row = 0; row < height; ++row)
        {
            AZ::u8* pixel = data + size_t{ row } * step;
            size_t noiseIndex = m_noise.empty() ? 0 : NextRandom(random) % m_noise.size();
            for (AZ::u32 column = 0; column < width; ++column, pixel += 4)
            {
                for (int channel = 0; channel < 3; ++channel)
                {
                    int value = table[pixel[channel]];
                    if (!m_noise.empty())
                    {
                        if (value > 0 && value < 255)
                        {
                            value = AZ::GetClamp(value + m_noise[noiseIndex], 0, 255);
                        }
                        noiseIndex = (noiseIndex + 1) % m_noise.size();
                    }
                    pixel[channel] = aznumeric_cast<AZ::u8>(value);
                }
            }
        }
    }

    float CameraResponse::GetExposure() const
    {
        AZStd::lock_guard<AZStd::mutex> lock(m_mutex);
        return m_currentExposure;
    }
} // namespace Multirotor
