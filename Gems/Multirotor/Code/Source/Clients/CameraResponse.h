#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/mutex.h>

namespace AZ
{
    class ReflectContext;
}

namespace Multirotor
{
    //! Response of a low dynamic range sensor. Levels are normalized to [0, 1].
    struct CameraResponseConfiguration
    {
        AZ_TYPE_INFO(CameraResponseConfiguration, "{B81F2D46-07C3-4A95-8E6D-3F1A9C5E7B24}");
        static void Reflect(AZ::ReflectContext* context);

        float m_blackLevel = 0.03f; //!< Linear signal at and below this is black.
        float m_whiteLevel = 0.7f; //!< Linear signal at and above this is white.
        float m_contrast = 1.2f; //!< Slope about mid grey, in encoded values. Clips both ends further.
        float m_gamma = 2.2f; //!< Relates encoded values to linear signal, for both the rendered and the output image.
        float m_noiseStdDev = 0.0f; //!< Gaussian noise in 8-bit levels. Pixels clipped to black or white stay clean.

        bool m_autoExposure = true;
        float m_exposure = 0.0f; //!< Exposure without auto exposure [EV].
        float m_targetBrightness = 0.4f; //!< Encoded level auto exposure brings the mean of the image to.
        float m_exposureTimeConstant = 0.5f; //!< Lag of auto exposure [s, sim time]. 0 follows the image immediately.
        float m_minExposure = -2.0f; //!< [EV]
        float m_maxExposure = 3.0f; //!< [EV]
    };

    //! Remaps rendered images to what a low dynamic range camera would produce: exposure as a gain on the linear signal,
    //! then hard clipping of blacks and whites. Pure computation, safe to call from several threads, as frames can be
    //! processed concurrently. The input is already tone mapped to 8 bits, so lowering the exposure cannot recover
    //! highlights: they turn grey.
    class CameraResponse
    {
    public:
        //! Not safe while images are being processed.
        void Configure(const CameraResponseConfiguration& config);

        //! Processes an rgba8 image in place, alpha stays as it is.
        //! @param time of the image [s]. An image older than the last one leaves the exposure as it is.
        void Process(AZ::u8* data, AZ::u32 width, AZ::u32 height, AZ::u32 step, double time);

        float GetExposure() const; //!< [EV]

    private:
        float MeasureExposure(const AZ::u8* data, AZ::u32 width, AZ::u32 height, AZ::u32 step) const;
        float UpdateExposure(float target, double time);

        CameraResponseConfiguration m_config;
        float m_linear[256] = {}; //!< From an encoded level to the linear signal.
        AZStd::vector<AZ::s8> m_noise; //!< Precomputed, as a generator per sample is too slow for an image.

        mutable AZStd::mutex m_mutex;
        bool m_adapted = false; //!< Guarded by m_mutex.
        float m_currentExposure = 0.0f; //!< Guarded by m_mutex. [EV]
        double m_lastTime = 0.0; //!< Guarded by m_mutex.
        AZ::u64 m_frame = 0; //!< Guarded by m_mutex. Seeds the noise of each image.
    };
} // namespace Multirotor
