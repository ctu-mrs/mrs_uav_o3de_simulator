#include <AzCore/Casting/numeric_cast.h>
#include <AzTest/AzTest.h>

#include <Clients/CameraResponse.h>

#include <cmath>

namespace UnitTest
{
    using Multirotor::CameraResponse;
    using Multirotor::CameraResponseConfiguration;

    class CameraResponseTest : public ::testing::Test
    {
    protected:
        //! The response that leaves an image as it is, except for rounding.
        static CameraResponseConfiguration Neutral()
        {
            CameraResponseConfiguration config;
            config.m_blackLevel = 0.0f;
            config.m_whiteLevel = 1.0f;
            config.m_contrast = 1.0f;
            config.m_autoExposure = false;
            config.m_exposure = 0.0f;
            return config;
        }

        static CameraResponseConfiguration AutoExposure()
        {
            CameraResponseConfiguration config = Neutral();
            config.m_autoExposure = true;
            config.m_minExposure = -6.0f;
            config.m_maxExposure = 6.0f;
            return config;
        }

        //! A 16 x 16 image of one grey level, with an alpha of 200.
        static AZStd::vector<AZ::u8> Grey(AZ::u8 level)
        {
            AZStd::vector<AZ::u8> image(16 * 16 * 4, level);
            for (size_t i = 3; i < image.size(); i += 4)
            {
                image[i] = 200;
            }
            return image;
        }

        AZ::u8 ProcessGrey(AZ::u8 level, double time = 0.0)
        {
            AZStd::vector<AZ::u8> image = Grey(level);
            m_response.Process(image.data(), 16, 16, 16 * 4, time);
            return image[0];
        }

        CameraResponse m_response;
    };

    TEST_F(CameraResponseTest, NeutralKeepsTheImage)
    {
        m_response.Configure(Neutral());
        for (int level = 0; level < 256; ++level)
        {
            EXPECT_NEAR(ProcessGrey(aznumeric_cast<AZ::u8>(level)), level, 1);
        }
    }

    TEST_F(CameraResponseTest, ClipsBlacksAndWhites)
    {
        CameraResponseConfiguration config = Neutral();
        config.m_blackLevel = 0.1f;
        config.m_whiteLevel = 0.5f;
        m_response.Configure(config);

        // In linear signal, 0.1^(1/2.2) * 255 = 89.5, and 0.5^(1/2.2) * 255 = 186.1.
        EXPECT_EQ(ProcessGrey(80), 0);
        EXPECT_EQ(ProcessGrey(89), 0);
        EXPECT_GT(ProcessGrey(120), 0);
        EXPECT_LT(ProcessGrey(170), 255);
        EXPECT_EQ(ProcessGrey(187), 255);
        EXPECT_EQ(ProcessGrey(255), 255);
    }

    TEST_F(CameraResponseTest, ResponseIsMonotonic)
    {
        CameraResponseConfiguration config;
        config.m_autoExposure = false;
        m_response.Configure(config);

        int previous = 0;
        for (int level = 0; level < 256; ++level)
        {
            const int output = ProcessGrey(aznumeric_cast<AZ::u8>(level));
            EXPECT_GE(output, previous);
            previous = output;
        }
    }

    TEST_F(CameraResponseTest, ContrastPivotsAboutMidGrey)
    {
        CameraResponseConfiguration config = Neutral();
        config.m_contrast = 2.0f;
        m_response.Configure(config);

        EXPECT_NEAR(ProcessGrey(128), 128, 2);
        EXPECT_NEAR(ProcessGrey(160), 192, 2);
        EXPECT_EQ(ProcessGrey(255), 255);
        EXPECT_EQ(ProcessGrey(0), 0);
    }

    TEST_F(CameraResponseTest, ManualExposureIsAGainOnTheLinearSignal)
    {
        CameraResponseConfiguration config = Neutral();
        config.m_exposure = 1.0f;
        m_response.Configure(config);

        // Twice the linear signal is 2^(1/2.2) times the encoded value.
        EXPECT_NEAR(ProcessGrey(100), 137, 1);
        EXPECT_EQ(ProcessGrey(200), 255);
    }

    TEST_F(CameraResponseTest, KeepsAlpha)
    {
        CameraResponseConfiguration config;
        config.m_noiseStdDev = 10.0f;
        m_response.Configure(config);

        AZStd::vector<AZ::u8> image = Grey(100);
        m_response.Process(image.data(), 16, 16, 16 * 4, 0.0);
        for (size_t i = 3; i < image.size(); i += 4)
        {
            EXPECT_EQ(image[i], 200);
        }
    }

    TEST_F(CameraResponseTest, HonorsRowPadding)
    {
        CameraResponseConfiguration config = Neutral();
        config.m_exposure = 1.0f;
        m_response.Configure(config);

        constexpr AZ::u32 Step = 16 * 4 + 8;
        AZStd::vector<AZ::u8> image(16 * Step, 100);
        m_response.Process(image.data(), 16, 16, Step, 0.0);
        EXPECT_EQ(image[Step - 1], 100); // The padding.
        EXPECT_NEAR(image[Step], 137, 1); // The next row.
    }

    TEST_F(CameraResponseTest, AutoExposureStartsAdapted)
    {
        CameraResponseConfiguration config = AutoExposure();
        config.m_exposureTimeConstant = 1.0f;
        m_response.Configure(config);

        EXPECT_NEAR(ProcessGrey(20, 100.0), config.m_targetBrightness * 255.0f, 2.0f);
    }

    TEST_F(CameraResponseTest, AutoExposureFollowsWithLag)
    {
        CameraResponseConfiguration config = AutoExposure();
        config.m_exposureTimeConstant = 1.0f;
        m_response.Configure(config);

        ProcessGrey(20, 0.0);
        const float dark = m_response.GetExposure();

        // One time constant later, 63% of the way to the exposure of a brighter image.
        ProcessGrey(100, 1.0);
        const float settled = dark - 2.2f * std::log2(100.0f / 20.0f);
        EXPECT_NEAR(m_response.GetExposure(), dark + 0.632f * (settled - dark), 0.01f);

        // The first bright image is overexposed, as on a real camera, and it then settles.
        ProcessGrey(100, 20.0);
        EXPECT_NEAR(m_response.GetExposure(), settled, 0.01f);
    }

    TEST_F(CameraResponseTest, AutoExposureOverexposesASuddenlyBrightScene)
    {
        CameraResponseConfiguration config = AutoExposure();
        config.m_exposureTimeConstant = 1.0f;
        m_response.Configure(config);

        ProcessGrey(10, 0.0);
        EXPECT_EQ(ProcessGrey(200, 0.03), 255);
    }

    TEST_F(CameraResponseTest, AutoExposureStopsAtItsLimits)
    {
        CameraResponseConfiguration config = AutoExposure();
        config.m_minExposure = -1.0f;
        config.m_maxExposure = 2.0f;
        config.m_exposureTimeConstant = 0.0f;
        m_response.Configure(config);

        ProcessGrey(0, 1.0);
        EXPECT_FLOAT_EQ(m_response.GetExposure(), 2.0f);
        ProcessGrey(255, 2.0);
        EXPECT_FLOAT_EQ(m_response.GetExposure(), -1.0f);
    }

    TEST_F(CameraResponseTest, OlderImageLeavesTheExposure)
    {
        CameraResponseConfiguration config = AutoExposure();
        config.m_exposureTimeConstant = 0.0f;
        m_response.Configure(config);

        ProcessGrey(50, 10.0);
        const float exposure = m_response.GetExposure();
        ProcessGrey(200, 9.0);
        EXPECT_FLOAT_EQ(m_response.GetExposure(), exposure);
    }

    TEST_F(CameraResponseTest, NoiseHasTheConfiguredDeviationAndSparesClippedPixels)
    {
        CameraResponseConfiguration config = Neutral();
        config.m_noiseStdDev = 4.0f;
        m_response.Configure(config);

        constexpr AZ::u32 Size = 256;
        AZStd::vector<AZ::u8> grey(Size * Size * 4, 128);
        m_response.Process(grey.data(), Size, Size, Size * 4, 0.0);

        double sum = 0.0;
        double squares = 0.0;
        for (size_t i = 0; i < grey.size(); i += 4)
        {
            const double value = grey[i];
            sum += value;
            squares += value * value;
        }
        const double count = Size * Size;
        const double mean = sum / count;
        EXPECT_NEAR(mean, 128.0, 0.5);
        EXPECT_NEAR(std::sqrt(squares / count - mean * mean), 4.0, 0.3);

        EXPECT_EQ(ProcessGrey(0), 0);
        EXPECT_EQ(ProcessGrey(255), 255);
    }

    TEST_F(CameraResponseTest, NoiseDiffersBetweenImages)
    {
        CameraResponseConfiguration config = Neutral();
        config.m_noiseStdDev = 4.0f;
        m_response.Configure(config);

        AZStd::vector<AZ::u8> first = Grey(128);
        AZStd::vector<AZ::u8> second = Grey(128);
        m_response.Process(first.data(), 16, 16, 16 * 4, 0.0);
        m_response.Process(second.data(), 16, 16, 16 * 4, 1.0);
        EXPECT_NE(first, second);
    }
} // namespace UnitTest
