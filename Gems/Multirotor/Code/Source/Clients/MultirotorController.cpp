#include "MultirotorController.h"

#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/std/math.h>

namespace Multirotor
{
    bool MultirotorController::Configure(const MultirotorConfiguration& config, const AZ::Vector3& inertiaDiagonal)
    {
        m_motor = config.m_motor;
        m_gains = config.m_controller;
        m_inertia = inertiaDiagonal;

        const size_t rotorCount = config.m_rotors.size();
        m_yawSign.resize(rotorCount);
        m_allocation.resize(rotorCount);
        m_rotorSpeeds.assign(rotorCount, 0.0f);
        m_rotorThrusts.assign(rotorCount, 0.0f);
        Reset();

        if (rotorCount < 4 || m_motor.m_thrustCoefficient <= 0.0f)
        {
            return false;
        }

        // Effectiveness matrix A (4 x N): column i maps rotor i thrust to (total thrust, roll, pitch, yaw torque).
        // A counter-clockwise rotor drives its propeller about +Z, so its reaction torque on the body is about -Z.
        const float yawPerThrust = m_motor.m_torqueCoefficient / m_motor.m_thrustCoefficient;
        AZStd::vector<AZ::Vector4> columns(rotorCount);
        for (size_t i = 0; i < rotorCount; ++i)
        {
            const AZ::Vector3& p = config.m_rotors[i].m_position;
            m_yawSign[i] = config.m_rotors[i].m_clockwise ? 1.0f : -1.0f;
            columns[i] = AZ::Vector4(1.0f, p.GetY(), -p.GetX(), m_yawSign[i] * yawPerThrust);
        }

        // Minimum-norm allocation: f = A^T (A A^T)^-1 w.
        AZ::Matrix4x4 aat = AZ::Matrix4x4::CreateZero();
        for (const AZ::Vector4& c : columns)
        {
            for (int row = 0; row < 4; ++row)
            {
                aat.SetRow(row, aat.GetRow(row) + c * c.GetElement(row));
            }
        }
        const AZ::Matrix4x4 aatInverse = aat.GetInverseFull();
        if (!(aat * aatInverse).IsClose(AZ::Matrix4x4::CreateIdentity(), 1e-3f))
        {
            return false;
        }
        for (size_t i = 0; i < rotorCount; ++i)
        {
            m_allocation[i] = aatInverse * columns[i];
        }
        return true;
    }

    void MultirotorController::Reset()
    {
        m_rateIntegral = AZ::Vector3::CreateZero();
        m_previousRateError = AZ::Vector3::CreateZero();
        m_hasPreviousRateError = false;
    }

    float MultirotorController::ThrottleToThrust(float throttle) const
    {
        const float omega = AZ::GetClamp(throttle, 0.0f, 1.0f) * m_motor.m_maxRotorSpeed;
        return aznumeric_cast<float>(m_allocation.size()) * m_motor.m_thrustCoefficient * omega * omega;
    }

    AZStd::vector<float> MultirotorController::Mix(float collectiveThrust, const AZ::Vector3& torque) const
    {
        const AZ::Vector4 wrench(collectiveThrust, torque.GetX(), torque.GetY(), torque.GetZ());
        const float maxThrust = m_motor.m_thrustCoefficient * m_motor.m_maxRotorSpeed * m_motor.m_maxRotorSpeed;
        AZStd::vector<float> thrusts(m_allocation.size());
        for (size_t i = 0; i < m_allocation.size(); ++i)
        {
            thrusts[i] = AZ::GetClamp(m_allocation[i].Dot(wrench), 0.0f, maxThrust);
        }
        return thrusts;
    }

    AZ::Vector3 MultirotorController::AttitudeControl(const AZ::Quaternion& desired, const AZ::Quaternion& current) const
    {
        // Error rotation expressed in the body frame; take the short way around.
        AZ::Quaternion error = current.GetConjugate() * desired;
        if (error.GetW() < 0.0f)
        {
            error = -error;
        }
        const AZ::Vector3 rate = m_gains.m_attitudeP * 2.0f * error.GetImaginary();
        return rate.GetClamp(-m_gains.m_maxBodyRate, m_gains.m_maxBodyRate);
    }

    AZ::Vector3 MultirotorController::RateControl(
        const AZ::Vector3& desiredRate, const AZ::Vector3& bodyRate, bool integrate, float dt)
    {
        const AZ::Vector3 error = desiredRate - bodyRate;

        if (integrate)
        {
            m_rateIntegral = (m_rateIntegral + error * dt).GetClamp(-m_gains.m_rateIntegralLimit, m_gains.m_rateIntegralLimit);
        }
        else
        {
            m_rateIntegral = AZ::Vector3::CreateZero();
        }

        const AZ::Vector3 derivative =
            (m_hasPreviousRateError && dt > 0.0f) ? (error - m_previousRateError) / dt : AZ::Vector3::CreateZero();
        m_previousRateError = error;
        m_hasPreviousRateError = true;

        const AZ::Vector3 angularAcceleration = m_gains.m_rateP * error + m_gains.m_rateI * m_rateIntegral + m_gains.m_rateD * derivative;

        // Feed-forward the gyroscopic term so the loop sees decoupled axes.
        return m_inertia * angularAcceleration + bodyRate.Cross(m_inertia * bodyRate);
    }

    void MultirotorController::Step(const Command& command, const State& state, float dt)
    {
        AZStd::vector<float> desiredThrusts(m_allocation.size(), 0.0f);

        if (command.m_mode != Mode::None)
        {
            const AZ::Vector3 desiredRate = command.m_mode == Mode::Attitude
                ? AttitudeControl(command.m_attitude, state.m_orientation)
                : command.m_bodyRate;
            const bool integrate = command.m_throttle >= m_gains.m_integratorThrottleThreshold;
            const AZ::Vector3 torque = RateControl(desiredRate, state.m_bodyRate, integrate, dt);
            desiredThrusts = Mix(ThrottleToThrust(command.m_throttle), torque);
        }
        else
        {
            Reset();
        }

        // First-order motor lag on rotor speed, discretized exactly for the step.
        const float alpha = m_motor.m_timeConstant > 0.0f ? 1.0f - AZStd::exp(-dt / m_motor.m_timeConstant) : 1.0f;
        m_yawReactionTorque = 0.0f;
        for (size_t i = 0; i < m_allocation.size(); ++i)
        {
            const float commandedSpeed = AZStd::sqrt(desiredThrusts[i] / m_motor.m_thrustCoefficient);
            m_rotorSpeeds[i] += alpha * (commandedSpeed - m_rotorSpeeds[i]);
            const float speedSquared = m_rotorSpeeds[i] * m_rotorSpeeds[i];
            m_rotorThrusts[i] = m_motor.m_thrustCoefficient * speedSquared;
            m_yawReactionTorque += m_yawSign[i] * m_motor.m_torqueCoefficient * speedSquared;
        }
    }
} // namespace Multirotor
