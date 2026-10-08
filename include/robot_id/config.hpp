#pragma once

#include <array>
#include <cstdint>

namespace robot_id::config {

// Frozen MVP baseline. Changing any value in this file is a requirements change,
// not an implementation tweak. Keep docs/REQUIREMENTS.md in sync.
inline constexpr int kDof = 6;
inline constexpr int kParametersPerLink = 10;
inline constexpr int kFullParameterCount = kDof * kParametersPerLink;

inline constexpr double kTorqueNoiseStdNm = 0.20;
inline constexpr std::uint32_t kRandomSeed = 20260910U;
inline constexpr double kIdentificationSamplePeriodS = 0.01;
inline constexpr int kIdentificationPeriods = 4;
inline constexpr double kSvdRelativeRankThreshold = 1e-8;

inline constexpr double kValidationDurationS = 18.0;
inline constexpr double kValidationSamplePeriodS = 0.01;

inline constexpr double kControlDurationS = 15.0;
inline constexpr double kControlStepS = 0.001;
inline constexpr int kControlLogStride = 10;

inline constexpr double kCtcKp = 64.0;
inline constexpr double kCtcKd = 16.0;
inline constexpr double kPdKp = 95.0;
inline constexpr double kPdKd = 18.0;

inline constexpr std::array<double, kDof> kInitialPositionOffsetRad = {
    0.08, -0.06, 0.07, -0.05, 0.05, -0.04};

// Project-level trajectory envelope, intentionally tighter than the URDF limits.
inline constexpr double kTrajectoryMaxAbsPositionRad = 1.8;
inline constexpr double kTrajectoryMaxAbsVelocityRadS = 1.8;
inline constexpr double kTrajectoryMaxAbsAccelerationRadS2 = 2.5;

// Definition of Done and deterministic A0 sampling protocol.
inline constexpr int kA0StateSamples = 100;
inline constexpr std::uint32_t kA0RandomSeed = 42U;
inline constexpr double kA0MaxAbsPositionRad = 0.7;
inline constexpr double kA0MaxAbsVelocityRadS = 0.8;
inline constexpr double kA0MaxAbsAccelerationRadS2 = 1.5;
inline constexpr double kA0MaxRegressorRelativeError = 1e-10;
inline constexpr double kA1MaxTorqueNrmse = 0.05;
inline constexpr double kA2MaxTrackingRmseDeg = 1.0;

}  // namespace robot_id::config
