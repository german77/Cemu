// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

// Based on dkms-hid-nintendo implementation, CTCaer joycon toolkit and dekuNukem reverse
// engineering https://github.com/nicman23/dkms-hid-nintendo/blob/master/src/hid-nintendo.c
// https://github.com/CTCaer/jc_toolkit
// https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering

#pragma once

#include <functional>
#include <span>

#include "input/api/Joycon/JoyconTypes.h"

namespace InputCommon::Joycon {

// Handles input packages and triggers the corresponding input events
class JoyconPoller {
public:
    JoyconPoller(ControllerType device_type_, JoyStickCalibration left_stick_calibration_,
                 JoyStickCalibration right_stick_calibration_,
                 MotionCalibration motion_calibration_);

    /// Handles data from passive packages
    void ReadPassiveMode(JCState& state, std::span<uint8> buffer);

    /// Handles data from active packages
    void ReadActiveMode(JCState& state, std::span<uint8> buffer, const MotionStatus& motion_status);

    /// Handles data from nfc or ir packages
    void ReadNfcIRMode(JCState& state, std::span<uint8> buffer, const MotionStatus& motion_status);

private:
    void UpdateActiveLeftPadInput(JCState& state, const InputReportActive& input,
                                  const MotionStatus& motion_status);
    void UpdateActiveRightPadInput(JCState& state, const InputReportActive& input,
                                   const MotionStatus& motion_status);
    void UpdateActiveProPadInput(JCState& state, const InputReportActive& input, const MotionStatus& motion_status);

    void UpdatePassiveLeftPadInput(JCState& state, const InputReportPassive& buffer);
    void UpdatePassiveRightPadInput(JCState& state, const InputReportPassive& buffer);
    void UpdatePassiveProPadInput(JCState& state, const InputReportPassive& buffer);

    /// Returns a calibrated joystick axis from raw axis data
    float GetAxisValue(uint16 raw_value, JoyStickAxisCalibration calibration) const;

    /// Returns a digital joystick axis from passive axis data
    std::pair<float, float> GetPassiveAxisValue(PassivePadStick raw_value) const;

    /// Returns a calibrated accelerometer axis from raw motion data
    float GetAccelerometerValue(sint16 raw, const MotionSensorCalibration& cal,
                              AccelerometerSensitivity sensitivity) const;

    /// Returns a calibrated gyro axis from raw motion data
    float GetGyroValue(sint16 raw_value, const MotionSensorCalibration& cal,
                     GyroSensitivity sensitivity) const;

    /// Returns a raw motion value from a buffer
    sint16 GetRawIMUValues(size_t sensor, size_t axis, const InputReportActive& input) const;

    /// Returns motion data from a buffer
    MotionData GetMotionInput(const InputReportActive& input,
                              const MotionStatus& motion_status) const;

    ControllerType device_type{};

    // Device calibration
    JoyStickCalibration left_stick_calibration{};
    JoyStickCalibration right_stick_calibration{};
    MotionCalibration motion_calibration{};
};

} // namespace InputCommon::Joycon
