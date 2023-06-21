// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

// Based on dkms-hid-nintendo implementation, CTCaer joycon toolkit and dekuNukem reverse
// engineering https://github.com/nicman23/dkms-hid-nintendo/blob/master/src/hid-nintendo.c
// https://github.com/CTCaer/jc_toolkit
// https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering

#pragma once

#include <vector>

#include "input/api/Joycon/JoyconCommonProtocol.h"

namespace InputCommon::Joycon {
enum class DriverResult;
struct JoyStickCalibration;
struct IMUCalibration;
struct JoyconHandle;
} // namespace InputCommon::Joycon

namespace InputCommon::Joycon {

/// Driver functions related to retrieving calibration data from the device
class CalibrationProtocol final : private JoyconCommonProtocol {
public:
    explicit CalibrationProtocol(std::shared_ptr<JoyconHandle> handle);

    /**
     * Sends a request to obtain the left stick calibration from memory
     * @param is_factory_calibration if true factory values will be returned
     * @returns JoyStickCalibration of the left joystick
     */
    DriverResult GetLeftJoyStickCalibration(JoyStickCalibration& calibration);

    /**
     * Sends a request to obtain the right stick calibration from memory
     * @param is_factory_calibration if true factory values will be returned
     * @returns JoyStickCalibration of the right joystick
     */
    DriverResult GetRightJoyStickCalibration(JoyStickCalibration& calibration);

    /**
     * Sends a request to obtain the motion calibration from memory
     * @returns ImuCalibration of the motion sensor
     */
    DriverResult GetImuCalibration(MotionCalibration& calibration);

private:
    /// Returns true if the specified address corresponds to the magic value of user calibration
    DriverResult HasUserCalibration(SpiAddress address, bool& has_user_calibration);

    /// Converts a raw calibration block to an uint16 value containing the x axis value
    uint16 GetXAxisCalibrationValue(std::span<uint8> block) const;

    /// Converts a raw calibration block to an uint16 value containing the y axis value
    uint16 GetYAxisCalibrationValue(std::span<uint8> block) const;

    /// Ensures that all joystick calibration values are set
    void ValidateCalibration(JoyStickCalibration& calibration);

    /// Ensures that all motion calibration values are set
    void ValidateCalibration(MotionCalibration& calibration);

    /// Returns the default value if the value is either zero or 0xFFF
    uint16 ValidateValue(uint16 value, uint16 default_value) const;

    /// Returns the default value if the value is either zero or 0xFFF
    sint16 ValidateValue(sint16 value, sint16 default_value) const;

    sint16 ring_data_max = 0;
    sint16 ring_data_default = 0;
    sint16 ring_data_min = 0;
};

} // namespace InputCommon::Joycon
