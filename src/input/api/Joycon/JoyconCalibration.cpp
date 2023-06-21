// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include <cstring>

#include "input/api/Joycon/JoyconCalibration.h"
#include "input/api/Joycon/JoyconTypes.h"

namespace InputCommon::Joycon {

CalibrationProtocol::CalibrationProtocol(std::shared_ptr<JoyconHandle> handle)
    : JoyconCommonProtocol(std::move(handle)) {}

DriverResult CalibrationProtocol::GetLeftJoyStickCalibration(JoyStickCalibration& calibration) {
    ScopedSetBlocking sb(this);
    DriverResult result{DriverResult::Success};
    JoystickLeftSpiCalibration spi_calibration{};
    bool has_user_calibration = false;
    calibration = {};

    if (result == DriverResult::Success) {
        result = HasUserCalibration(SpiAddress::USER_LEFT_MAGIC, has_user_calibration);
    }

    // Read User defined calibration
    if (result == DriverResult::Success && has_user_calibration) {
        result = ReadSPI(SpiAddress::USER_LEFT_DATA, spi_calibration);
    }

    // Read Factory calibration
    if (result == DriverResult::Success && !has_user_calibration) {
        result = ReadSPI(SpiAddress::FACT_LEFT_DATA, spi_calibration);
    }

    if (result == DriverResult::Success) {
        calibration.x.center = GetXAxisCalibrationValue(spi_calibration.center);
        calibration.y.center = GetYAxisCalibrationValue(spi_calibration.center);
        calibration.x.min = GetXAxisCalibrationValue(spi_calibration.min);
        calibration.y.min = GetYAxisCalibrationValue(spi_calibration.min);
        calibration.x.max = GetXAxisCalibrationValue(spi_calibration.max);
        calibration.y.max = GetYAxisCalibrationValue(spi_calibration.max);
    }

    // Set a valid default calibration if data is missing
    ValidateCalibration(calibration);

    return result;
}

DriverResult CalibrationProtocol::GetRightJoyStickCalibration(JoyStickCalibration& calibration) {
    ScopedSetBlocking sb(this);
    DriverResult result{DriverResult::Success};
    JoystickRightSpiCalibration spi_calibration{};
    bool has_user_calibration = false;
    calibration = {};

    if (result == DriverResult::Success) {
        result = HasUserCalibration(SpiAddress::USER_RIGHT_MAGIC, has_user_calibration);
    }

    // Read User defined calibration
    if (result == DriverResult::Success && has_user_calibration) {
        result = ReadSPI(SpiAddress::USER_RIGHT_DATA, spi_calibration);
    }

    // Read Factory calibration
    if (result == DriverResult::Success && !has_user_calibration) {
        result = ReadSPI(SpiAddress::FACT_RIGHT_DATA, spi_calibration);
    }

    if (result == DriverResult::Success) {
        calibration.x.center = GetXAxisCalibrationValue(spi_calibration.center);
        calibration.y.center = GetYAxisCalibrationValue(spi_calibration.center);
        calibration.x.min = GetXAxisCalibrationValue(spi_calibration.min);
        calibration.y.min = GetYAxisCalibrationValue(spi_calibration.min);
        calibration.x.max = GetXAxisCalibrationValue(spi_calibration.max);
        calibration.y.max = GetYAxisCalibrationValue(spi_calibration.max);
    }

    // Set a valid default calibration if data is missing
    ValidateCalibration(calibration);

    return result;
}

DriverResult CalibrationProtocol::GetImuCalibration(MotionCalibration& calibration) {
    ScopedSetBlocking sb(this);
    DriverResult result{DriverResult::Success};
    ImuSpiCalibration spi_calibration{};
    bool has_user_calibration = false;
    calibration = {};

    if (result == DriverResult::Success) {
        result = HasUserCalibration(SpiAddress::USER_IMU_MAGIC, has_user_calibration);
    }

    // Read User defined calibration
    if (result == DriverResult::Success && has_user_calibration) {
        result = ReadSPI(SpiAddress::USER_IMU_DATA, spi_calibration);
    }

    // Read Factory calibration
    if (result == DriverResult::Success && !has_user_calibration) {
        result = ReadSPI(SpiAddress::FACT_IMU_DATA, spi_calibration);
    }

    if (result == DriverResult::Success) {
        calibration.accelerometer[0].offset = spi_calibration.accelerometer_offset[0];
        calibration.accelerometer[1].offset = spi_calibration.accelerometer_offset[1];
        calibration.accelerometer[2].offset = spi_calibration.accelerometer_offset[2];

        calibration.accelerometer[0].scale = spi_calibration.accelerometer_scale[0];
        calibration.accelerometer[1].scale = spi_calibration.accelerometer_scale[1];
        calibration.accelerometer[2].scale = spi_calibration.accelerometer_scale[2];

        calibration.gyro[0].offset = spi_calibration.gyroscope_offset[0];
        calibration.gyro[1].offset = spi_calibration.gyroscope_offset[1];
        calibration.gyro[2].offset = spi_calibration.gyroscope_offset[2];

        calibration.gyro[0].scale = spi_calibration.gyroscope_scale[0];
        calibration.gyro[1].scale = spi_calibration.gyroscope_scale[1];
        calibration.gyro[2].scale = spi_calibration.gyroscope_scale[2];
    }

    ValidateCalibration(calibration);

    return result;
}

DriverResult CalibrationProtocol::HasUserCalibration(SpiAddress address,
                                                     bool& has_user_calibration) {
    MagicSpiCalibration spi_magic{};
    const DriverResult result{ReadSPI(address, spi_magic)};
    has_user_calibration = false;
    if (result == DriverResult::Success) {
        has_user_calibration = spi_magic.first == CalibrationMagic::USR_MAGIC_0 &&
                               spi_magic.second == CalibrationMagic::USR_MAGIC_1;
    }
    return result;
}

uint16 CalibrationProtocol::GetXAxisCalibrationValue(std::span<uint8> block) const {
    return static_cast<uint16>(((block[1] & 0x0F) << 8) | block[0]);
}

uint16 CalibrationProtocol::GetYAxisCalibrationValue(std::span<uint8> block) const {
    return static_cast<uint16>((block[2] << 4) | (block[1] >> 4));
}

void CalibrationProtocol::ValidateCalibration(JoyStickCalibration& calibration) {
    constexpr uint16 DefaultStickCenter{0x800};
    constexpr uint16 DefaultStickRange{0x6cc};

    calibration.x.center = ValidateValue(calibration.x.center, DefaultStickCenter);
    calibration.x.max = ValidateValue(calibration.x.max, DefaultStickRange);
    calibration.x.min = ValidateValue(calibration.x.min, DefaultStickRange);

    calibration.y.center = ValidateValue(calibration.y.center, DefaultStickCenter);
    calibration.y.max = ValidateValue(calibration.y.max, DefaultStickRange);
    calibration.y.min = ValidateValue(calibration.y.min, DefaultStickRange);
}

void CalibrationProtocol::ValidateCalibration(MotionCalibration& calibration) {
    constexpr sint16 DefaultAccelerometerScale{0x4000};
    constexpr sint16 DefaultGyroScale{0x3be7};
    constexpr sint16 DefaultOffset{0};

    for (auto& sensor : calibration.accelerometer) {
        sensor.scale = ValidateValue(sensor.scale, DefaultAccelerometerScale);
        sensor.offset = ValidateValue(sensor.offset, DefaultOffset);
    }
    for (auto& sensor : calibration.gyro) {
        sensor.scale = ValidateValue(sensor.scale, DefaultGyroScale);
        sensor.offset = ValidateValue(sensor.offset, DefaultOffset);
    }
}

uint16 CalibrationProtocol::ValidateValue(uint16 value, uint16 default_value) const {
    if (value == 0) {
        return default_value;
    }
    if (value == 0xFFF) {
        return default_value;
    }
    return value;
}

sint16 CalibrationProtocol::ValidateValue(sint16 value, sint16 default_value) const {
    if (value == 0) {
        return default_value;
    }
    if (value == 0xFFF) {
        return default_value;
    }
    return value;
}

} // namespace InputCommon::Joycon
