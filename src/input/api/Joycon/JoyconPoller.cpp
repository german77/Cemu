// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "input/api/Joycon/JoyconPoller.h"

namespace InputCommon::Joycon {

JoyconPoller::JoyconPoller(ControllerType device_type_, JoyStickCalibration left_stick_calibration_,
                           JoyStickCalibration right_stick_calibration_,
                           MotionCalibration motion_calibration_)
    : device_type{device_type_}, left_stick_calibration{left_stick_calibration_},
      right_stick_calibration{right_stick_calibration_}, motion_calibration{motion_calibration_} {}

void JoyconPoller::SetCallbacks(const JoyconCallbacks& callbacks_) {
    callbacks = std::move(callbacks_);
}

void JoyconPoller::ReadActiveMode(std::span<uint8> buffer, const MotionStatus& motion_status) {
    InputReportActive data{};
    memcpy(&data, buffer.data(), sizeof(InputReportActive));

    switch (device_type) {
    case ControllerType::Left:
        UpdateActiveLeftPadInput(data, motion_status);
        break;
    case ControllerType::Right:
        UpdateActiveRightPadInput(data, motion_status);
        break;
    case ControllerType::Pro:
        UpdateActiveProPadInput(data, motion_status);
        break;
    default:
        break;
    }
}

void JoyconPoller::ReadPassiveMode(std::span<uint8> buffer) {
    InputReportPassive data{};
    memcpy(&data, buffer.data(), sizeof(InputReportPassive));

    switch (device_type) {
    case ControllerType::Left:
        UpdatePassiveLeftPadInput(data);
        break;
    case ControllerType::Right:
        UpdatePassiveRightPadInput(data);
        break;
    case ControllerType::Pro:
        UpdatePassiveProPadInput(data);
        break;
    default:
        break;
    }
}

void JoyconPoller::ReadNfcIRMode(std::span<uint8> buffer, const MotionStatus& motion_status) {
    // This mode is compatible with the active mode
    ReadActiveMode(buffer, motion_status);
}

void JoyconPoller::UpdateAmiibo(const Joycon::TagInfo& tag_info) {
    callbacks.on_amiibo_data(tag_info);
}

void JoyconPoller::UpdateActiveLeftPadInput(const InputReportActive& input,
                                            const MotionStatus& motion_status) {
    static constexpr std::array<Joycon::PadButton, 11> left_buttons{
        Joycon::PadButton::Down,    Joycon::PadButton::Up,     Joycon::PadButton::Right,
        Joycon::PadButton::Left,    Joycon::PadButton::LeftSL, Joycon::PadButton::LeftSR,
        Joycon::PadButton::L,       Joycon::PadButton::ZL,     Joycon::PadButton::Minus,
        Joycon::PadButton::Capture, Joycon::PadButton::StickL,
    };

    const uint32 raw_button =
        static_cast<uint32>(input.button_input[2] | ((input.button_input[1] & 0b00101001) << 16));
    for (std::size_t i = 0; i < left_buttons.size(); ++i) {
        const bool button_status = (raw_button & static_cast<uint32>(left_buttons[i])) != 0;
        const int button = static_cast<int>(left_buttons[i]);
        callbacks.on_button_data(button, button_status);
    }

    const uint16 raw_left_axis_x =
        static_cast<uint16>(input.left_stick_state[0] | ((input.left_stick_state[1] & 0xf) << 8));
    const uint16 raw_left_axis_y =
        static_cast<uint16>((input.left_stick_state[1] >> 4) | (input.left_stick_state[2] << 4));
    const float left_axis_x = GetAxisValue(raw_left_axis_x, left_stick_calibration.x);
    const float left_axis_y = GetAxisValue(raw_left_axis_y, left_stick_calibration.y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickX), left_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickY), left_axis_y);

    if (motion_status.is_enabled) {
        auto left_motion = GetMotionInput(input, motion_status);
        // Rotate motion axis to the correct direction
        left_motion.accel_y = -left_motion.accel_y;
        left_motion.accel_z = -left_motion.accel_z;
        left_motion.gyro_x = -left_motion.gyro_x;
        callbacks.on_motion_data(static_cast<int>(PadMotion::LeftMotion), left_motion);
    }
}

void JoyconPoller::UpdateActiveRightPadInput(const InputReportActive& input,
                                             const MotionStatus& motion_status) {
    static constexpr std::array<Joycon::PadButton, 11> right_buttons{
        Joycon::PadButton::Y,    Joycon::PadButton::X,       Joycon::PadButton::B,
        Joycon::PadButton::A,    Joycon::PadButton::RightSL, Joycon::PadButton::RightSR,
        Joycon::PadButton::R,    Joycon::PadButton::ZR,      Joycon::PadButton::Plus,
        Joycon::PadButton::Home, Joycon::PadButton::StickR,
    };

    const uint32 raw_button =
        static_cast<uint32>((input.button_input[0] << 8) | (input.button_input[1] << 16));
    for (std::size_t i = 0; i < right_buttons.size(); ++i) {
        const bool button_status = (raw_button & static_cast<uint32>(right_buttons[i])) != 0;
        const int button = static_cast<int>(right_buttons[i]);
        callbacks.on_button_data(button, button_status);
    }

    const uint16 raw_right_axis_x =
        static_cast<uint16>(input.right_stick_state[0] | ((input.right_stick_state[1] & 0xf) << 8));
    const uint16 raw_right_axis_y =
        static_cast<uint16>((input.right_stick_state[1] >> 4) | (input.right_stick_state[2] << 4));
    const float right_axis_x = GetAxisValue(raw_right_axis_x, right_stick_calibration.x);
    const float right_axis_y = GetAxisValue(raw_right_axis_y, right_stick_calibration.y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickX), right_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickY), right_axis_y);

    if (motion_status.is_enabled) {
        auto right_motion = GetMotionInput(input, motion_status);
        // Rotate motion axis to the correct direction
        right_motion.accel_x = -right_motion.accel_x;
        right_motion.accel_y = -right_motion.accel_y;
        right_motion.gyro_z = -right_motion.gyro_z;
        callbacks.on_motion_data(static_cast<int>(PadMotion::RightMotion), right_motion);
    }
}

void JoyconPoller::UpdateActiveProPadInput(const InputReportActive& input,
                                           const MotionStatus& motion_status) {
    static constexpr std::array<Joycon::PadButton, 18> pro_buttons{
        Joycon::PadButton::Down,  Joycon::PadButton::Up,      Joycon::PadButton::Right,
        Joycon::PadButton::Left,  Joycon::PadButton::L,       Joycon::PadButton::ZL,
        Joycon::PadButton::Minus, Joycon::PadButton::Capture, Joycon::PadButton::Y,
        Joycon::PadButton::X,     Joycon::PadButton::B,       Joycon::PadButton::A,
        Joycon::PadButton::R,     Joycon::PadButton::ZR,      Joycon::PadButton::Plus,
        Joycon::PadButton::Home,  Joycon::PadButton::StickL,  Joycon::PadButton::StickR,
    };

    const uint32 raw_button = static_cast<uint32>(input.button_input[2] | (input.button_input[0] << 8) |
                                            (input.button_input[1] << 16));
    for (std::size_t i = 0; i < pro_buttons.size(); ++i) {
        const bool button_status = (raw_button & static_cast<uint32>(pro_buttons[i])) != 0;
        const int button = static_cast<int>(pro_buttons[i]);
        callbacks.on_button_data(button, button_status);
    }

    const uint16 raw_left_axis_x =
        static_cast<uint16>(input.left_stick_state[0] | ((input.left_stick_state[1] & 0xf) << 8));
    const uint16 raw_left_axis_y =
        static_cast<uint16>((input.left_stick_state[1] >> 4) | (input.left_stick_state[2] << 4));
    const uint16 raw_right_axis_x =
        static_cast<uint16>(input.right_stick_state[0] | ((input.right_stick_state[1] & 0xf) << 8));
    const uint16 raw_right_axis_y =
        static_cast<uint16>((input.right_stick_state[1] >> 4) | (input.right_stick_state[2] << 4));

    const float left_axis_x = GetAxisValue(raw_left_axis_x, left_stick_calibration.x);
    const float left_axis_y = GetAxisValue(raw_left_axis_y, left_stick_calibration.y);
    const float right_axis_x = GetAxisValue(raw_right_axis_x, right_stick_calibration.x);
    const float right_axis_y = GetAxisValue(raw_right_axis_y, right_stick_calibration.y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickX), left_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickY), left_axis_y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickX), right_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickY), right_axis_y);

    if (motion_status.is_enabled) {
        auto pro_motion = GetMotionInput(input, motion_status);
        pro_motion.gyro_x = -pro_motion.gyro_x;
        pro_motion.accel_y = -pro_motion.accel_y;
        pro_motion.accel_z = -pro_motion.accel_z;
        callbacks.on_motion_data(static_cast<int>(PadMotion::LeftMotion), pro_motion);
        callbacks.on_motion_data(static_cast<int>(PadMotion::RightMotion), pro_motion);
    }
}

void JoyconPoller::UpdatePassiveLeftPadInput(const InputReportPassive& input) {
    static constexpr std::array<PassivePadButton, 11> left_buttons{
        PassivePadButton::Down_A,  PassivePadButton::Right_X, PassivePadButton::Left_B,
        PassivePadButton::Up_Y,    PassivePadButton::SL,      PassivePadButton::SR,
        PassivePadButton::L_R,     PassivePadButton::ZL_ZR,   PassivePadButton::Minus,
        PassivePadButton::Capture, PassivePadButton::StickL,
    };

    for (auto left_button : left_buttons) {
        const bool button_status = (input.button_input & static_cast<uint32>(left_button)) != 0;
        const int button = static_cast<int>(left_button);
        callbacks.on_button_data(button, button_status);
    }

    const auto [left_axis_x, left_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state));
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickX), left_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickY), left_axis_y);
}

void JoyconPoller::UpdatePassiveRightPadInput(const InputReportPassive& input) {
    static constexpr std::array<PassivePadButton, 11> right_buttons{
        PassivePadButton::Down_A, PassivePadButton::Right_X, PassivePadButton::Left_B,
        PassivePadButton::Up_Y,   PassivePadButton::SL,      PassivePadButton::SR,
        PassivePadButton::L_R,    PassivePadButton::ZL_ZR,   PassivePadButton::Plus,
        PassivePadButton::Home,   PassivePadButton::StickR,
    };

    for (auto right_button : right_buttons) {
        const bool button_status = (input.button_input & static_cast<uint32>(right_button)) != 0;
        const int button = static_cast<int>(right_button);
        callbacks.on_button_data(button, button_status);
    }

    const auto [right_axis_x, right_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state));
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickX), right_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickY), right_axis_y);
}

void JoyconPoller::UpdatePassiveProPadInput(const InputReportPassive& input) {
    static constexpr std::array<PassivePadButton, 14> pro_buttons{
        PassivePadButton::Down_A, PassivePadButton::Right_X, PassivePadButton::Left_B,
        PassivePadButton::Up_Y,   PassivePadButton::SL,      PassivePadButton::SR,
        PassivePadButton::L_R,    PassivePadButton::ZL_ZR,   PassivePadButton::Minus,
        PassivePadButton::Plus,   PassivePadButton::Capture, PassivePadButton::Home,
        PassivePadButton::StickL, PassivePadButton::StickR,
    };

    for (auto pro_button : pro_buttons) {
        const bool button_status = (input.button_input & static_cast<uint32>(pro_button)) != 0;
        const int button = static_cast<int>(pro_button);
        callbacks.on_button_data(button, button_status);
    }

    const auto [left_axis_x, left_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state & 0xf));
    const auto [right_axis_x, right_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state >> 4));
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickX), left_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickY), left_axis_y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickX), right_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickY), right_axis_y);
}

float JoyconPoller::GetAxisValue(uint16 raw_value, Joycon::JoyStickAxisCalibration calibration) const {
    const float value = static_cast<float>(raw_value - calibration.center);
    if (value > 0.0f) {
        return value / calibration.max;
    }
    return value / calibration.min;
}

std::pair<float, float> JoyconPoller::GetPassiveAxisValue(PassivePadStick raw_value) const {
    switch (raw_value) {
    case PassivePadStick::Right:
        return {1.0f, 0.0f};
    case PassivePadStick::RightDown:
        return {1.0f, -1.0f};
    case PassivePadStick::Down:
        return {0.0f, -1.0f};
    case PassivePadStick::DownLeft:
        return {-1.0f, -1.0f};
    case PassivePadStick::Left:
        return {-1.0f, 0.0f};
    case PassivePadStick::LeftUp:
        return {-1.0f, 1.0f};
    case PassivePadStick::Up:
        return {0.0f, 1.0f};
    case PassivePadStick::UpRight:
        return {1.0f, 1.0f};
    case PassivePadStick::Neutral:
    default:
        return {0.0f, 0.0f};
    }
}

float JoyconPoller::GetAccelerometerValue(sint16 raw, const MotionSensorCalibration& cal,
                                        AccelerometerSensitivity sensitivity) const {
    const float value = raw * (1.0f / (cal.scale - cal.offset)) * 4;
    switch (sensitivity) {
    case Joycon::AccelerometerSensitivity::G2:
        return value / 4.0f;
    case Joycon::AccelerometerSensitivity::G4:
        return value / 2.0f;
    case Joycon::AccelerometerSensitivity::G8:
        return value;
    case Joycon::AccelerometerSensitivity::G16:
        return value * 2.0f;
    }
    return value;
}

float JoyconPoller::GetGyroValue(sint16 raw, const MotionSensorCalibration& cal,
                               GyroSensitivity sensitivity) const {
    const float value = (raw - cal.offset) * (936.0f / (cal.scale - cal.offset)) / 360.0f;
    switch (sensitivity) {
    case Joycon::GyroSensitivity::DPS250:
        return value / 8.0f;
    case Joycon::GyroSensitivity::DPS500:
        return value / 4.0f;
    case Joycon::GyroSensitivity::DPS1000:
        return value / 2.0f;
    case Joycon::GyroSensitivity::DPS2000:
        return value;
    }
    return value;
}

sint16 JoyconPoller::GetRawIMUValues(std::size_t sensor, size_t axis,
                                  const InputReportActive& input) const {
    return input.motion_input[(sensor * 3) + axis];
}

MotionData JoyconPoller::GetMotionInput(const InputReportActive& input,
                                        const MotionStatus& motion_status) const {
    MotionData motion{};
    const auto& accel_cal = motion_calibration.accelerometer;
    const auto& gyro_cal = motion_calibration.gyro;
    const sint16 raw_accel_x = input.motion_input[1];
    const sint16 raw_accel_y = input.motion_input[0];
    const sint16 raw_accel_z = input.motion_input[2];
    const sint16 raw_gyro_x = input.motion_input[4];
    const sint16 raw_gyro_y = input.motion_input[3];
    const sint16 raw_gyro_z = input.motion_input[5];

    motion.delta_timestamp = motion_status.delta_time;
    motion.accel_x =
        GetAccelerometerValue(raw_accel_x, accel_cal[1], motion_status.accelerometer_sensitivity);
    motion.accel_y =
        GetAccelerometerValue(raw_accel_y, accel_cal[0], motion_status.accelerometer_sensitivity);
    motion.accel_z =
        GetAccelerometerValue(raw_accel_z, accel_cal[2], motion_status.accelerometer_sensitivity);
    motion.gyro_x = GetGyroValue(raw_gyro_x, gyro_cal[1], motion_status.gyro_sensitivity);
    motion.gyro_y = GetGyroValue(raw_gyro_y, gyro_cal[0], motion_status.gyro_sensitivity);
    motion.gyro_z = GetGyroValue(raw_gyro_z, gyro_cal[2], motion_status.gyro_sensitivity);

    // TODO(German77): Return all three samples data
    return motion;
}

} // namespace InputCommon::Joycon
