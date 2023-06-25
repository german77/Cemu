// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "input/api/Joycon/JoyconPoller.h"

namespace InputCommon::Joycon {

JoyconPoller::JoyconPoller(ControllerType device_type_, JoyStickCalibration left_stick_calibration_,
                           JoyStickCalibration right_stick_calibration_,
                           MotionCalibration motion_calibration_)
    : device_type{device_type_}, left_stick_calibration{left_stick_calibration_},
      right_stick_calibration{right_stick_calibration_}, motion_calibration{motion_calibration_} {}


void JoyconPoller::ReadActiveMode(JCState& state, std::span<uint8> buffer, const MotionStatus& motion_status) {
    InputReportActive data{};
    memcpy(&data, buffer.data(), sizeof(InputReportActive));

    switch (device_type) {
    case ControllerType::Left:
        UpdateActiveLeftPadInput(state, data, motion_status);
        break;
    case ControllerType::Right:
        UpdateActiveRightPadInput(state, data, motion_status);
        break;
    case ControllerType::Pro:
        UpdateActiveProPadInput(state, data, motion_status);
        break;
    default:
        break;
    }
}

void JoyconPoller::ReadPassiveMode(JCState& state, std::span<uint8> buffer) {
    InputReportPassive data{};
    memcpy(&data, buffer.data(), sizeof(InputReportPassive));

    switch (device_type) {
    case ControllerType::Left:
        UpdatePassiveLeftPadInput(state, data);
        break;
    case ControllerType::Right:
        UpdatePassiveRightPadInput(state, data);
        break;
    case ControllerType::Pro:
        UpdatePassiveProPadInput(state, data);
        break;
    default:
        break;
    }
}

void JoyconPoller::ReadNfcIRMode(JCState& state, std::span<uint8> buffer, const MotionStatus& motion_status) {
    // This mode is compatible with the active mode
    ReadActiveMode(state, buffer, motion_status);
}

void JoyconPoller::UpdateActiveLeftPadInput(JCState& state, const InputReportActive& input,
                                            const MotionStatus& motion_status) {
    static constexpr std::array<Joycon::PadButton, 11> left_buttons{
        Joycon::PadButton::Down,    Joycon::PadButton::Up,     Joycon::PadButton::Right,
        Joycon::PadButton::Left,    Joycon::PadButton::LeftSL, Joycon::PadButton::LeftSR,
        Joycon::PadButton::L,       Joycon::PadButton::ZL,     Joycon::PadButton::Minus,
        Joycon::PadButton::Capture, Joycon::PadButton::StickL,
    };

    const uint32 raw_button =
        static_cast<uint32>(input.button_input[2] | ((input.button_input[1] & 0b00101001) << 16));
    state.button = raw_button;

    const uint16 raw_left_axis_x =
        static_cast<uint16>(input.left_stick_state[0] | ((input.left_stick_state[1] & 0xf) << 8));
    const uint16 raw_left_axis_y =
        static_cast<uint16>((input.left_stick_state[1] >> 4) | (input.left_stick_state[2] << 4));
    const float left_axis_x = GetAxisValue(raw_left_axis_x, left_stick_calibration.x);
    const float left_axis_y = GetAxisValue(raw_left_axis_y, left_stick_calibration.y);
    state.lstick_x = left_axis_x;
    state.lstick_y = left_axis_y;

    if (motion_status.is_enabled) {
        auto left_motion = GetMotionInput(input, motion_status);
        // Rotate motion axis to the correct direction
        left_motion.accel_y = -left_motion.accel_y;
        left_motion.accel_z = -left_motion.accel_z;
        left_motion.gyro_x = -left_motion.gyro_x;
        //callbacks.on_motion_data(static_cast<int>(PadMotion::LeftMotion), left_motion);
    }
    state.valid = true;
}

void JoyconPoller::UpdateActiveRightPadInput(JCState& state, const InputReportActive& input,
                                             const MotionStatus& motion_status) {
    static constexpr std::array<Joycon::PadButton, 11> right_buttons{
        Joycon::PadButton::Y,    Joycon::PadButton::X,       Joycon::PadButton::B,
        Joycon::PadButton::A,    Joycon::PadButton::RightSL, Joycon::PadButton::RightSR,
        Joycon::PadButton::R,    Joycon::PadButton::ZR,      Joycon::PadButton::Plus,
        Joycon::PadButton::Home, Joycon::PadButton::StickR,
    };

    const uint32 raw_button =
        static_cast<uint32>((input.button_input[0] << 8) | (input.button_input[1] << 16));
    state.button = raw_button;

    const uint16 raw_right_axis_x =
        static_cast<uint16>(input.right_stick_state[0] | ((input.right_stick_state[1] & 0xf) << 8));
    const uint16 raw_right_axis_y =
        static_cast<uint16>((input.right_stick_state[1] >> 4) | (input.right_stick_state[2] << 4));
    const float right_axis_x = GetAxisValue(raw_right_axis_x, right_stick_calibration.x);
    const float right_axis_y = GetAxisValue(raw_right_axis_y, right_stick_calibration.y);
    state.rstick_x = right_axis_x;
    state.rstick_y = right_axis_y;

    if (motion_status.is_enabled) {
        auto right_motion = GetMotionInput(input, motion_status);
        // Rotate motion axis to the correct direction
        right_motion.accel_x = -right_motion.accel_x;
        right_motion.accel_y = -right_motion.accel_y;
        right_motion.gyro_z = -right_motion.gyro_z;
        //callbacks.on_motion_data(static_cast<int>(PadMotion::RightMotion), right_motion);
    }
    state.valid = true;
}

void JoyconPoller::UpdateActiveProPadInput(JCState& state, const InputReportActive& input,
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
    state.button = raw_button;

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
    state.lstick_x = left_axis_x;
    state.lstick_y = left_axis_y;
    state.rstick_x = right_axis_x;
    state.rstick_y = right_axis_y;

    if (motion_status.is_enabled) {
        auto pro_motion = GetMotionInput(input, motion_status);
        pro_motion.gyro_x = -pro_motion.gyro_x;
        pro_motion.accel_y = -pro_motion.accel_y;
        pro_motion.accel_z = -pro_motion.accel_z;
        //callbacks.on_motion_data(static_cast<int>(PadMotion::LeftMotion), pro_motion);
        //callbacks.on_motion_data(static_cast<int>(PadMotion::RightMotion), pro_motion);
    }
    state.valid = true;
}

void JoyconPoller::UpdatePassiveLeftPadInput(JCState& state, const InputReportPassive& input) {
    state.button = input.button_input;

    const auto [left_axis_x, left_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state));
    state.lstick_x = left_axis_x;
    state.lstick_y = left_axis_y;
    state.valid = true;
}

void JoyconPoller::UpdatePassiveRightPadInput(JCState& state, const InputReportPassive& input) {
    state.button = input.button_input;

    const auto [right_axis_x, right_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state));
    state.rstick_x = right_axis_x;
    state.rstick_y = right_axis_y;
    state.valid = true;
}

void JoyconPoller::UpdatePassiveProPadInput(JCState& state, const InputReportPassive& input) {
    state.button = input.button_input;

    const auto [left_axis_x, left_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state & 0xf));
    const auto [right_axis_x, right_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state >> 4));
    state.lstick_x = left_axis_x;
    state.lstick_y = left_axis_y;
    state.rstick_x = right_axis_x;
    state.rstick_y = right_axis_y;
    state.valid = true;
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
