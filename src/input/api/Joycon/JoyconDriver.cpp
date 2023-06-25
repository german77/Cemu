// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "input/api/Joycon/JoyconDriver.h"
#include "input/api/Joycon/JoyconCalibration.h"
#include "input/api/Joycon/JoyconGenericFunctions.h"
#include "input/api/Joycon/JoyconPoller.h"
#include "input/api/Joycon/JoyconRumble.h"
#pragma optimize("", off)
namespace InputCommon::Joycon {
JoyconDriver::JoyconDriver(std::size_t port_) : port{port_} {
    hidapi_handle = std::make_shared<JoyconHandle>();
}

JoyconDriver::~JoyconDriver() {
    Stop();
}

void JoyconDriver::Stop() {
    is_connected = false;
    input_thread = {};
}

DriverResult JoyconDriver::RequestDeviceAccess(SDL_hid_device_info* device_info) {
    std::scoped_lock lock{mutex};

    handle_device_type = ControllerType::None;
    GetDeviceType(device_info, handle_device_type);
    if (handle_device_type == ControllerType::None) {
        return DriverResult::UnsupportedControllerType;
    }

    hidapi_handle->handle =
        SDL_hid_open(device_info->vendor_id, device_info->product_id, device_info->serial_number);
    std::memcpy(&handle_serial_number, device_info->serial_number, 15);
    if (!hidapi_handle->handle) {
        return DriverResult::HandleInUse;
    }
    SDL_hid_set_nonblocking(hidapi_handle->handle, 1);
    return DriverResult::Success;
}

DriverResult JoyconDriver::InitializeDevice() {
    if (!hidapi_handle->handle) {
        return DriverResult::InvalidHandle;
    }
    std::scoped_lock lock{mutex};
    disable_input_thread = true;

    // Reset Counters
    error_counter = 0;
    hidapi_handle->packet_counter = 0;

    // Set HW default configuration
    vibration_enabled = true;
    motion_enabled = true;
    passive_enabled = false;
    gyro_sensitivity = Joycon::GyroSensitivity::DPS2000;
    gyro_performance = Joycon::GyroPerformance::HZ833;
    accelerometer_sensitivity = Joycon::AccelerometerSensitivity::G8;
    accelerometer_performance = Joycon::AccelerometerPerformance::HZ100;

    // Initialize HW Protocols
    calibration_protocol = std::make_unique<CalibrationProtocol>(hidapi_handle);
    generic_protocol = std::make_unique<GenericProtocol>(hidapi_handle);
    rumble_protocol = std::make_unique<RumbleProtocol>(hidapi_handle);

    // Get fixed joycon info
    generic_protocol->GetVersionNumber(version);
    generic_protocol->SetLowPowerMode(false);
    if (handle_device_type == ControllerType::Pro) {
        // Some 3rd party controllers aren't pro controllers
        generic_protocol->GetControllerType(device_type);
    } else {
        device_type = handle_device_type;
    }
    generic_protocol->GetSerialNumber(serial_number);
    supported_features = GetSupportedFeatures();

    // Get Calibration data
    calibration_protocol->GetLeftJoyStickCalibration(left_stick_calibration);
    calibration_protocol->GetRightJoyStickCalibration(right_stick_calibration);
    calibration_protocol->GetImuCalibration(motion_calibration);

    // Set led status
    generic_protocol->SetLedBlinkPattern(static_cast<uint8>(1 + port));

    // Apply HW configuration
    SetPollingMode();

    // Initialize joycon poller
    joycon_poller = std::make_unique<JoyconPoller>(device_type, left_stick_calibration,
                                                   right_stick_calibration, motion_calibration);

    // Start polling for data
    is_connected = true;
    if (!input_thread_running) {
        input_thread =
            std::jthread([this](std::stop_token stop_token) { InputThread(stop_token); });
    }

    disable_input_thread = false;
    return DriverResult::Success;
}

void JoyconDriver::InputThread(std::stop_token stop_token) {
    input_thread_running = true;

    // Max update rate is 5ms, ensure we are always able to read a bit faster
    constexpr int ThreadDelay = 2;
    std::vector<uint8> buffer(MaxBufferSize);

    while (!stop_token.stop_requested()) {
        int status = 0;

        if (!IsInputThreadValid()) {
            input_thread.request_stop();
            continue;
        }

        // By disabling the input thread we can ensure custom commands will succeed as no package is
        // skipped
        if (!disable_input_thread) {
            status = SDL_hid_read_timeout(hidapi_handle->handle, buffer.data(), buffer.size(),
                                          ThreadDelay);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(ThreadDelay));
        }

        if (IsPayloadCorrect(status, buffer)) {
            OnNewData(buffer);
        }

        std::this_thread::yield();
    }

    is_connected = false;
    input_thread_running = false;
}

void JoyconDriver::OnNewData(std::span<uint8> buffer) {
    const auto report_mode = static_cast<ReportMode>(buffer[0]);

    // Packages can be a little bit inconsistent. Average the delta time to provide a smoother
    // motion experience
    switch (report_mode) {
    case ReportMode::STANDARD_FULL_60HZ:
    case ReportMode::NFC_IR_MODE_60HZ:
    case ReportMode::SIMPLE_HID_MODE: {
        const auto now = std::chrono::steady_clock::now();
        const auto new_delta_time = static_cast<uint64>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - last_update).count());
        delta_time = ((delta_time * 8) + (new_delta_time * 2)) / 10;
        last_update = now;
        break;
    }
    default:
        break;
    }

    const MotionStatus motion_status{
        .is_enabled = motion_enabled,
        .delta_time = delta_time,
        .gyro_sensitivity = gyro_sensitivity,
        .accelerometer_sensitivity = accelerometer_sensitivity,
    };

    state.valid = false;
    switch (report_mode) {
    case ReportMode::STANDARD_FULL_60HZ:
        joycon_poller->ReadActiveMode(state, buffer, motion_status);
        break;
    case ReportMode::NFC_IR_MODE_60HZ:
        joycon_poller->ReadNfcIRMode(state, buffer, motion_status);
        break;
    case ReportMode::SIMPLE_HID_MODE:
        joycon_poller->ReadPassiveMode(state, buffer);
        break;
    default:
        break;
    }
}

DriverResult JoyconDriver::SetPollingMode() {
    disable_input_thread = true;

    rumble_protocol->EnableRumble(vibration_enabled && supported_features.vibration);

    if (motion_enabled && supported_features.motion) {
        generic_protocol->EnableImu(true);
        generic_protocol->SetImuConfig(gyro_sensitivity, gyro_performance,
                                       accelerometer_sensitivity, accelerometer_performance);
    } else {
        generic_protocol->EnableImu(false);
    }

    if (passive_enabled && supported_features.passive) {
        const auto result = generic_protocol->EnablePassiveMode();
        if (result == DriverResult::Success) {
            disable_input_thread = false;
            return result;
        }
    }

    // Default Mode
    const auto result = generic_protocol->EnableActiveMode();
    if (result != DriverResult::Success) {
    }
    // Switch calls this function after enabling active mode
    generic_protocol->TriggersElapsed();

    disable_input_thread = false;
    return result;
}

JoyconDriver::SupportedFeatures JoyconDriver::GetSupportedFeatures() {
    SupportedFeatures features{
        .passive = true,
        .motion = true,
        .vibration = true,
    };

    if (device_type == ControllerType::Right) {
        features.nfc = true;
    }

    if (device_type == ControllerType::Pro) {
        features.nfc = true;
    }
    return features;
}

bool JoyconDriver::IsInputThreadValid() const {
    if (!is_connected.load()) {
        return false;
    }
    if (hidapi_handle->handle == nullptr) {
        return false;
    }
    // Controller is not responding. Terminate connection
    if (error_counter > MaxErrorCount) {
        return false;
    }
    return true;
}

bool JoyconDriver::IsPayloadCorrect(int status, std::span<const uint8> buffer) {
    if (status <= -1) {
        error_counter++;
        return false;
    }
    // There's no new data
    if (status == 0) {
        return false;
    }
    // No reply ever starts with zero
    if (buffer[0] == 0x00) {
        error_counter++;
        return false;
    }
    error_counter = 0;
    return true;
}

DriverResult JoyconDriver::SetVibration(const VibrationValue& vibration) {
    std::scoped_lock lock{mutex};
    if (disable_input_thread) {
        return DriverResult::HandleInUse;
    }
    return rumble_protocol->SendVibration(vibration);
}

DriverResult JoyconDriver::SetLedConfig(uint8 led_pattern) {
    std::scoped_lock lock{mutex};
    if (disable_input_thread) {
        return DriverResult::HandleInUse;
    }
    return generic_protocol->SetLedPattern(led_pattern);
}

DriverResult JoyconDriver::SetPassiveMode() {
    std::scoped_lock lock{mutex};
    motion_enabled = false;
    passive_enabled = true;
    return SetPollingMode();
}

DriverResult JoyconDriver::SetActiveMode() {
    std::scoped_lock lock{mutex};
    motion_enabled = true;
    passive_enabled = false;
    return SetPollingMode();
}

JCState JoyconDriver::GetState() {
    return state;
}

bool JoyconDriver::IsConnected() const {
    std::scoped_lock lock{mutex};
    return is_connected.load();
}

bool JoyconDriver::IsMotionEnabled() const {
    std::scoped_lock lock{mutex};
    return motion_enabled;
}

bool JoyconDriver::IsVibrationEnabled() const {
    std::scoped_lock lock{mutex};
    return vibration_enabled;
}

FirmwareVersion JoyconDriver::GetDeviceVersion() const {
    std::scoped_lock lock{mutex};
    return version;
}

std::size_t JoyconDriver::GetDevicePort() const {
    std::scoped_lock lock{mutex};
    return port;
}

ControllerType JoyconDriver::GetDeviceType() const {
    std::scoped_lock lock{mutex};
    return device_type;
}

ControllerType JoyconDriver::GetHandleDeviceType() const {
    std::scoped_lock lock{mutex};
    return handle_device_type;
}

SerialNumber JoyconDriver::GetSerialNumber() const {
    std::scoped_lock lock{mutex};
    return serial_number;
}

SerialNumber JoyconDriver::GetHandleSerialNumber() const {
    std::scoped_lock lock{mutex};
    return handle_serial_number;
}

DriverResult JoyconDriver::GetDeviceType(SDL_hid_device_info* device_info,
                                         ControllerType& controller_type) {
    static constexpr std::array<std::pair<uint32, ControllerType>, 6> supported_devices{
        std::pair<uint32, ControllerType>{0x2006, ControllerType::Left},
        {0x2007, ControllerType::Right},
        {0x2009, ControllerType::Pro},
    };
    constexpr uint16 nintendo_vendor_id = 0x057e;

    controller_type = ControllerType::None;
    if (device_info->vendor_id != nintendo_vendor_id) {
        return DriverResult::UnsupportedControllerType;
    }

    for (const auto& [product_id, type] : supported_devices) {
        if (device_info->product_id == static_cast<uint16>(product_id)) {
            controller_type = type;
            return Joycon::DriverResult::Success;
        }
    }
    return Joycon::DriverResult::UnsupportedControllerType;
}

DriverResult JoyconDriver::GetSerialNumber(SDL_hid_device_info* device_info,
                                           SerialNumber& serial_number) {
    if (device_info->serial_number == nullptr) {
        return DriverResult::Unknown;
    }
    std::memcpy(&serial_number, device_info->serial_number, 15);
    return Joycon::DriverResult::Success;
}

} // namespace InputCommon::Joycon
#pragma optimize("", on)