// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <span>
#include <thread>

#include "input/api/Joycon/JoyconTypes.h"

namespace InputCommon::Joycon {
class CalibrationProtocol;
class GenericProtocol;
class NfcProtocol;
class JoyconPoller;
class RumbleProtocol;

class JoyconDriver final {
public:
    explicit JoyconDriver(std::size_t port_);

    ~JoyconDriver();

    DriverResult RequestDeviceAccess(SDL_hid_device_info* device_info);
    DriverResult InitializeDevice();
    void Stop();

    bool IsConnected() const;
    bool IsMotionEnabled() const;
    bool IsVibrationEnabled() const;

    FirmwareVersion GetDeviceVersion() const;
    std::size_t GetDevicePort() const;
    ControllerType GetDeviceType() const;
    ControllerType GetHandleDeviceType() const;
    SerialNumber GetSerialNumber() const;
    SerialNumber GetHandleSerialNumber() const;

    DriverResult SetVibration(const VibrationValue& vibration);
    DriverResult SetLedConfig(uint8 led_pattern);
    DriverResult SetPassiveMode();
    DriverResult SetActiveMode();
    DriverResult SetNfcMode();
    DriverResult StartNfcPolling();
    DriverResult StopNfcPolling();
    DriverResult ReadAmiiboData(std::vector<uint8>& out_data);
    DriverResult WriteNfcData(std::span<const uint8> data);
    DriverResult ReadMifareData(std::span<const MifareReadChunk> request,
                                std::span<MifareReadData> out_data);
    DriverResult WriteMifareData(std::span<const MifareWriteChunk> request);

    JCState GetState();

    // Returns device type from hidapi handle
    static DriverResult GetDeviceType(SDL_hid_device_info* device_info,
                                      ControllerType& controller_type);

    // Returns serial number from hidapi handle
    static DriverResult GetSerialNumber(SDL_hid_device_info* device_info,
                                        SerialNumber& serial_number);

private:
    struct SupportedFeatures {
        bool passive{};
        bool motion{};
        bool nfc{};
        bool vibration{};
    };

    /// Main thread, actively request new data from the handle
    void InputThread(std::stop_token stop_token);

    /// Called every time a valid package arrives
    void OnNewData(std::span<uint8> buffer);

    /// Updates device configuration to enable or disable features
    DriverResult SetPollingMode();

    /// Returns true if input thread is valid and doesn't need to be stopped
    bool IsInputThreadValid() const;

    /// Returns true if the data should be interpreted. Otherwise the error counter is incremented
    bool IsPayloadCorrect(int status, std::span<const uint8> buffer);

    /// Returns a list of supported features that can be enabled on this device
    SupportedFeatures GetSupportedFeatures();

    // Protocol Features
    std::unique_ptr<CalibrationProtocol> calibration_protocol;
    std::unique_ptr<GenericProtocol> generic_protocol;
    std::unique_ptr<NfcProtocol> nfc_protocol;
    std::unique_ptr<JoyconPoller> joycon_poller;
    std::unique_ptr<RumbleProtocol> rumble_protocol;

    // Connection status
    std::atomic<bool> is_connected{};
    uint64 delta_time;
    std::size_t error_counter{};
    std::shared_ptr<JoyconHandle> hidapi_handle;
    std::chrono::time_point<std::chrono::steady_clock> last_update;

    // External device status
    bool amiibo_detected{};

    // Hardware configuration
    uint8 leds{};
    ReportMode mode{};
    bool passive_enabled{};   // Low power mode, Ideal for multiple controllers at the same time
    bool motion_enabled{};    // Enables motion input
    bool nfc_enabled{};       // Enables Amiibo detection
    bool vibration_enabled{}; // Allows vibrations

    // Calibration data
    GyroSensitivity gyro_sensitivity{};
    GyroPerformance gyro_performance{};
    AccelerometerSensitivity accelerometer_sensitivity{};
    AccelerometerPerformance accelerometer_performance{};
    JoyStickCalibration left_stick_calibration{};
    JoyStickCalibration right_stick_calibration{};
    MotionCalibration motion_calibration{};

    // Fixed joycon info
    FirmwareVersion version{};
    std::size_t port{};
    ControllerType device_type{};        // Device type reported by controller
    ControllerType handle_device_type{}; // Device type reported by hidapi
    SerialNumber serial_number{};        // Serial number reported by controller
    SerialNumber handle_serial_number{}; // Serial number type reported by hidapi
    SupportedFeatures supported_features{};

    // Thread related
    mutable std::mutex mutex;
    std::jthread input_thread;
    bool input_thread_running{};
    bool disable_input_thread{};
    JCState state{};
};

} // namespace InputCommon::Joycon
