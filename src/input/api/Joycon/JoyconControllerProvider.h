#pragma once

#include <SDL2/SDL_hidapi.h>

#include "input/api/ControllerProvider.h"

// Pad Identifier of data source
struct PadIdentifier {
	uint64 guid{};
	std::size_t port{};
	std::size_t pad{};

	friend constexpr bool operator==(const PadIdentifier&, const PadIdentifier&) = default;
};

namespace InputCommon::Joycon {
	struct MotionData;
	struct TagInfo;
	enum class ControllerType : uint8;
	enum class DriverResult;
	class JoyconDriver;
} // namespace InputCommon::Joycon

class JoyconControllerProvider : public ControllerProviderBase
{
	friend class JoyconController;
public:
	constexpr static size_t kMaxAdapters = 4;
	constexpr static size_t kMaxIndex = 4;

	JoyconControllerProvider();
	~JoyconControllerProvider();

	inline static InputAPI::Type kAPIType = InputAPI::Joycon;
	InputAPI::Type api() const override { return kAPIType; }

	std::vector<std::shared_ptr<ControllerBase>> get_controllers() override;

	uint32 get_adapter_count() const;
	bool has_battery(uint32 controller_type, uint32 controller_index) const;
	bool has_low_battery(uint32 controller_type, uint32 controller_index) const;
	bool has_motion(uint32 controller_type, uint32 controller_index) const;
	bool has_rumble_connected(uint32 controller_type, uint32 controller_index) const;
	bool is_connected(uint32 controller_type, uint32 controller_index) const;

	void set_rumble_state(uint32 controller_type, uint32 controller_index, bool state);
	
	struct JCState
	{
		bool valid = false;
		uint16 button = 0;

		uint8 lstick_x = 0;
		uint8 lstick_y = 0;

		uint8 rstick_x = 0;
		uint8 rstick_y = 0;

		uint8 lstick = 0;
		uint8 rstick = 0;
	};
	JCState get_state(uint32 adapter_index, uint32 index);

private:
	std::atomic_bool m_running = false;
	std::thread m_reader_thread, m_writer_thread;

	void reader_thread();
	void writer_thread();


	/// Returns true if device is valid and not registered
	bool IsDeviceNew(SDL_hid_device_info* device_info) const;

	/// Tries to connect to the new device
	void RegisterNewDevice(SDL_hid_device_info* device_info);

	/// Returns the next free handle
	std::shared_ptr<InputCommon::Joycon::JoyconDriver> GetNextFreeHandle(InputCommon::Joycon::ControllerType type) const;

	/// Returns a JoyconHandle corresponding to a PadIdentifier
	std::shared_ptr<InputCommon::Joycon::JoyconDriver> GetHandle(uint32 controller_type, uint32 controller_index) const;

	void OnBatteryUpdate(std::size_t port, InputCommon::Joycon::ControllerType type, uint8 value);
	void OnButtonUpdate(std::size_t port, InputCommon::Joycon::ControllerType type, int id, bool value);
	void OnStickUpdate(std::size_t port, InputCommon::Joycon::ControllerType type, int id, float value);
	void OnMotionUpdate(std::size_t port, InputCommon::Joycon::ControllerType type, int id,
		const InputCommon::Joycon::MotionData& value);

	std::mutex m_writer_mutex;
	std::condition_variable m_writer_cond;
	bool m_rumble_changed = false;

	// Joycon types are split by type to ease supporting dualjoycon configurations
	std::array<std::shared_ptr<InputCommon::Joycon::JoyconDriver>, kMaxAdapters> m_left_joycons{};
	std::array<std::shared_ptr<InputCommon::Joycon::JoyconDriver>, kMaxAdapters> m_right_joycons{};
	std::array<std::shared_ptr<InputCommon::Joycon::JoyconDriver>, kMaxAdapters> m_pro_controller{};

};
