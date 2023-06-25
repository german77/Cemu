#include "input/api/Joycon/JoyconControllerProvider.h"
#include "input/api/Joycon/JoyconController.h"
#include "input/api/Joycon/JoyconDriver.h"
#include "input/api/Joycon/JoyconTypes.h"

constexpr uint16_t kVendorId = 0x57e;

JoyconControllerProvider::JoyconControllerProvider()
{
	uint32 port = 0;
	for (auto& device : m_left_joycons) {
		device = std::make_shared<InputCommon::Joycon::JoyconDriver>(port++);
	}
	port = 0;
	for (auto& device : m_right_joycons) {
		device = std::make_shared<InputCommon::Joycon::JoyconDriver>(port++);
	}
	port = 0;
	for (auto& device : m_pro_controller) {
		device = std::make_shared<InputCommon::Joycon::JoyconDriver>(port++);
	}

	m_running = true;
	m_reader_thread = std::thread(&JoyconControllerProvider::reader_thread, this);
	m_writer_thread = std::thread(&JoyconControllerProvider::writer_thread, this);
}

JoyconControllerProvider::~JoyconControllerProvider()
{
	if (m_running)
	{
		m_running = false;
		m_reader_thread.join();
	}

	for (const auto& device : m_left_joycons) {
		if (!device) {
			continue;
		}
		device->Stop();
	}
	for (const auto& device : m_right_joycons) {
		if (!device) {
			continue;
		}
		device->Stop();
	}
	for (const auto& device : m_pro_controller) {
		if (!device) {
			continue;
		}
		device->Stop();
	}
}

std::vector<ControllerPtr> JoyconControllerProvider::get_controllers()
{
	std::vector<ControllerPtr> result;

	for (const auto& device : m_left_joycons) {
		if (!device) {
			continue;
		}
		if (device->IsConnected()) {
			const uint32 type = static_cast<uint32>(device->GetDeviceType());
			const uint32 index = static_cast<uint32>(device->GetDevicePort());
			result.emplace_back(std::make_shared<JoyconController>(type, index));
		}
	}
	for (const auto& device : m_right_joycons) {
		if (!device) {
			continue;
		}
		if (device->IsConnected()) {
			const uint32 type = static_cast<uint32>(device->GetDeviceType());
			const uint32 index = static_cast<uint32>(device->GetDevicePort());
			result.emplace_back(std::make_shared<JoyconController>(type, index));
		}
	}
	for (const auto& device : m_pro_controller) {
		if (!device) {
			continue;
		}
		if (device->IsConnected()) {
			const uint32 type = static_cast<uint32>(device->GetDeviceType());
			const uint32 index = static_cast<uint32>(device->GetDevicePort());
			result.emplace_back(std::make_shared<JoyconController>(type, index));
		}
	}

	return result;
}

bool JoyconControllerProvider::has_battery(uint32 controller_type, uint32 controller_index) const
{
	const auto handle = GetHandle(controller_type, controller_index);
	if (handle == nullptr) {
		return false;
	}
	return true;
}

bool JoyconControllerProvider::has_low_battery(uint32 controller_type, uint32 controller_index) const
{
	const auto handle = GetHandle(controller_type, controller_index);
	if (handle == nullptr) {
		return false;
	}
	return false;
}

bool JoyconControllerProvider::has_motion(uint32 controller_type, uint32 controller_index) const
{
	const auto handle = GetHandle(controller_type, controller_index);
	if (handle == nullptr) {
		return false;
	}
	return handle->IsMotionEnabled();
}


bool JoyconControllerProvider::has_rumble_connected(uint32 controller_type, uint32 controller_index) const
{
	const auto handle = GetHandle(controller_type, controller_index);
	if (handle == nullptr) {
		return false;
	}
	return handle->IsVibrationEnabled();
}

bool JoyconControllerProvider::is_connected(uint32 controller_type, uint32 controller_index) const
{
	const auto handle = GetHandle(controller_type, controller_index);
	if (handle == nullptr) {
		return false;
	}
	return handle->IsConnected();
}

void JoyconControllerProvider::set_rumble_state(uint32 controller_type, uint32 controller_index, bool state)
{
}

InputCommon::Joycon::JCState JoyconControllerProvider::get_state(uint32 controller_type, uint32 controller_index)
{
	const auto handle = GetHandle(controller_type, controller_index);
	if (handle == nullptr) {
		return {};
	}
	if (!handle->IsConnected()) {
		return {};
	}
	return handle->GetState();
}

#ifdef interface
#undef interface
#endif

bool JoyconControllerProvider::IsDeviceNew(SDL_hid_device_info* device_info) const {
	InputCommon::Joycon::ControllerType type{};
	InputCommon::Joycon::SerialNumber serial_number{};

	const auto result = InputCommon::Joycon::JoyconDriver::GetDeviceType(device_info, type);
	if (result != InputCommon::Joycon::DriverResult::Success) {
		return false;
	}

	const auto result2 = InputCommon::Joycon::JoyconDriver::GetSerialNumber(device_info, serial_number);
	if (result2 != InputCommon::Joycon::DriverResult::Success) {
		return false;
	}

	auto is_handle_identical = [serial_number](std::shared_ptr<InputCommon::Joycon::JoyconDriver> device) {
		if (!device) {
			return false;
		}
		if (!device->IsConnected()) {
			return false;
		}
		if (device->GetHandleSerialNumber() != serial_number) {
			return false;
		}
		return true;
	};

	// Check if device already exist
	switch (type) {
	case InputCommon::Joycon::ControllerType::Left:
		for (const auto& device : m_left_joycons) {
			if (is_handle_identical(device)) {
				return false;
			}
		}
		break;
	case InputCommon::Joycon::ControllerType::Right:
		for (const auto& device : m_right_joycons) {
			if (is_handle_identical(device)) {
				return false;
			}
		}
		break;
	case InputCommon::Joycon::ControllerType::Pro:
		for (const auto& device : m_pro_controller) {
			if (is_handle_identical(device)) {
				return false;
			}
		}
		break;
	default:
		return false;
	}

	return true;
}

void JoyconControllerProvider::RegisterNewDevice(SDL_hid_device_info* device_info) {
	InputCommon::Joycon::ControllerType type{};
	auto result = InputCommon::Joycon::JoyconDriver::GetDeviceType(device_info, type);
	auto handle = GetNextFreeHandle(type);
	if (handle == nullptr) {
		return;
	}
	if (result == InputCommon::Joycon::DriverResult::Success) {
		result = handle->RequestDeviceAccess(device_info);
	}
	if (result == InputCommon::Joycon::DriverResult::Success) {
		handle->InitializeDevice();
	}
}

std::shared_ptr<InputCommon::Joycon::JoyconDriver> JoyconControllerProvider::GetNextFreeHandle(
	InputCommon::Joycon::ControllerType type) const {
	if (type == InputCommon::Joycon::ControllerType::Left) {
		const auto unconnected_device =
			std::ranges::find_if(m_left_joycons, [](auto& device) { return !device->IsConnected(); });
		if (unconnected_device != m_left_joycons.end()) {
			return *unconnected_device;
		}
	}
	if (type == InputCommon::Joycon::ControllerType::Right) {
		const auto unconnected_device = std::ranges::find_if(
			m_right_joycons, [](auto& device) { return !device->IsConnected(); });

		if (unconnected_device != m_right_joycons.end()) {
			return *unconnected_device;
		}
	}
	if (type == InputCommon::Joycon::ControllerType::Pro) {
		const auto unconnected_device = std::ranges::find_if(
			m_pro_controller, [](auto& device) { return !device->IsConnected(); });

		if (unconnected_device != m_pro_controller.end()) {
			return *unconnected_device;
		}
	}
	return nullptr;
}

std::shared_ptr<InputCommon::Joycon::JoyconDriver> JoyconControllerProvider::GetHandle(uint32 controller_type, uint32 controller_index) const {
	auto is_handle_active = [&](std::shared_ptr<InputCommon::Joycon::JoyconDriver> device) {
		if (!device) {
			return false;
		}
		if (!device->IsConnected()) {
			return false;
		}
		if (device->GetDevicePort() == controller_index) {
			return true;
		}
		return false;
	};
	const auto type = static_cast<InputCommon::Joycon::ControllerType>(controller_type);

	if (type == InputCommon::Joycon::ControllerType::Left) {
		const auto matching_device = std::ranges::find_if(
			m_left_joycons, [is_handle_active](auto& device) { return is_handle_active(device); });

		if (matching_device != m_left_joycons.end()) {
			return *matching_device;
		}
	}

	if (type == InputCommon::Joycon::ControllerType::Right) {
		const auto matching_device = std::ranges::find_if(
			m_right_joycons, [is_handle_active](auto& device) { return is_handle_active(device); });

		if (matching_device != m_right_joycons.end()) {
			return *matching_device;
		}
	}

	if (type == InputCommon::Joycon::ControllerType::Pro) {
		const auto matching_device = std::ranges::find_if(
			m_pro_controller, [is_handle_active](auto& device) { return is_handle_active(device); });

		if (matching_device != m_pro_controller.end()) {
			return *matching_device;
		}
	}

	return nullptr;
}

void JoyconControllerProvider::reader_thread()
{
	SetThreadName("JCControllerAdapter::reader_thread");

	while (m_running.load(std::memory_order_relaxed))
	{
		SDL_hid_device_info* devs = SDL_hid_enumerate(kVendorId, 0x0);
		SDL_hid_device_info* cur_dev = devs;

		while (cur_dev) {
			if (IsDeviceNew(cur_dev)) {
				RegisterNewDevice(cur_dev);
			}
			cur_dev = cur_dev->next;
		}

		SDL_hid_free_enumeration(devs);
	}
}

void JoyconControllerProvider::writer_thread()
{
	SetThreadName("GCControllerAdapter::writer_thread");

	std::array<std::array<bool, 4>, kMaxAdapters> rumble_states{};

	while (m_running.load(std::memory_order_relaxed))
	{
		std::unique_lock lock(m_writer_mutex);
		if (!m_rumble_changed && m_writer_cond.wait_for(lock, std::chrono::milliseconds(250)) == std::cv_status::timeout)
		{
			if (!m_running)
				return;

			continue;
		}

		bool cmd_sent = false;
		for (size_t i = 0; i < kMaxAdapters; ++i)
		{
			
		}

		if(cmd_sent)
		{
			lock.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}
}
