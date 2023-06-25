#include "input/api/Joycon/JoyconController.h"
#include "input/api/Joycon/JoyconTypes.h"
#pragma optimize("", off)
JoyconController::JoyconController(uint32 type, uint32 index)
	: base_type(fmt::format("{}_{}", type, index), fmt::format("{} {}", GetJoyconName(type), index + 1)), m_type(type),
	  m_index(index)
{
	m_settings.axis.range = 1.20f;
	m_settings.rotation.range = 1.25f;
	m_settings.trigger.range = 1.07f;
}

bool JoyconController::is_connected()
{
	return m_provider->is_connected(m_type, m_index);
}

bool JoyconController::connect() {
	return m_provider->is_connected(m_type, m_index);
}

bool JoyconController::has_battery() {
	return m_provider->has_battery(m_type, m_index);
}
bool JoyconController::has_low_battery() {
	return m_provider->has_low_battery(m_type, m_index);
}

bool JoyconController::has_motion() {
	return m_provider->has_motion(m_type, m_index);
}

bool JoyconController::has_rumble()
{
	return m_provider->has_rumble_connected(m_type, m_index);
}

void JoyconController::start_rumble()
{
	if (m_settings.rumble <= 0)
		return;

	m_provider->set_rumble_state(m_type, m_index, true);
}

void JoyconController::stop_rumble()
{
	m_provider->set_rumble_state(m_type, m_index, false);
}

MotionSample JoyconController::get_motion_sample() {
	return {};
}


std::string JoyconController::get_button_name(uint64 button) const
{
	switch (button)
	{
	case kButton0: return "Dpad down";
	case kButton1: return "Dpad up";
	case kButton2: return "Dpad right";
	case kButton3: return "Dpad left";
	case kButton4: return "SR";
	case kButton5: return "SL";
	case kButton6: return "L";
	case kButton7: return "ZL";
	case kButton8: return "Y";
	case kButton9: return "X";
	case kButton10: return "B";
	case kButton11: return "A";
	case kButton12: return "SR";
	case kButton13: return "SL";

	case kButton14: return "R";
	case kButton15: return "ZR";
	case kButton16: return "Minus";
	case kButton17: return "Plus";

	case kButton18: return "Stick R";
	case kButton19: return "Stick L";

	case kButton20: return "Home";
	case kButton21: return "Capture";
	}

	return base_type::get_button_name(button);
}

std::string JoyconController::GetJoyconName(uint32 type) const {
	switch (type) {
	case 1:
		return "Left Joycon";
	case 2:
		return "Right Joycon";
	case 3:
		return "Pro Controller";
	case 4:
		return "Dual Joycon";
	default:
		return "Unknown Switch Controller";
	}
}

ControllerState JoyconController::raw_state()
{
	ControllerState result{};
	if (!is_connected())
		return result;

	const auto state = m_provider->get_state(m_type, m_index);
	if (state.valid)
	{

		for (auto i = 0; i <= kButton11; ++i)
		{
			result.buttons.SetButtonState(i, HAS_BIT(state.button, i));
		}

		// printf("(%d, %d) - (%d, %d) - (%d, %d)\n", state.lstick_x, state.lstick_y, state.rstick_x, state.rstick_y, state.lstick, state.rstick);
		result.axis.x = state.lstick_x;

		result.axis.y = state.lstick_y;

		result.rotation.x = state.rstick_x;

		result.rotation.y = state.rstick_y;


		result.trigger.x = (float)state.lstick / std::numeric_limits<uint8>::max();
		result.trigger.y = (float)state.rstick / std::numeric_limits<uint8>::max();
	}
	
	return result;
}
#pragma optimize("", on)