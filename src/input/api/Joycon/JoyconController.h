#pragma once

#include "input/api/Controller.h"
#include "input/api/Joycon/JoyconControllerProvider.h"

class JoyconController : public Controller<JoyconControllerProvider>
{
public:
	JoyconController(uint32 adapter, uint32 index);
	
	std::string_view api_name() const override
	{
		static_assert(to_string(InputAPI::Joycon) == "Joycon");
		return to_string(InputAPI::Joycon);
	}
	InputAPI::Type api() const override { return InputAPI::Joycon; }

	bool is_connected() override;
	bool connect() override;

	bool has_battery() override;
	bool has_low_battery() override;

	bool has_motion() override;
	bool has_rumble() override;


	void start_rumble() override;
	void stop_rumble() override;

	MotionSample get_motion_sample() override;

	std::string get_button_name(uint64 button) const override;

protected:
	ControllerState raw_state() override;

	uint32 m_type;
	uint32 m_index;
};
