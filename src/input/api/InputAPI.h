#pragma once

#include "util/helpers/helpers.h"

namespace InputAPI
{
	enum Type
	{
		DirectInput,
		DSUClient,
		GameCube,
		Joycon,
		Keyboard,
		SDLController,
		Wiimote,
		XInput,

		WGIGamepad,
		WGIRawController,

		MAX
	};

	constexpr std::string_view to_string(Type type)
	{
		switch (type)
		{
		case DirectInput:
			return "DirectInput";
		case DSUClient:
			return "DSUController";
		case GameCube:
			return "GameCube";
		case Joycon:
			return "Joycon";
		case Keyboard:
			return "Keyboard";
		case SDLController:
			return "SDLController";
		case Wiimote:
			return "Wiimote";
		case XInput:
			return "XInput";
		case WGIGamepad:
			return "WGIGamepad";
		case WGIRawController:
			return "WGIRawController";
		default:
			break;
		}

		throw std::runtime_error(fmt::format("unknown input api: {}", to_underlying(type)));
	}

	constexpr Type from_string(std::string_view str)
	{
		if (str == to_string(DirectInput))
			return DirectInput;
		else if (str == to_string(DSUClient))
			return DSUClient;
		else if (str == "DSU") // legacy
			return DSUClient;
		else if (str == to_string(GameCube))
			return GameCube;
		else if (str == to_string(Joycon))
			return Joycon;
		else if (str == to_string(Keyboard))
				return Keyboard;
		else if (str == to_string(SDLController))
			return SDLController;
		else if (str == to_string(Wiimote))
			return Wiimote;
		else if (str == to_string(XInput))
			return XInput;
		
		//else if (str == "WGIGamepad")
		//	return WGIGamepad;
		//
		//else if (str == "WGIRawController")
		//	return WGIRawController;

		throw std::runtime_error(fmt::format("unknown input api: {}", str));
	}
}
