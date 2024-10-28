#pragma once

#include "Engine/Input/KeyButtonState.hpp"
#include "Engine/Input/AnalogJoystick.hpp"

enum XboxButtonID
{
	BUTTON_A,					//=	XINPUT_GAMEPAD_A,
	BUTTON_B,					//=	XINPUT_GAMEPAD_B,
	BUTTON_X,					//=	XINPUT_GAMEPAD_X,
	BUTTON_Y,					//=	XINPUT_GAMEPAD_Y,
	BUTTON_LEFT_THUMB,			//=	XINPUT_GAMEPAD_LEFT_THUMB,
	BUTTON_RIGHT_THUMB,			//=	XINPUT_GAMEPAD_RIGHT_THUMB,
	BUTTON_LEFT_SHOULDER,		//=	XINPUT_GAMEPAD_LEFT_SHOULDER,
	BUTTON_RIGHT_SHOULDER,		//=	XINPUT_GAMEPAD_RIGHT_SHOULDER,
	BUTTON_START,				//=	XINPUT_GAMEPAD_START,
	BUTTON_BACK,				//=	XINPUT_GAMEPAD_BACK,
	BUTTON_DPAD_UP,				//=	XINPUT_GAMEPAD_DPAD_UP,
	BUTTON_DPAD_RIGHT,			//=	XINPUT_GAMEPAD_DPAD_RIGHT,
	BUTTON_DPAD_LEFT,			//=	XINPUT_GAMEPAD_DPAD_LEFT,
	BUTTON_DPAD_DOWN,			//=	XINPUT_GAMEPAD_DPAD_DOWN,
	XBOX_NUM
};

const float XBOX_STICK_INNER_DEADZONE_RADIUS_PERCENTAGE = 0.3f;
const float XBOX_STICK_OUTER_DEADZONE_RADIUS_PERCENTAGE = 0.95f;
const float XBOX_STICK_RAW_MAX = 32767.f;
const float XBOX_STICK_RAW_MIN = -32768.f;

class XboxController
{
	int								m_id = -1;
	bool							m_isConnected = false;
	float							m_leftTrigger = 0.0f;
	float							m_rightTrigger = 0.0f;
	KeyButtonState					m_keyButtonState[static_cast<int>(XboxButtonID::XBOX_NUM)];
	AnalogJoystick					m_leftStick;
	AnalogJoystick					m_rightStick;
private:
	void							Update();
	void							Reset();
	void							UpdateJoyStick(AnalogJoystick& out_joystick, short rawX, short rawY);
	void							UpdateTrigger(float& out_triggerValue, unsigned char rawValue);
	void							UpdateButton(XboxButtonID buttonID, unsigned short buttonFlags, unsigned short buttonFlag);
public:
									XboxController() {};
									XboxController(int controllerID);
									~XboxController();

	bool							IsConnected() const;
	int								GetControllerID() const;
	AnalogJoystick const&			GetLeftStick() const;
	AnalogJoystick const&			GetRightStick() const;
	float							GetLeftTrigger() const;
	float							GetRightTrigger() const;
	KeyButtonState const&			GetButton(XboxButtonID buttonID) const;
	bool							IsButtonDown(XboxButtonID buttonID) const;
	bool							WasButtonJustPressed(XboxButtonID buttonID) const;
	bool							WasButtonJustReleased(XboxButtonID buttonID) const;
private:
	friend class InputSystem;
};