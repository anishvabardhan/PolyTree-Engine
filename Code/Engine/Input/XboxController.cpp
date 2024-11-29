#include "Engine/Input/XboxController.hpp"

#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>	

#include <Xinput.h>

#pragma comment( lib, "xinput9_1_0" )

void XboxController::Update()
{
	XINPUT_STATE xboxControllerState = {};
	DWORD errorStatus = XInputGetState(m_id, &xboxControllerState);

	if (errorStatus == ERROR_SUCCESS)
	{
		m_isConnected = true;

		UpdateTrigger(m_leftTrigger, xboxControllerState.Gamepad.bLeftTrigger);
		UpdateTrigger(m_rightTrigger, xboxControllerState.Gamepad.bRightTrigger);

		UpdateJoyStick(m_leftStick, xboxControllerState.Gamepad.sThumbLX, xboxControllerState.Gamepad.sThumbLY);
		UpdateJoyStick(m_rightStick, xboxControllerState.Gamepad.sThumbRX, xboxControllerState.Gamepad.sThumbRY);

		UpdateButton(XboxButtonID::BUTTON_A, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_A);
		UpdateButton(XboxButtonID::BUTTON_B, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_B);
		UpdateButton(XboxButtonID::BUTTON_X, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_X);
		UpdateButton(XboxButtonID::BUTTON_Y, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_Y);
		UpdateButton(XboxButtonID::BUTTON_LEFT_THUMB, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_LEFT_THUMB);
		UpdateButton(XboxButtonID::BUTTON_RIGHT_THUMB, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_RIGHT_THUMB);
		UpdateButton(XboxButtonID::BUTTON_LEFT_SHOULDER, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_LEFT_SHOULDER);
		UpdateButton(XboxButtonID::BUTTON_RIGHT_SHOULDER, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_RIGHT_SHOULDER);
		UpdateButton(XboxButtonID::BUTTON_START, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_START);
		UpdateButton(XboxButtonID::BUTTON_BACK, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_BACK);
		UpdateButton(XboxButtonID::BUTTON_DPAD_UP, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_DPAD_UP);
		UpdateButton(XboxButtonID::BUTTON_DPAD_RIGHT, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_DPAD_RIGHT);
		UpdateButton(XboxButtonID::BUTTON_DPAD_LEFT, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_DPAD_LEFT);
		UpdateButton(XboxButtonID::BUTTON_DPAD_DOWN, xboxControllerState.Gamepad.wButtons, XINPUT_GAMEPAD_DPAD_DOWN);
	}
}

void XboxController::Reset()
{
	m_isConnected = false;

	for (int index = 0; index < XboxButtonID::XBOX_NUM; index++)
	{
		m_keyButtonState[index].Reset();
	}

	m_leftStick.Reset();
	m_rightStick.Reset();
	m_leftTrigger = 0.0f;
	m_rightTrigger = 0.0f;
}

void XboxController::UpdateJoyStick(AnalogJoystick& out_joystick, short rawX, short rawY)
{
	float normalizedX = RangeMap((float)rawX, XBOX_STICK_RAW_MIN, XBOX_STICK_RAW_MAX, -1.0f, 1.0f);
	float normalizedY = RangeMap((float)rawY, XBOX_STICK_RAW_MIN, XBOX_STICK_RAW_MAX, -1.0f, 1.0f);

	out_joystick.UpdatePosition(normalizedX, normalizedY);
}

void XboxController::UpdateTrigger(float& out_triggerValue, unsigned char rawValue)
{
	out_triggerValue = (1.0f / 255.0f) * (float)rawValue;
}

void XboxController::UpdateButton(XboxButtonID buttonID, unsigned short buttonFlags, unsigned short buttonFlag)
{
	bool isPressed = (buttonFlags & buttonFlag) == buttonFlag;
	m_keyButtonState[buttonID].UpdateStatus(isPressed);
}

XboxController::XboxController(int controllerID)
	:m_id(controllerID)
{
	m_leftStick.SetDeadZoneThresholds(XBOX_STICK_INNER_DEADZONE_RADIUS_PERCENTAGE, XBOX_STICK_OUTER_DEADZONE_RADIUS_PERCENTAGE);
	m_rightStick.SetDeadZoneThresholds(XBOX_STICK_INNER_DEADZONE_RADIUS_PERCENTAGE, XBOX_STICK_OUTER_DEADZONE_RADIUS_PERCENTAGE);
}

XboxController::~XboxController()
{
}

bool XboxController::IsConnected() const
{
	return m_isConnected;
}

int XboxController::GetControllerID() const
{
	return m_id;
}

AnalogJoystick const& XboxController::GetLeftStick() const
{
	return m_leftStick;
}

AnalogJoystick const& XboxController::GetRightStick() const
{
	return m_rightStick;
}

float XboxController::GetLeftTrigger() const
{
	return m_leftTrigger;
}

float XboxController::GetRightTrigger() const
{
	return m_rightTrigger;
}

KeyButtonState const& XboxController::GetButton(XboxButtonID buttonID) const
{
	return m_keyButtonState[buttonID];
}

bool XboxController::IsButtonDown(XboxButtonID buttonID) const
{
	return m_keyButtonState[buttonID].m_isPressed;
}

bool XboxController::WasButtonJustPressed(XboxButtonID buttonID) const
{
	return m_keyButtonState[buttonID].m_isPressed && !m_keyButtonState[buttonID].m_wasPressedLastFrame;
}

bool XboxController::WasButtonJustReleased(XboxButtonID buttonID) const
{
	return !m_keyButtonState[buttonID].m_isPressed && m_keyButtonState[buttonID].m_wasPressedLastFrame;
}
