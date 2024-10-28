#pragma once

struct KeyButtonState
{
public:
	bool m_isPressed = false;
	bool m_wasPressedLastFrame = false;
public:
	KeyButtonState();
	~KeyButtonState();

	void UpdateStatus(bool isPressed);

	void Reset();
};