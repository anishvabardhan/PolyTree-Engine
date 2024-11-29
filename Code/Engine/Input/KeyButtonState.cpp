#include "Engine/Input/KeyButtonState.hpp"

KeyButtonState::KeyButtonState()
{
}

KeyButtonState::~KeyButtonState()
{
}

void KeyButtonState::UpdateStatus(bool isPressed)
{
	m_wasPressedLastFrame = m_isPressed;
	m_isPressed = isPressed;
}

void KeyButtonState::Reset()
{
	m_isPressed = false;
	m_wasPressedLastFrame = false;
}
