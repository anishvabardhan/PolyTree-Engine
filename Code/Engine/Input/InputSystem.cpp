#include "Engine/Input/InputSystem.hpp"

#include "Engine/Core/NamedStrings.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Math/MathUtils.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>	

InputSystem* g_theInputSystem = nullptr;

unsigned char const KEYCODE_F1			= VK_F1;
unsigned char const KEYCODE_F2			= VK_F2;
unsigned char const KEYCODE_F3			= VK_F3;
unsigned char const KEYCODE_F4			= VK_F4;
unsigned char const KEYCODE_F5			= VK_F5;
unsigned char const KEYCODE_F6			= VK_F6;
unsigned char const KEYCODE_F7			= VK_F7;
unsigned char const KEYCODE_F8			= VK_F8;
unsigned char const KEYCODE_F9			= VK_F9;
unsigned char const KEYCODE_F10			= VK_F10;
unsigned char const KEYCODE_F11			= VK_F11;
unsigned char const KEYCODE_ESC			= VK_ESCAPE;
unsigned char const KEYCODE_SPACE		= VK_SPACE;
unsigned char const KEYCODE_UPARROW		= VK_UP;
unsigned char const KEYCODE_DOWNARROW	= VK_DOWN;
unsigned char const KEYCODE_LEFTARROW	= VK_LEFT;
unsigned char const KEYCODE_RIGHTARROW	= VK_RIGHT;
unsigned char const KEYCODE_LEFT_MOUSE	= VK_LBUTTON;
unsigned char const KEYCODE_RIGHT_MOUSE = VK_RBUTTON;
unsigned char const KEYCODE_BACKSPACE	= VK_BACK;
unsigned char const KEYCODE_SHIFT		= VK_SHIFT;
unsigned char const KEYCODE_ENTER		= VK_RETURN;
unsigned char const KEYCODE_INSERT		= VK_INSERT;
unsigned char const KEYCODE_DELETE		= VK_DELETE;
unsigned char const KEYCODE_HOME		= VK_HOME;
unsigned char const KEYCODE_END			= VK_END;
unsigned char const KEYCODE_OEM_4		= VK_OEM_4;
unsigned char const KEYCODE_OEM_6		= VK_OEM_6;
unsigned char const KEYCODE_OEM_COMMA	= VK_OEM_COMMA;
unsigned char const KEYCODE_OEM_PERIOD	= VK_OEM_PERIOD;
unsigned char const KEYCODE_F12			= 0x81;
unsigned char const KEYCODE_TILDE		= 0xC0;

InputSystem::InputSystem(InputConfig const& config)
	: m_config(config)
{
	for (int id = 0; id < NUM_XBOX_CONTROLLERS; id++)
	{
		m_controllers[id] = XboxController(id);
	}
}

InputSystem::~InputSystem()
{
}

void InputSystem::StartUp()
{
	SubscribeEventCallbackFunction("KEYPRESSED", InputSystem::Event_KeyPressed);
	SubscribeEventCallbackFunction("KEYRELEASED", InputSystem::Event_KeyReleased);
}

void InputSystem::BeginFrame()
{
	for (int id = 0; id < NUM_XBOX_CONTROLLERS; id++)
	{
		m_controllers[id].Update();
	}

	if (g_theWindow->GetHwnd() == ::GetActiveWindow())
	{
		if (m_cursorState.m_cursorHiddenMode)
		{
			::ShowCursor(false);
		}
		else
		{
			::ShowCursor(true);
		}

		if (m_cursorState.m_cursorRelativeMode)
		{
			HWND windowHandle = HWND(g_theWindow->GetHwnd());
			POINT cursorCoords;
			RECT clientRect;

			::GetCursorPos(&cursorCoords);
			::ScreenToClient(windowHandle, &cursorCoords);
			::GetClientRect(windowHandle, &clientRect);

			m_previousCursorPosition = IntVec2(cursorCoords.x, cursorCoords.y);

			RECT windowRect;
			::GetWindowRect((HWND)g_theWindow->GetHwnd(), &windowRect);

			Vec2 windowCenter = Vec2((float)(windowRect.right + windowRect.left) * 0.5f, (float)(windowRect.bottom + windowRect.top) * 0.5f);

			::SetCursorPos(int(windowCenter.x), int(windowCenter.y));

			::GetCursorPos(&cursorCoords);
			::ScreenToClient(windowHandle, &cursorCoords);

			m_cursorState.m_cursorClientPosition = IntVec2(cursorCoords.x, cursorCoords.y);

			m_cursorState.m_cursorClientDelta = m_cursorState.m_cursorClientPosition - m_previousCursorPosition;
		}
		else
		{
			m_cursorState.m_cursorClientDelta = IntVec2::ZERO;
		}
	}
}

void InputSystem::EndFrame()
{
	for (int index = 0; index < NUM_KEYCODES; index++)
	{
		m_keyStates[index].m_wasPressedLastFrame = m_keyStates[index].m_isPressed;
	}
}

void InputSystem::ShutDown()
{
}

void InputSystem::HandleKeyPressed(unsigned char keyCode)
{
	m_keyStates[keyCode].m_isPressed = true;
}

void InputSystem::HandleKeyReleased(unsigned char keyCode)
{
	m_keyStates[keyCode].m_isPressed = false;
}

void InputSystem::ResetCursorMode()
{
	if (m_cursorState.m_cursorHiddenMode)
	{
		while(ShowCursor(false) >= 0);
	}
	else
	{
		while (ShowCursor(true) < 0);
	}
}

void InputSystem::SetCursorMode(bool hiddenMode, bool relativeMode)
{
	m_cursorState.m_cursorHiddenMode = hiddenMode;
	m_cursorState.m_cursorRelativeMode = relativeMode;
}

IntVec2 InputSystem::GetCursorClientDelta() const
{
	return m_cursorState.m_cursorClientDelta;
}

Vec2 InputSystem::GetCursorNormalisedPosition() const
{
	HWND windowHandle = HWND(g_theWindow->GetHwnd());
	POINT cursorCoords;
	RECT clientRect;

	::GetCursorPos(&cursorCoords);
	::ScreenToClient(windowHandle, &cursorCoords);
	::GetClientRect(windowHandle, &clientRect);

	float cursorX = (static_cast<float>(cursorCoords.x) / static_cast<float>(clientRect.right - clientRect.left));
	float cursorY = (static_cast<float>(cursorCoords.y) / static_cast<float>(clientRect.bottom - clientRect.top));

	return Vec2(cursorX, 1.0f - cursorY);
}

Vec3 InputSystem::GetMouseForward(float fov, Camera const& camera)
{
	Vec2 cursorPos = GetCursorNormalisedPosition();

	float screenWidth = (float)g_theWindow->GetClientDimensions().x;
	float screenHeight = (float)g_theWindow->GetClientDimensions().y;

	float planeDistance = screenHeight / (2.0f * TanDegrees(fov * 0.5f));

	Vec3 forward = camera.GetModelMatrix().GetIBasis3D();
	Vec3 left = camera.GetModelMatrix().GetJBasis3D();
	Vec3 up = camera.GetModelMatrix().GetKBasis3D();

	Vec3 center = camera.m_position + forward * planeDistance;
	Vec3 rightOffset = -1.0f * left * (cursorPos.x - 0.5f) * screenWidth;
	Vec3 upOffset = up * (cursorPos.y - 0.5f) * screenHeight;

	Vec3 target = center + rightOffset + upOffset;

	return (target - camera.m_position).GetNormalized();
}

bool InputSystem::IsKeyDown(unsigned char keyCode)
{
	return m_keyStates[keyCode].m_isPressed;
}

bool InputSystem::WasKeyJustPressed(unsigned char keyCode)
{
	return m_keyStates[keyCode].m_isPressed && !m_keyStates[keyCode].m_wasPressedLastFrame;
}

bool InputSystem::WasKeyJustReleased(unsigned char keyCode)
{
	return !m_keyStates[keyCode].m_isPressed && m_keyStates[keyCode].m_wasPressedLastFrame;
}

bool InputSystem::Event_KeyPressed(EventArgs& args)
{
	if(!g_theInputSystem)
		return false;

	unsigned char keyCode = (unsigned char)args.GetValue("KeyCode", -1);
	g_theInputSystem->HandleKeyPressed(keyCode);

	return true;
}

bool InputSystem::Event_KeyReleased(EventArgs& args)
{
	if (!g_theInputSystem)
		return false;

	unsigned char keyCode = (unsigned char)args.GetValue("KeyCode", -1);
	g_theInputSystem->HandleKeyReleased(keyCode);

	return true;
}

XboxController const& InputSystem::GetController(int controllerID) const
{
	return m_controllers[controllerID];
}
