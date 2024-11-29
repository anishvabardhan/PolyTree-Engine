#pragma once

#include "Engine/Input/KeyButtonState.hpp"
#include "Engine/Input/XboxController.hpp"
#include "Engine/Core/EventSystem.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Renderer/Camera.hpp"

extern unsigned char const KEYCODE_F1;
extern unsigned char const KEYCODE_F2;
extern unsigned char const KEYCODE_F3;
extern unsigned char const KEYCODE_F4;
extern unsigned char const KEYCODE_F5;
extern unsigned char const KEYCODE_F6;
extern unsigned char const KEYCODE_F7;
extern unsigned char const KEYCODE_F8;
extern unsigned char const KEYCODE_F9;
extern unsigned char const KEYCODE_F10;
extern unsigned char const KEYCODE_F11;
extern unsigned char const KEYCODE_F12;
extern unsigned char const KEYCODE_TILDE;
extern unsigned char const KEYCODE_ESC;
extern unsigned char const KEYCODE_SPACE;
extern unsigned char const KEYCODE_UPARROW;
extern unsigned char const KEYCODE_DOWNARROW;
extern unsigned char const KEYCODE_LEFTARROW;
extern unsigned char const KEYCODE_RIGHTARROW;
extern unsigned char const KEYCODE_LEFT_MOUSE;
extern unsigned char const KEYCODE_RIGHT_MOUSE;
extern unsigned char const KEYCODE_BACKSPACE;
extern unsigned char const KEYCODE_SHIFT;
extern unsigned char const KEYCODE_ENTER;
extern unsigned char const KEYCODE_INSERT;
extern unsigned char const KEYCODE_DELETE;
extern unsigned char const KEYCODE_HOME;
extern unsigned char const KEYCODE_END;
extern unsigned char const KEYCODE_OEM_4;
extern unsigned char const KEYCODE_OEM_6;
extern unsigned char const KEYCODE_OEM_COMMA;
extern unsigned char const KEYCODE_OEM_PERIOD;

constexpr int NUM_KEYCODES			= 256;
constexpr int NUM_XBOX_CONTROLLERS	= 4;

struct CursorState
{
	IntVec2 m_cursorClientDelta;
	IntVec2 m_cursorClientPosition;

	bool m_cursorHiddenMode = false;
	bool m_cursorRelativeMode = false;
};

struct InputConfig
{

};

class InputSystem
{
	XboxController			m_controllers[NUM_XBOX_CONTROLLERS];
	KeyButtonState			m_keyStates[NUM_KEYCODES];
protected:
	InputConfig				m_config;
	IntVec2					m_previousCursorPosition;
public:
	CursorState				m_cursorState;
public:
							InputSystem(InputConfig const& config);
							~InputSystem();

	void					StartUp();
	void					BeginFrame();
	void					EndFrame();
	void					ShutDown();

	void					HandleKeyPressed(unsigned char keyCode);
	void					HandleKeyReleased(unsigned char keyCode);

	void					ResetCursorMode();

	void					SetCursorMode(bool hiddenMode, bool relativeMode);
	IntVec2					GetCursorClientDelta() const;
	Vec2					GetCursorNormalisedPosition() const;

	Vec3					GetMouseForward(float fov, Camera const& camera);

	bool					IsKeyDown(unsigned char keyCode);
	bool					WasKeyJustPressed(unsigned char keyCode);
	bool					WasKeyJustReleased(unsigned char keyCode);

	static bool				Event_KeyPressed(EventArgs& args);
	static bool				Event_KeyReleased(EventArgs& args);

	XboxController const&	GetController(int controllerID) const;
};