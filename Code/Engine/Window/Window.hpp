#pragma once

#include <string>

#include "Engine/Math/IntVec2.hpp"

struct Vec2;
struct IntVec2;
class InputSystem;

struct WindowConfig 
{
	InputSystem* m_inputSystem	= nullptr;
	std::string m_windowTitle	= "Untitled App";
	float m_clientAspect		= 2.0f;
	bool m_isFullScreen			= false;
	IntVec2 m_size				= IntVec2(-1, -1);
	IntVec2 m_pos				= IntVec2(-1, -1);
};

class Window
{
protected:
	void* m_dc = nullptr;
	void* m_hWnd = nullptr;
	WindowConfig m_config;
	IntVec2 m_clientDimensions;
	static Window* s_mainWindow;
public:
	Window(WindowConfig const& config);
	~Window();

	void StartUp();
	void BeginFrame();
	void EndFrame();
	void ShutDown();

	WindowConfig const& GetConfig() const;
	float GetAspect() const;
	void* GetHwnd() const;
	IntVec2 GetClientDimensions() const;
	void* GetDeviceContext() const;

	static Window* GetWindowContext();

	bool GetFileName(std::string& fileName);

protected:
	void CreateOSWindow();
	void RunMessagePump();
};