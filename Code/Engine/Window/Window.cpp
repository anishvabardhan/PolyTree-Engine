#include "Engine/Window/Window.hpp"

#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Core/NamedStrings.hpp"
#include "Engine/Core/EventSystem.hpp"

#include "Engine/Input/InputSystem.hpp"

#include "ThirdParty/ImGui/imgui.h"
#include "ThirdParty/ImGui/imgui_impl_win32.h"
#include "ThirdParty/ImGui/imgui_impl_dx12.h"

#define WIN32_LEAN_AND_MEAN		// Always #define this before #including <windows.h>
#include <windows.h>
#include <commdlg.h>

Window* Window::s_mainWindow = nullptr;

LRESULT CALLBACK WindowsMessageHandlingProcedure(HWND windowHandle, UINT wmMessageCode, WPARAM wParam, LPARAM lParam)
{
	extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
	if (ImGui_ImplWin32_WndProcHandler(windowHandle, wmMessageCode, wParam, lParam))
		return true;

	switch (wmMessageCode)
	{
		// App close requested via "X" button, or right-click "Close Window" on task bar, or "Close" from system menu, or Alt-F4
	case WM_CLOSE:
	{
		EventArgs args;
		FireEvent("QUIT", args);

		return 0; // "Consumes" this message (tells Windows "okay, we handled it")
	}
	case WM_CHAR:
	{
		EventArgs args;
		args.SetValue("KeyCode", Stringf("%d", (unsigned char)wParam));
		FireEvent("CHARINPUT", args);

		return 0;
	}
	// Raw physical keyboard "key-was-just-depressed" event (case-insensitive, not translated)
	case WM_KEYDOWN:
	{
		EventArgs args;
		args.SetValue("KeyCode", Stringf("%d", (unsigned char)wParam));
		FireEvent("KEYPRESSED", args);

		return 0;
	}

	// Raw physical keyboard "key-was-just-released" event (case-insensitive, not translated)
	case WM_KEYUP:
	{
		EventArgs args;
		args.SetValue("KeyCode", Stringf("%d", (unsigned char)wParam));
		FireEvent("KEYRELEASED", args);

		return 0;
	}

	case WM_LBUTTONDOWN:
	{
		unsigned char asKey = KEYCODE_LEFT_MOUSE;
		EventArgs args;
		args.SetValue("KeyCode", Stringf("%d", asKey));
		FireEvent("KEYPRESSED", args);

		return 0;
	}

	case WM_LBUTTONUP:
	{
		unsigned char asKey = KEYCODE_LEFT_MOUSE;
		EventArgs args;
		args.SetValue("KeyCode", Stringf("%d", asKey));
		FireEvent("KEYRELEASED", args);

		return 0;
	}

	case WM_RBUTTONDOWN:
	{
		unsigned char asKey = KEYCODE_RIGHT_MOUSE;
		EventArgs args;
		args.SetValue("KeyCode", Stringf("%d", asKey));
		FireEvent("KEYPRESSED", args);

		return 0;
	}

	case WM_RBUTTONUP:
	{
		unsigned char asKey = KEYCODE_RIGHT_MOUSE;
		EventArgs args;
		args.SetValue("KeyCode", Stringf("%d", asKey));
		FireEvent("KEYRELEASED", args);

		return 0;
	}
	}

	// Send back to Windows any unhandled/unconsumed messages we want other apps to see (e.g. play/pause in music apps, etc.)
	return DefWindowProc(windowHandle, wmMessageCode, wParam, lParam);
}

Window::Window(WindowConfig const& config)
	: m_config(config)
{
	s_mainWindow = this;
}

Window::~Window()
{
}

void Window::StartUp()
{
	CreateOSWindow();
}

void Window::BeginFrame()
{
	RunMessagePump();
}

void Window::EndFrame()
{
}

void Window::ShutDown()
{
}

WindowConfig const& Window::GetConfig() const
{
	return m_config;
}

Window* Window::GetWindowContext()
{
	return s_mainWindow;
}

bool Window::GetFileName(std::string& fileName)
{
	int directorySize = MAX_PATH;
	//char* directoryPtr = nullptr;

	char directoryPath[MAX_PATH];
	directoryPath[0] = '\0';

	GetCurrentDirectoryA(directorySize, directoryPath);

	char filename[MAX_PATH];
	filename[0] = '\0';

	char fileDirectoryPath[MAX_PATH];
	fileDirectoryPath[0] = '\0';

	OPENFILENAMEA data = {};
	data.lStructSize = sizeof(data);
	data.lpstrFile = filename;
	data.nMaxFile = sizeof(filename);
	data.lpstrFilter = "All\0*.*\0";
	data.nFilterIndex = 1;
	data.lpstrInitialDir = fileDirectoryPath;
	data.hwndOwner = (HWND)GetHwnd();
	data.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

	bool result = GetOpenFileNameA(&data);
	
	SetCurrentDirectoryA(directoryPath);

	fileName = filename;
	
	return result;
}

void Window::CreateOSWindow()
{
	WNDCLASSEX windowClassDescription;
	memset(&windowClassDescription, 0, sizeof(windowClassDescription));
	windowClassDescription.cbSize = sizeof(windowClassDescription);
	windowClassDescription.style = CS_OWNDC; 
	windowClassDescription.lpfnWndProc = static_cast<WNDPROC>(WindowsMessageHandlingProcedure); 
	windowClassDescription.hInstance = GetModuleHandle(NULL);
	windowClassDescription.hIcon = NULL;
	windowClassDescription.hCursor = NULL;
	windowClassDescription.lpszClassName = TEXT("Simple Window Class");
	RegisterClassEx(&windowClassDescription);

	DWORD windowStyleFlags = WS_CAPTION | WS_BORDER | WS_THICKFRAME | WS_SYSMENU | WS_OVERLAPPED;
	
	RECT clientRect;

	if (m_config.m_size.x != -1 && m_config.m_size.y != -1)
	{
		clientRect.left = 0;
		clientRect.top = 0;
		clientRect.right = m_config.m_size.x;
		clientRect.bottom = m_config.m_size.y;
	}
	else
	{
		clientRect.left = 0;
		clientRect.top = 0;
		clientRect.right = 1600;
		clientRect.bottom = 800;
	}

	if (m_config.m_isFullScreen)
	{
		windowStyleFlags = WS_POPUP | WS_VISIBLE;

		clientRect.left = 0;
		clientRect.top = 0;
		clientRect.right = GetSystemMetrics(SM_CXSCREEN);
		clientRect.bottom = GetSystemMetrics(SM_CYSCREEN);

		m_config.m_clientAspect = (float)clientRect.right / (float)clientRect.bottom;
	}
	else
	{
		if (m_config.m_size.x == -1 && m_config.m_size.y != -1)
		{
			int adjustWidth = static_cast<int>((float)m_config.m_size.y * m_config.m_clientAspect);

			clientRect.right = clientRect.left + adjustWidth;
		}
		else if (m_config.m_size.x != -1 && m_config.m_size.y == -1)
		{
			int adjustHeight = static_cast<int>((float)m_config.m_size.x / m_config.m_clientAspect);

			clientRect.bottom = clientRect.top+ adjustHeight;
		}
		
		if (m_config.m_pos.x >= 0 && m_config.m_pos.y >= 0)
		{
			clientRect.left += m_config.m_pos.x;
			clientRect.top += m_config.m_pos.y;
			clientRect.right += m_config.m_pos.x;
			clientRect.bottom += m_config.m_pos.y;
		}
		else
		{
			// Default center position if not specified
			int screenWidth = GetSystemMetrics(SM_CXSCREEN);
			int screenHeight = GetSystemMetrics(SM_CYSCREEN);
			clientRect.left = (screenWidth - (clientRect.right - clientRect.left)) / 2;
			clientRect.top = (screenHeight - (clientRect.bottom - clientRect.top)) / 2;
			clientRect.right = clientRect.left + (clientRect.right - clientRect.left);
			clientRect.bottom = clientRect.top + (clientRect.bottom - clientRect.top);
		}
	}

	AdjustWindowRectEx(&clientRect, windowStyleFlags, FALSE, WS_EX_APPWINDOW);

	WCHAR windowTitle[1024];
	//LPCCH title = LPCCH(m_config.m_windowTitle);
	MultiByteToWideChar(GetACP(), 0, m_config.m_windowTitle.c_str(), -1, windowTitle, sizeof(windowTitle) / sizeof(windowTitle[0]));
	m_hWnd = CreateWindowEx(
		WS_EX_APPWINDOW,
		windowClassDescription.lpszClassName,
		windowTitle,
		windowStyleFlags,
		clientRect.left,
		clientRect.top,
		clientRect.right - clientRect.left,
		clientRect.bottom - clientRect.top,
		NULL,
		NULL,
		GetModuleHandle(NULL),
		NULL);

	ShowWindow(reinterpret_cast<HWND>(m_hWnd), SW_SHOW);
	SetForegroundWindow(reinterpret_cast<HWND>(m_hWnd));
	SetFocus(reinterpret_cast<HWND>(m_hWnd));
	UpdateWindow(reinterpret_cast<HWND>(m_hWnd));

	m_dc = GetDC(reinterpret_cast<HWND>(m_hWnd));

	RECT finalRect;
	GetClientRect(reinterpret_cast<HWND>(m_hWnd), &finalRect);

	m_clientDimensions = IntVec2(finalRect.right - finalRect.left, finalRect.bottom - finalRect.top);
	HCURSOR cursor = LoadCursor(NULL, IDC_ARROW);
	SetCursor(cursor);
}

void* Window::GetDeviceContext() const
{
	return m_dc;
}

void Window::RunMessagePump()
{
	MSG queuedMessage;
	for (;; )
	{
		const BOOL wasMessagePresent = PeekMessage(&queuedMessage, NULL, 0, 0, PM_REMOVE);
		if (!wasMessagePresent)
		{
			break;
		}

		TranslateMessage(&queuedMessage);
		DispatchMessage(&queuedMessage); 
	}
}

float Window::GetAspect() const
{
	return m_config.m_clientAspect;
}

void* Window::GetHwnd() const
{
	return m_hWnd;
}

IntVec2 Window::GetClientDimensions() const
{
	return m_clientDimensions;
}
