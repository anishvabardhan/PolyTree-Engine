#pragma once

#include <vector>
#include <string>

#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/EventSystem.hpp"

class Camera;
class Timer;

enum DevConsoleMode
{
	VISIBLE,
	HIDDEN,

	TOTAL
};

struct DevConsoleLine
{
	std::string		m_lineText;
	Rgba8			m_color;
	int				m_displayFrame;
	double			m_displayTime;
public:
	DevConsoleLine(std::string const& text, Rgba8 const& color);
};

struct DevConsoleConfig
{
	Renderer*				m_renderer = nullptr;
	Camera*					m_camera = nullptr;
	std::string				m_fontFilePath;
	float					m_fontAspect = 1.0f;
	float					m_numOfLinesOnScreen =30.5f;
	int						m_maxCommandHistory = 128;
	bool					m_startOpen = false;
};

class DevConsole
{
	DevConsoleConfig				m_config;
	DevConsoleMode					m_mode = DevConsoleMode::HIDDEN;
	int								m_frameNumber = 0;
	bool							m_isOpen = false;
	std::string						m_inputText;
	std::string						m_cliCursor = "|";
	int								m_insertionPointPosition = 0;
	bool							m_insertionPointVisible = true;
	Timer*							m_insertionPointBlinkTimer = nullptr;
	std::vector<std::string>		m_commandHistory;
	int								m_historyIndex = -1;
public:
	std::vector<DevConsoleLine>		m_lines;
public:
	DevConsole(DevConsoleConfig const& config);
	~DevConsole();

	void StartUp();
	void ShutDown();
	void BeginFrame();
	void EndFrame();

	void Execute(std::string const& consoleCommandText, bool echoCommand = false);
	void AddLine(Rgba8 const& color, std::string const& text);
	void Render(AABB2 const& bounds, Renderer* rendererOverride = nullptr) const;

	void ToggleOpen();
	bool IsOpen();

	DevConsoleMode GetMode() const;
	void SetMode(DevConsoleMode mode);
	void ToggleMode(DevConsoleMode mode);

	static const Rgba8 ERROR;
	static const Rgba8 WARNING;
	static const Rgba8 INFO_MAJOR;
	static const Rgba8 INFO_MINOR;
	static const Rgba8 COMMAND_ECHO;
	static const Rgba8 COMMAND_REMOTE_ECHO;
	static const Rgba8 INPUT_TEXT;
	static const Rgba8 INPUT_INSERTION_POINT;

	static bool Event_KeyPressed(EventArgs& args);
	static bool Event_CharInput(EventArgs& args);
	static bool Command_Clear(EventArgs& args);
	static bool Command_Echo(EventArgs& args);
	static bool Command_Help(EventArgs& args);
protected:
	void Render_OpenFull(AABB2 const& bounds, Renderer& renderer, BitmapFont& font, float fontAspect = 1.0f) const;
};