#include "Engine/Core/DevConsole.hpp"

#include "Engine/Renderer/BitmapFont.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/EventSystem.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/Timer.hpp"
#include "Engine/Core/Clock.hpp"

#include <algorithm>

extern Renderer* g_theRenderer;

DevConsole* g_theConsole = nullptr;

Rgba8 const DevConsole::ERROR = Rgba8(255, 0, 0, 255);
Rgba8 const DevConsole::WARNING = Rgba8(255, 255, 0, 255);
Rgba8 const DevConsole::INFO_MAJOR = Rgba8(0, 255, 0, 255);
Rgba8 const DevConsole::INFO_MINOR = Rgba8(255, 165, 0, 255);
Rgba8 const DevConsole::INPUT_TEXT = Rgba8(255, 223, 0, 255);
Rgba8 const DevConsole::COMMAND_ECHO = Rgba8(255, 255, 255, 255);
Rgba8 const DevConsole::COMMAND_REMOTE_ECHO = Rgba8(255, 0, 255, 255);

DevConsole::DevConsole(DevConsoleConfig const& config)
	: m_config(config)
{
}

DevConsole::~DevConsole()
{
}

void DevConsole::StartUp()
{
	m_insertionPointBlinkTimer = new Timer(0.5f, &Clock::GetSystemClock());
	m_insertionPointBlinkTimer->Start();

	SubscribeEventCallbackFunction("KEYPRESSED", DevConsole::Event_KeyPressed);
	SubscribeEventCallbackFunction("CHARINPUT", DevConsole::Event_CharInput);
	SubscribeEventCallbackFunction("ECHO", DevConsole::Command_Echo);
	SubscribeEventCallbackFunction("CLEAR", DevConsole::Command_Clear);
	SubscribeEventCallbackFunction("HELP", DevConsole::Command_Help);

	DevConsoleLine line = DevConsoleLine("Type HELP to get a list of commands", DevConsole::COMMAND_REMOTE_ECHO);
	m_lines.push_back(line);
}

void DevConsole::ShutDown()
{
	m_insertionPointBlinkTimer->Stop();
}

void DevConsole::BeginFrame()
{
	m_frameNumber++;

	while (m_insertionPointBlinkTimer->DecrementPeriodIfElapsed())
	{
		m_insertionPointVisible = !m_insertionPointVisible;
	}
}

void DevConsole::EndFrame()
{
}

void DevConsole::Execute(std::string const& consoleCommandText, bool echoCommand)
{
	// Split text into individual lines
	std::vector<std::string> commandLines = SplitStringOnDelimiter(consoleCommandText, '\n');

	// Loop through each command line, process each argument first and then fire 
	for (const std::string& commandLine : commandLines)
	{
		// Split quotes & command name and parts 
		std::vector<std::string> commandQuotes = SplitStringWithQuotes(commandLine, ' ');

		if (!commandQuotes.empty())
		{
			std::string commandName = commandQuotes[0];
			if (commandQuotes.size() > 1) // Only proceed if there are additional arguments
			{
				// Loop through each argument and split into key and value
				EventArgs args;
				for (size_t argumentIndex = 1; argumentIndex < commandQuotes.size(); argumentIndex++)
				{
					std::transform(commandName.begin(), commandName.end(), commandName.begin(), [](unsigned char c) -> unsigned char { return (unsigned char)std::toupper(c); });
					if (commandName == "ECHO")
					{
						std::vector<std::string> argPairs = SplitStringWithQuotes(commandQuotes[argumentIndex], '=');
						if (argPairs.size() < 2)
						{
							AddLine(ERROR, "Arguments expect a name value that has a '=' sign to it ");
							return;
						}

						std::transform(argPairs[0].begin(), argPairs[0].end(), argPairs[0].begin(), [](unsigned char c) -> unsigned char { return (unsigned char)std::toupper(c); });
						args.SetValue(argPairs[0], argPairs[argumentIndex]);
					}
					else
					{
						std::vector<std::string> argPairs = SplitStringWithQuotes(commandQuotes[argumentIndex], '=');
						if (argPairs.size() < 2)
						{
							AddLine(ERROR, "Arguments expect a name value that has a '=' sign to it ");
							return;
						}
						args.SetValue(argPairs[0], argPairs[argumentIndex]);
					}
				}
				FireEvent(commandName, args);
			}
			else
			{
				FireEvent(commandName);
			}
		}

		if (echoCommand)
		{
			g_theConsole->AddLine(COMMAND_ECHO, commandLine);
		}
	}
}

void DevConsole::AddLine(Rgba8 const& color, std::string const& text)
{
	DevConsoleLine line = DevConsoleLine(text, color);

	line.m_displayFrame = m_frameNumber;
	line.m_displayTime = Clock::GetSystemClock().GetTotalSeconds();
	std::string lineText = Stringf("%.2f", line.m_displayTime) + "s (Frame- " + std::to_string(line.m_displayFrame) + ") : " + line.m_lineText;
	line.m_lineText = lineText;

	m_lines.push_back(line);
}

void DevConsole::Render(AABB2 const& bounds, Renderer* rendererOverride) const
{
	BitmapFont* font = rendererOverride->CreateOrGetBitmapFont(m_config.m_fontFilePath.c_str());

	Render_OpenFull(bounds, *rendererOverride, *font);
}

void DevConsole::ToggleOpen()
{
	m_isOpen = !m_isOpen;

	if (m_isOpen)
	{
		m_insertionPointBlinkTimer->Start();
	}
	else
	{
		m_insertionPointBlinkTimer->Stop();
	}
}

bool DevConsole::IsOpen()
{
	return m_isOpen;
}

DevConsoleMode DevConsole::GetMode() const
{
	return m_mode;
}

void DevConsole::SetMode(DevConsoleMode mode)
{
	m_mode = mode;
}

void DevConsole::ToggleMode(DevConsoleMode mode)
{
	if(m_mode == mode)
		m_mode = DevConsoleMode::HIDDEN;
	else
		m_mode = mode;
}

bool DevConsole::Event_KeyPressed(EventArgs& args)
{
	if(g_theConsole->IsOpen())
	{
		unsigned char keyCode = (unsigned char)args.GetValue("KeyCode", -1);

		if (keyCode == KEYCODE_ENTER)
		{
			if (g_theConsole->m_inputText.size() > 0)
			{
				Strings text = SplitStringOnDelimiter(g_theConsole->m_inputText, ' ');

				if (text[0] == "ECHO")
				{
					g_theConsole->Execute(g_theConsole->m_inputText, true);
					g_theConsole->m_commandHistory.push_back(g_theConsole->m_inputText);
					g_theConsole->m_historyIndex = (int)g_theConsole->m_commandHistory.size();
					g_theConsole->m_cliCursor.clear();
					g_theConsole->m_cliCursor = "|";
					g_theConsole->m_inputText.clear();
					g_theConsole->m_insertionPointPosition = 0;
					return true;
				}

				g_theConsole->Execute(g_theConsole->m_inputText);
				g_theConsole->m_commandHistory.push_back(g_theConsole->m_inputText);
				g_theConsole->m_historyIndex = (int)g_theConsole->m_commandHistory.size();
				g_theConsole->m_cliCursor.clear();
				g_theConsole->m_cliCursor = "|";
				g_theConsole->m_inputText.clear();
				g_theConsole->m_insertionPointPosition = 0;
				return true;
			}
			else
			{
				g_theConsole->m_isOpen = false;

				return true;
			}
		}

		if (keyCode == KEYCODE_ESC)
		{
			if (g_theConsole->m_inputText.size() > 0)
			{
				g_theConsole->m_cliCursor.clear();
				g_theConsole->m_cliCursor = "|";
				g_theConsole->m_inputText.clear();
				return true;
			}
			else
			{
				g_theConsole->m_isOpen = false;

				return true;
			}
		}

		if (keyCode == KEYCODE_BACKSPACE && g_theConsole->m_inputText.size() > 0)
		{
			g_theConsole->m_inputText.erase(g_theConsole->m_inputText.begin() + g_theConsole->m_insertionPointPosition - 1);
			g_theConsole->m_cliCursor[g_theConsole->m_cliCursor.size() - 2] = g_theConsole->m_cliCursor[g_theConsole->m_cliCursor.size() - 1];
			g_theConsole->m_cliCursor.erase(g_theConsole->m_cliCursor.begin() + g_theConsole->m_cliCursor.size() - 1);
			g_theConsole->m_insertionPointPosition--;
			return true;
		}

		if (keyCode == KEYCODE_DELETE && g_theConsole->m_inputText.size() > 0)
		{
			g_theConsole->m_inputText.erase(g_theConsole->m_inputText.begin() + g_theConsole->m_insertionPointPosition);
			return true;
		}

		if (keyCode == KEYCODE_UPARROW && g_theConsole->m_historyIndex > 0)
		{
			g_theConsole->m_historyIndex--;
			g_theConsole->m_inputText = g_theConsole->m_commandHistory[g_theConsole->m_historyIndex];
		}

		if (keyCode == KEYCODE_DOWNARROW && g_theConsole->m_historyIndex < (int)g_theConsole->m_commandHistory.size() - 1)
		{
			g_theConsole->m_historyIndex++;
			g_theConsole->m_inputText = g_theConsole->m_commandHistory[g_theConsole->m_historyIndex];
		}

		if (keyCode == KEYCODE_LEFTARROW && g_theConsole->m_cliCursor.size() > 1)
		{
			g_theConsole->m_cliCursor.pop_back();
			g_theConsole->m_cliCursor.pop_back();
			g_theConsole->m_cliCursor.push_back('|');
			g_theConsole->m_insertionPointPosition--;
		}

		if (keyCode == KEYCODE_RIGHTARROW && g_theConsole->m_cliCursor.size() <= g_theConsole->m_inputText.size())
		{
			g_theConsole->m_cliCursor.pop_back();
			g_theConsole->m_cliCursor.push_back(' ');
			g_theConsole->m_cliCursor.push_back('|');
			g_theConsole->m_insertionPointPosition++;
		}

		if (keyCode == KEYCODE_HOME)
		{
			g_theConsole->m_cliCursor.clear();
			g_theConsole->m_cliCursor = "|";
			g_theConsole->m_insertionPointPosition = 0;
		}

		if (keyCode == KEYCODE_END)
		{
			g_theConsole->m_cliCursor.clear();
			
			for (size_t index = 0; index < g_theConsole->m_inputText.size(); index++)
			{
				g_theConsole->m_cliCursor.push_back(' ');
			}

			g_theConsole->m_cliCursor.push_back('|');

			g_theConsole->m_insertionPointPosition = (int)g_theConsole->m_cliCursor.size() - 1;
		}
	}

	return false;
}

bool DevConsole::Event_CharInput(EventArgs& args)
{
	if (g_theConsole->IsOpen())
	{
		unsigned char keyCode = (unsigned char)args.GetValue("KeyCode", -1);

		if (keyCode == -1)
			return false;

		if (keyCode != 96 && keyCode != KEYCODE_ENTER && keyCode != KEYCODE_BACKSPACE && keyCode != KEYCODE_ESC)
		{
			
			g_theConsole->m_inputText.insert(g_theConsole->m_inputText.begin() + g_theConsole->m_insertionPointPosition, keyCode);
			g_theConsole->m_cliCursor = ' ' + g_theConsole->m_cliCursor;
			g_theConsole->m_insertionPointPosition++;
		}
	}

	return true;
}

bool DevConsole::Command_Clear(EventArgs& args)
{
	UNUSED(args);

	if (g_theConsole->IsOpen())
	{
		if (g_theConsole->m_lines.size() == 0)
			return true;

		g_theConsole->m_lines.clear();
	}

	return false;
}

bool DevConsole::Command_Echo(EventArgs& args)
{
	if (args.HasArgument("MESSAGE"))
	{
		std::string message = args.GetValue("MESSAGE", "");
		std::string echoMessage = "Echo Message=" + message;
		g_theConsole->AddLine(DevConsole::INFO_MINOR, echoMessage);
		g_theConsole->AddLine(DevConsole::INFO_MAJOR, message);
	}
	else
	{
		// Handle the case where the "message" argument is missing
		g_theConsole->AddLine(DevConsole::ERROR, "Error: Arguments are missing or incorrect");
	}
	return true;
}

bool DevConsole::Command_Help(EventArgs& args)
{
	UNUSED(args);

	if (g_theConsole->IsOpen())
	{
		Strings commands;

		commands = g_theEventSystem->GetAllCommands();

		DevConsoleLine line = DevConsoleLine("----------LIST OF ALL COMMANDS----------", DevConsole::INFO_MAJOR);
		g_theConsole->m_lines.push_back(line);

		line = DevConsoleLine("", Rgba8());
		g_theConsole->m_lines.push_back(line);

		for (size_t index = 0; index < commands.size(); index++)
		{
			if (commands[index] != "CHARINPUT" && commands[index] != "KEYRELEASED" && commands[index] != "KEYPRESSED")
			{
				line = DevConsoleLine(commands[index], DevConsole::INFO_MINOR);
				g_theConsole->m_lines.push_back(line);
			}
		}
		
		line = DevConsoleLine("", Rgba8());
		g_theConsole->m_lines.push_back(line);
		
		line = DevConsoleLine("----------------------------------------", DevConsole::INFO_MAJOR);
		g_theConsole->m_lines.push_back(line);
		
		line = DevConsoleLine("", Rgba8());
		g_theConsole->m_lines.push_back(line);
	}

	return true;
}

void DevConsole::Render_OpenFull(AABB2 const& bounds, Renderer& renderer, BitmapFont& font, float fontAspect) const
{
	UNUSED(fontAspect);

	std::vector<Vertex_PCU> consoleVerts;

	AddVertsForAABB2D(consoleVerts, bounds, Rgba8(0, 0, 0, 127));

	std::vector<Vertex_PCU> textVerts;
	float cellHeight = bounds.GetDimensions().y / m_config.m_numOfLinesOnScreen;
	
	AddVertsForAABB2D(consoleVerts, AABB2(bounds.m_mins, Vec2(bounds.m_maxs.x, cellHeight + (bounds.m_maxs.y * 0.0125f))), Rgba8(0, 0, 0, 255));

	for (size_t index = 0; index < m_lines.size(); index++)
	{
		Vec2 linePosition = Vec2(0.0f, cellHeight * (m_lines.size() - index) + (bounds.m_maxs.x * 0.0125f));

		AABB2 lineBox = AABB2((bounds.m_maxs.x * 0.0125f), linePosition.y, bounds.m_maxs.x, linePosition.y + cellHeight + (bounds.m_maxs.y * 0.0125f));

		font.AddVertsForTextInBox2D(textVerts, lineBox, cellHeight - (bounds.m_maxs.y * 0.00625f), m_lines[index].m_lineText, m_lines[index].m_color, 0.75f, Vec2(0.0f, 0.5f));
	}

	font.AddVertsForTextInBox2D(textVerts, AABB2(Vec2((bounds.m_maxs.x * 0.0125f), 0.0f), Vec2(bounds.m_maxs.x, cellHeight + (bounds.m_maxs.y * 0.0125f))), cellHeight - (bounds.m_maxs.y * 0.00625f), m_inputText, g_theConsole->INPUT_TEXT, 0.75f, Vec2(0.0f, 0.5f));

	std::vector<Vertex_PCU> cursorVerts;
	
	font.AddVertsForTextInBox2D(cursorVerts, AABB2(Vec2((bounds.m_maxs.x * 0.0125f), 0.0f), Vec2(bounds.m_maxs.x, cellHeight + (bounds.m_maxs.y * 0.0125f))), cellHeight - (bounds.m_maxs.y * 0.00625f), m_cliCursor, Rgba8::WHITE, 0.75f, Vec2(-0.005f, 0.5f));

	AddVertsForLineSegment2D(consoleVerts, Vec2(0.0f, (cellHeight + (bounds.m_maxs.y * 0.0125f))), Vec2(bounds.m_maxs.x, (cellHeight + (bounds.m_maxs.y * 0.0125f))), (bounds.m_maxs.y * 0.003125f), Rgba8(0, 255, 255, 225));
	AddVertsForLineSegment2D(consoleVerts, Vec2(0.0f, 0.5f), Vec2(bounds.m_maxs.x, 0.5f), (bounds.m_maxs.y * 0.003125f), Rgba8(0, 255, 255, 225));
	AddVertsForLineSegment2D(consoleVerts, Vec2(0.5f, 0.0f), Vec2(0.5f, (cellHeight + (bounds.m_maxs.y * 0.0125f))), (bounds.m_maxs.y * 0.003125f), Rgba8(0, 255, 255, 225));
	AddVertsForLineSegment2D(consoleVerts, Vec2(bounds.m_maxs.x - 0.5f, 0.0f), Vec2(bounds.m_maxs.x - 0.5f, (cellHeight + (bounds.m_maxs.y * 0.0125f))), (bounds.m_maxs.y * 0.003125f), Rgba8(0, 255, 255, 225));

	Mat44 modelMatrix;

	renderer.SetModelConstants(RootSig::DEFAULT_PIPELINE, modelMatrix, Rgba8::WHITE);

	renderer.SetBlendMode(BlendMode::ALPHA);
	renderer.SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);
	renderer.BindTexture();

	renderer.DrawVertexArray(static_cast<int>(consoleVerts.size()), consoleVerts.data());

	if (m_insertionPointVisible)
	{
		renderer.SetBlendMode(BlendMode::ALPHA);
		renderer.SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);
		renderer.BindTexture(0, &font.GetTexture());
		renderer.DrawVertexArray(static_cast<int>(cursorVerts.size()), cursorVerts.data());
	}

	renderer.SetBlendMode(BlendMode::ALPHA);
	renderer.SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);
	renderer.BindTexture(0, &font.GetTexture());
	renderer.DrawVertexArray(static_cast<int>(textVerts.size()), textVerts.data());
}

DevConsoleLine::DevConsoleLine(std::string const& text, Rgba8 const& color)
	: m_lineText(text), m_color(color)
{
}