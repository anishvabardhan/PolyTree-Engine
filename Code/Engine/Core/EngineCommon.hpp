#pragma once

#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Core/DevConsole.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Core/EventSystem.hpp"
#include "Engine/Core/NamedStrings.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Window/Window.hpp"
#include "Engine/Input/InputSystem.hpp"
#include "Engine/Renderer/Renderer.hpp"

#define UNUSED(x) (void)x
#define DELETE_PTR(x) if(x) { delete x; x = nullptr; }

#pragma warning(disable:26812)

extern Renderer* g_theRenderer;
extern NamedStrings g_defaultConfigBlackboard;
extern NamedStrings g_gameConfigBlackboard;
extern DevConsole* g_theConsole;
extern EventSystem* g_theEventSystem;
extern Window* g_theWindow;