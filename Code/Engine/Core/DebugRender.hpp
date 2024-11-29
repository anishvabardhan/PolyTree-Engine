#pragma once

#include "Engine/Core/Rgba8.hpp"
#include "Engine/Core/EventSystem.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Renderer/Renderer.hpp"

#include <string>

class Renderer;
class Camera;
class Timer;
struct Vec3;
struct Vec2;
struct Mat44;

struct DebugRenderConfig
{
	Renderer*					m_renderer = nullptr;
	std::string					m_fontName = "Data/Fonts/SquirrelFixedFont.png";
};

enum class DebugRenderMode
{
	ALWAYS,
	USE_DEPTH,
	X_RAY,
	COUNT
};

void DebugRenderSystemStartup(DebugRenderConfig const& config);
void DebugRenderSystemShutdown();

void DebugRenderSetVisible();
void DebugRenderSetHidden();
void DebugRenderClear();

void DebugRenderBeginFrame();
void DebugRenderWorld(Camera& camera);
void DebugRenderScreen(Camera& camera);
void DebugRenderEndFrame();

void DebugAddWorldPoint(Vec3 const& pos, float radius, float duration, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddWorldLine(Vec3 const& start, Vec3 const& end, float radius, float duration, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddWorldWireCylinder(Vec3 const& base, Vec3 const& top, float radius, float duration, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddWorldWireSphere(Vec3 const& center, float radius, float duration, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddWorldArrow(Vec3 const& start, Vec3 const& end, float radius, float duration, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddWorldWireArrow(Vec3 const& start, Vec3 const& end, float radius, float duration, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddWorldBasis(Mat44 const& transform, float duration, float length, float radius, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddWorldText(std::string const& text, Mat44 const& transform, float textHeight, Vec2 const& alignment = Vec2(.5f, .5f), float duration = -1, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddWorldBillboardText(std::string const& text, Mat44 const& transform, float textHeight, Vec2 const& alignment = Vec2(.5f, .5f), float duration = -1, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddScreenText(std::string const& text, Mat44 const& transform, float textHeight, Vec2 const& alignment = Vec2(.5f, .5f), float duration = -1, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
void DebugAddMessage(std::string const& text, Vec3 const& screenPosition, float textHeight, Vec2 const& alignment = Vec2(.5f, .5f), float duration = -1, Rgba8 const& startColor = Rgba8::WHITE, Rgba8 const& endColor = Rgba8::WHITE, DebugRenderMode mode = DebugRenderMode::USE_DEPTH);
bool Command_DebugRenderClear(EventArgs& args);
bool Command_DebugRenderToggle(EventArgs& args);