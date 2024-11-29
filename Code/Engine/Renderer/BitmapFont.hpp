#pragma once

#include "Engine/Core/Rgba8.hpp"
#include "Engine/Renderer/SpriteSheet.hpp"

#include <vector>
#include <string>

struct Vec2;
struct Vertex_PCU;
struct Rgba8;
class Texture;

enum TextBoxMode
{
	SHRINK_TO_FIT,
	OVERRUN,

	TOTAL_MODES
};

class BitmapFont
{
protected:
	std::string	m_fontFilePathNameWithNoExtension;
	SpriteSheet*	m_fontGlyphsSpriteSheet;

private:
	BitmapFont(char const* fontFilePathNameWithNoExtension, Texture& fontTexture);
public:
	~BitmapFont();
	Texture& GetTexture();

	void AddVertsForText2D(std::vector<Vertex_PCU>& vertexArray, Vec2& textMins,
		float cellHeight, std::string const& text, Rgba8 const& tint = Rgba8::WHITE, float cellAspect = 1.f);

	void AddVertsForTextInBox2D(std::vector<Vertex_PCU>& vertexArray, AABB2 const& box, float cellHeight,
		std::string const& text, Rgba8 const& tint = Rgba8::WHITE, float cellAspect = 1.f,
		Vec2 const& alignment = Vec2(.5f, .5f), TextBoxMode mode = TextBoxMode::SHRINK_TO_FIT, int maxGlyphsToDraw = 99999999);

	void AddVertsForText3DAtOriginXForward(std::vector<Vertex_PCU>& verts, float cellHeight, std::string const& text,
		Rgba8 const& tint = Rgba8::WHITE, float cellAspect = 1.0f, Vec2 const& alingment = Vec2(0.5f, 0.5f), int maxGlyphsToDraw = 999999999);

	float GetTextWidth(float cellHeight, std::string const& text, float cellAspect = 1.f);

protected:
	float GetGlyphAspect(int glyphUnicode) const;

private:
	friend class Renderer;
	friend class DX11Renderer;
	friend class DX12Renderer;
};