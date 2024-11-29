#pragma once

#include "Engine/Math/IntVec2.hpp"
#include "Engine/Renderer/SpriteDefinition.hpp"

#include <vector>

struct AABB2;
struct Vec2;
class Texture;

class SpriteSheet
{
protected:
	Texture&						m_texture;
	IntVec2							m_dimensions;
public:
	std::vector<SpriteDefinition>	m_spriteDefs;
	explicit						SpriteSheet(Texture& texture, IntVec2 const& simpleGridlayout);
	~SpriteSheet() = default;

	void							CalculateUVsOfSpriteSheet();

	Texture&						GetTexture() const;
	int								GetNumSprites() const;
	IntVec2							GetSheetDimensions() const;
	SpriteDefinition const&			GetSpriteDef(int spriteIndex) const;
	void							GetSpriteUVs(Vec2& out_uvAtMins, Vec2& out_uv_AtMaxs, int spriteIndex) const;
	AABB2							GetSpriteUVs(int spriteIndex) const;
};