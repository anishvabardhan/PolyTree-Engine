#pragma once

#include "Engine/Math/Vec2.hpp"

struct AABB2;
class Texture;
class SpriteSheet;

class SpriteDefinition
{
protected:
	SpriteSheet const&	m_spriteSheet;
	int					m_spriteIndex = -1;
	Vec2				m_uvAtMins = Vec2::ZERO;
	Vec2				m_uvAtMaxs = Vec2::ONE;
public:
	explicit			SpriteDefinition(SpriteSheet const& spriteSheet, int spriteIndex, Vec2 const& uvAtMins, Vec2 const& uvAtMaxs);
	~SpriteDefinition() {}

	void				GetUVs(Vec2& out_uvAtMins, Vec2& out_uvAtMaxs) const;
	AABB2				GetUVs() const;
	SpriteSheet const&	GetSpriteSheet() const;
	Texture&			GetTexture() const;
	float				GetAspect() const;
};
