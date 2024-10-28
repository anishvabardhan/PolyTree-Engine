#include "Engine/Renderer/SpriteSheet.hpp"

#include "Engine/Renderer/Texture.hpp"
#include "Engine/Renderer/SpriteDefinition.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/Vec2.hpp"

SpriteSheet::SpriteSheet(Texture& texture, IntVec2 const& simpleGridlayout)
	: m_texture(texture), m_dimensions(simpleGridlayout)
{
	CalculateUVsOfSpriteSheet();
}

void SpriteSheet::CalculateUVsOfSpriteSheet()
{
	int numOfTotalSprites = m_dimensions.x * m_dimensions.y;

	for (int index = 0; index < numOfTotalSprites; index++)
	{
		int tileX = index % m_dimensions.x;
		int tileY = index / m_dimensions.x;

		if (m_dimensions.x >= m_dimensions.y)
		{
			if (m_dimensions.y > 1)
			{
				tileY = m_dimensions.x - 1 - tileY;
			}
		}

		AABB2 uvBounds;

		float unitX = static_cast<float>(1.0f / static_cast<float>(m_dimensions.x));
		float unitY = static_cast<float>(1.0f / static_cast<float>(m_dimensions.y));

		uvBounds = AABB2(Vec2(unitX * tileX, unitY * tileY), Vec2(unitX * (tileX + 1), unitY * (tileY + 1)));

		float nudgeValueX = static_cast<float>(1.0f / 128.0f) * uvBounds.GetDimensions().x;
		float nudgeValueY = static_cast<float>(1.0f / 128.0f) * uvBounds.GetDimensions().y;
		
		uvBounds.m_mins.x += nudgeValueX;
		uvBounds.m_mins.y += nudgeValueY;
		
		uvBounds.m_maxs.x -= nudgeValueX;
		uvBounds.m_maxs.y -= nudgeValueY;

		SpriteDefinition sprite = SpriteDefinition(*this, index, uvBounds.m_mins, uvBounds.m_maxs);

		m_spriteDefs.push_back(sprite);
	}
}

Texture& SpriteSheet::GetTexture() const
{
	return m_texture;
}

int SpriteSheet::GetNumSprites() const
{
	return static_cast<int>(m_spriteDefs.size());
}

IntVec2 SpriteSheet::GetSheetDimensions() const
{
	return m_dimensions;
}

SpriteDefinition const& SpriteSheet::GetSpriteDef(int spriteIndex) const
{
	return m_spriteDefs[spriteIndex];
}

void SpriteSheet::GetSpriteUVs(Vec2& out_uvAtMins, Vec2& out_uv_AtMaxs, int spriteIndex) const
{
	m_spriteDefs[spriteIndex].GetUVs(out_uvAtMins, out_uv_AtMaxs);
}

AABB2 SpriteSheet::GetSpriteUVs(int spriteIndex) const
{
	return m_spriteDefs[spriteIndex].GetUVs();
}
