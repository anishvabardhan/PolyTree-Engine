#include "Engine/Renderer/BitmapFont.hpp"

#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Renderer/SpriteSheet.hpp"
#include "Engine/Core/EngineCommon.hpp"

BitmapFont::BitmapFont(char const* fontFilePathNameWithNoExtension, Texture& fontTexture)
	: m_fontFilePathNameWithNoExtension(fontFilePathNameWithNoExtension)
{
	m_fontGlyphsSpriteSheet = new SpriteSheet(fontTexture, IntVec2(16, 16));
}

BitmapFont::~BitmapFont()
{
	DELETE_PTR(m_fontGlyphsSpriteSheet);
}

Texture& BitmapFont::GetTexture()
{
	return m_fontGlyphsSpriteSheet->GetTexture();
}

void BitmapFont::AddVertsForText2D(std::vector<Vertex_PCU>& vertexArray, Vec2& textMins, float cellHeight, std::string const& text, Rgba8 const& tint, float cellAspect)
{
	float cellWidth = cellHeight * cellAspect;

	for (size_t index = 0; index < text.size(); index++)
	{
		AABB2 bounds = AABB2(textMins.x + (cellWidth * index), textMins.y, textMins.x + (cellWidth * (index + 1)), textMins.y + cellHeight);

		SpriteDefinition const& testSpriteDef = m_fontGlyphsSpriteSheet->GetSpriteDef(text[index]);

		Vec2 uvMins, uvMaxs;
		testSpriteDef.GetUVs(uvMins, uvMaxs);

		if (text[index] == '\n')
		{
			textMins.x = -1.0f * (cellWidth * index);
			textMins.y -= cellHeight;
		}

		AddVertsForAABB2D(vertexArray, bounds, tint, uvMins, uvMaxs);
	}
}

void BitmapFont::AddVertsForTextInBox2D(std::vector<Vertex_PCU>& vertexArray, AABB2 const& box, float cellHeight, std::string const& text, Rgba8 const& tint, float cellAspect, Vec2 const& alignment, TextBoxMode mode, int maxGlyphsToDraw)
{
	float textHeight = cellHeight;

	Strings stringTexts = SplitStringOnDelimiter(text, '\n');

	for (size_t index = 0; index < stringTexts.size(); index++)
	{
		float cellWidth = textHeight * cellAspect;

		if (mode == SHRINK_TO_FIT)
		{
			float textLength = stringTexts[index].size() * cellWidth;

			if (textLength > box.GetDimensions().x)
			{
				float shrinkValue = box.GetDimensions().x / textLength;
				cellWidth = shrinkValue * textHeight * cellAspect;
			}
		}
		else if (mode == OVERRUN)
		{
			cellWidth = textHeight * cellAspect;
		}

		Vec2 textStartPosition = box.m_mins;
		int textIndex = 0;
		int textSize = static_cast<int>(stringTexts[index].size()) < maxGlyphsToDraw ? static_cast<int>(stringTexts[index].size()) : maxGlyphsToDraw;

		while (textIndex < textSize)
		{
			textStartPosition.x = box.m_mins.x + ((box.GetDimensions().x - (stringTexts[index].size() * cellWidth)) * alignment.x);
			textStartPosition.y = box.m_mins.y + ((stringTexts.size() - 1 - index) * textHeight) + ((box.GetDimensions().y - (textHeight * stringTexts.size())) * alignment.y);

			AABB2 bounds = AABB2(textStartPosition.x + (cellWidth * textIndex), textStartPosition.y, textStartPosition.x + (cellWidth * (textIndex + 1)), textStartPosition.y + textHeight);

			SpriteDefinition const& testSpriteDef = m_fontGlyphsSpriteSheet->GetSpriteDef(stringTexts[index][textIndex]);

			Vec2 uvMins, uvMaxs;
			testSpriteDef.GetUVs(uvMins, uvMaxs);

			AddVertsForAABB2D(vertexArray, bounds, tint, uvMins, uvMaxs);

			textIndex++;
		}
	}
}

void BitmapFont::AddVertsForText3DAtOriginXForward(std::vector<Vertex_PCU>& verts, float cellHeight, std::string const& text, Rgba8 const& tint, float cellAspect, Vec2 const& alingment, int maxGlyphsToDraw)
{
	UNUSED(maxGlyphsToDraw);
	UNUSED(alingment);
	UNUSED(tint);

	float cellWidth = cellHeight * cellAspect;

	Vec2 textMins = Vec2::ZERO;

	for (size_t index = 0; index < text.size(); index++)
	{
		AABB2 bounds = AABB2(textMins.x + (cellWidth * index), textMins.y, textMins.x + (cellWidth * (index + 1)), textMins.y + cellHeight);

		SpriteDefinition const& testSpriteDef = m_fontGlyphsSpriteSheet->GetSpriteDef(text[index]);

		Vec2 uvMins, uvMaxs;
		testSpriteDef.GetUVs(uvMins, uvMaxs);

		if (text[index] == '\n')
		{
			textMins.x = -1.0f * (cellWidth * index);
			textMins.y -= cellHeight;
		}

		bounds.m_mins.x -= cellWidth * text.size() * 0.5f;
		bounds.m_mins.y -= cellHeight * 0.5f;

		bounds.m_maxs.x -= cellWidth * text.size() * 0.5f;
		bounds.m_maxs.y -= cellHeight * 0.5f;

		AddVertsForQuad3D(verts, Vec3(0.0f, bounds.m_mins.x, bounds.m_mins.y), Vec3(0.0f, bounds.m_maxs.x, bounds.m_mins.y), Vec3(0.0f, bounds.m_maxs.x, bounds.m_maxs.y), Vec3(0.0f, bounds.m_mins.x, bounds.m_maxs.y), tint, AABB2(uvMins, uvMaxs));
	}


}

float BitmapFont::GetTextWidth(float cellHeight, std::string const& text, float cellAspect)
{
	float cellWidth = cellHeight * cellAspect;

	return cellWidth * text.size();
}

float BitmapFont::GetGlyphAspect(int glyphUnicode) const
{
	UNUSED(glyphUnicode);

	return 1.0f;
}
