#include "Engine/Core/TileHeatMap.hpp"

#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/MathUtils.hpp"

TileHeatMap::TileHeatMap(IntVec2 const& dimensions)
	: m_dimensions(dimensions)
{
	int totalTiles = m_dimensions.x * m_dimensions.y;

	m_values.reserve(totalTiles);

	for (int index = 0; index < totalTiles; index++)
	{
		m_values.push_back(0.0f);
	}
}

TileHeatMap::~TileHeatMap()
{
}

void TileHeatMap::AddVertsForDebugDraw(std::vector<Vertex_PCU>& verts, AABB2 bounds, FloatRange valueRange, Rgba8 lowColor, Rgba8 highColor, float specialValue, Rgba8 specialColor)
{
	Vec2 tileDimensions = bounds.GetDimensions();
	tileDimensions.x /= m_dimensions.x;
	tileDimensions.y /= m_dimensions.y;

	int totalTiles = m_dimensions.x * m_dimensions.y;

	for (int tileIndex = 0; tileIndex < totalTiles; tileIndex++)
	{
		int tileX = tileIndex % m_dimensions.x;
		int tileY = tileIndex / m_dimensions.x;

		AABB2 tileBounds(static_cast<float>(tileX * tileDimensions.x), static_cast<float>(tileY * tileDimensions.y), static_cast<float>((tileX + 1) * tileDimensions.x), static_cast<float>((tileY + 1) * tileDimensions.y));

		if (m_values[tileIndex] == specialValue)
		{
			AddVertsForAABB2D(verts, tileBounds, specialColor);
		}
		else
		{
			m_values[tileIndex] = RangeMapClamped(m_values[tileIndex], valueRange.m_min, valueRange.m_max, 0.0f, 1.0f);

			Rgba8 color = Interpolate(lowColor, highColor, m_values[tileIndex]);

			AddVertsForAABB2D(verts, tileBounds, color);
		}
	}
}

float TileHeatMap::GetTileHeatValue(IntVec2 tileCoord) const
{
	int tileIndex = tileCoord.x + (tileCoord.y * m_dimensions.x);

	return m_values[tileIndex];
}

void TileHeatMap::SetTileHeatValue(IntVec2 tileCoord, float value)
{
	int tileIndex = tileCoord.x + (tileCoord.y * m_dimensions.x);

	m_values[tileIndex] = value;
}

void TileHeatMap::SetAllValues(float value)
{
	int totalTiles = m_dimensions.x * m_dimensions.y;

	for (int index = 0; index < totalTiles; index++)
	{
		m_values[index] = value;
	}
}

void TileHeatMap::AddTileHeatValue(IntVec2 tileCoord, float value)
{
	int tileIndex = tileCoord.x + (tileCoord.y * m_dimensions.x);

	m_values[tileIndex] += value;
}
