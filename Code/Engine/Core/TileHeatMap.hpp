#pragma once

#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Math/FloatRange.hpp"

#include <vector>

struct Vertex_PCU;
struct AABB2;

class TileHeatMap
{
public:
	IntVec2 m_dimensions;
	std::vector<float> m_values;
public:
	TileHeatMap(IntVec2 const& dimensions);
	~TileHeatMap();

	void AddVertsForDebugDraw( std::vector<Vertex_PCU>& verts, AABB2 bounds, FloatRange valueRange = FloatRange(0.0, 1.0f), Rgba8 lowColor = Rgba8(0,0,0,100), Rgba8 highColor = Rgba8(255,255,255,100), float specialValue = 999999.0f, Rgba8 specialColor = Rgba8(0, 0, 255, 255) );
	float GetTileHeatValue(IntVec2 tileCoord) const;
	void SetTileHeatValue(IntVec2 tileCoord, float value);
	void SetAllValues(float value);
	void AddTileHeatValue(IntVec2 tileCoord, float value);
};