#pragma once

#include <vector>
#include <string>

#include "Engine/Core/Noise.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Core/MeshVertex_PCU.hpp"

struct TerrainType
{
	std::string					m_name;
	Rgba8						m_color;
	float						m_height;

								TerrainType() = default;
								TerrainType(std::string name, Rgba8 color, float height);
								~TerrainType() = default;
};

class Terrain
{
public:
	std::vector<TerrainType>	m_regions;
	std::vector<MeshVertex_PCUTBN>	m_vertices;
	std::vector<unsigned int>	m_indices;
	int							m_triangleIndex = 0;
public:
								Terrain() = default;
								Terrain(int width, int height);
								~Terrain() = default;

	void						CreateRegion(std::string name, Rgba8 color, float height);
	void						AddTriangle(int a, int b, int c);
	void						GenerateTerrain(NoiseMap heightMap);
};