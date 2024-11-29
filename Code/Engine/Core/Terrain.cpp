#include "Terrain.hpp"

#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/VertexUtils.hpp"

Terrain::Terrain(int width, int height)
{
	m_vertices.resize(width * height);
	m_indices.resize((width - 1) * (height - 1) * 6);
}

void Terrain::CreateRegion(std::string name, Rgba8 color, float height)
{
	m_regions.push_back(TerrainType(name, color, height));
}

void Terrain::AddTriangle(int a, int b, int c)
{
	m_indices[m_triangleIndex++] = a;
	m_indices[m_triangleIndex++] = b;
	m_indices[m_triangleIndex++] = c;
}

void Terrain::GenerateTerrain(NoiseMap heightMap)
{
	int mapWidth = (int)heightMap[0].size();
	int mapHeight = (int)heightMap.size();

	Vec2 uvCoord;

	int vertexIndex = 0;

	for (int y = 0; y < mapHeight; y++)
	{
		for (int x = 0; x < mapWidth; x++)
		{
			Vec3 vertex = Vec3((float)x, (float)y, (heightMap[y][x] * 75.0f) - 25.0f);

			uvCoord.x = RangeMap((float)x, 0.0f, mapWidth * 1.0f, 0.0f, 64.0f);
			uvCoord.y = RangeMap((float)y, 0.0f, mapHeight * 1.0f, 0.0f, 64.0f);

			for (int i = 0; i < m_regions.size(); i++)
			{
				if (m_regions[i].m_height >= heightMap[y][x])
				{
					float color[4];
					m_regions[i].m_color.GetAsFloats(color);

					m_vertices[vertexIndex] = MeshVertex_PCUTBN(vertex, Vec4(color[0], color[1], color[2], color[3]), uvCoord, Vec3::ZERO);
					break;
				}
			}

			if (x < mapWidth - 1 && y < mapHeight - 1)
			{
				AddTriangle(vertexIndex, vertexIndex + 1, vertexIndex + mapWidth + 1);
				AddTriangle(vertexIndex, vertexIndex + mapWidth + 1, vertexIndex + mapWidth);
			}

			vertexIndex++;
		}
	}

	CalculateTangentSpaceBasisVectors(m_vertices, m_indices, true, true);
}

TerrainType::TerrainType(std::string name, Rgba8 color, float height)
{
	m_name = name;
	m_color = color;
	m_height = height;
}
