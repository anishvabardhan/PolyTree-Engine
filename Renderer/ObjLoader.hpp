#pragma once

#include "Game/EngineBuildPreferences.hpp"

#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Renderer/Model.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <map>

struct Mat44;

struct Vertex
{
	int m_v = -1;
	int m_vt = -1;
	int m_vn = -1;
};

struct VertexHash {
	size_t operator()(const Vertex& vertex) const {

		return std::hash<int>()(vertex.m_v) ^ (std::hash<int>()(vertex.m_vt) << 1) ^ (std::hash<int>()(vertex.m_vn) << 2);
	}
};

struct VertexEqual {
	bool operator()(const Vertex& lhs, const Vertex& rhs) const {
		return lhs.m_v == rhs.m_v && lhs.m_vt == rhs.m_vt && lhs.m_vn == rhs.m_vn;
	}
};

struct Face
{
	std::vector<Vertex> m_vertices;
	Rgba8 m_color;
};

struct Triangle
{
	int m_vertexPositionIndex[3]{-1, -1, -1};
	Rgba8 m_color = Rgba8::WHITE;
	int m_vertexTextureCoordinateIndex[3]{-1, -1, -1};
	int m_vertexNormalIndex[3]{-1, -1, -1};
	Triangle(int v0, int v1, int v2,int t0 = -1, int t1 = -1, int t2 = -1, int n0 = -1, int n1 = -1, int n2 = -1, Rgba8 const& color = Rgba8::WHITE)
	{
		m_color = color;

		m_vertexPositionIndex[0] = v0;
		m_vertexPositionIndex[1] = v1;
		m_vertexPositionIndex[2] = v2;

		m_vertexTextureCoordinateIndex[0] = t0;
		m_vertexTextureCoordinateIndex[1] = t1;
		m_vertexTextureCoordinateIndex[2] = t2;

		m_vertexNormalIndex[0] = n0;
		m_vertexNormalIndex[1] = n1;
		m_vertexNormalIndex[2] = n2;
	}
};

class ObjLoader
{
public:
	ObjLoader() = default;
	~ObjLoader() = default;

	static bool Load(std::string const& fileName, Mat44 const& transform, std::vector<Vertex_PCUTBN>& outVertices, std::vector<unsigned int>& outIndices, bool& outHasNormals, bool& outHasUVs);

	static void ParsingVertexPos(std::string line, std::vector<Vec3>& positions);
	static void ParsingVertexUVs(std::string line, std::vector<Vec2>& uvs, bool& outHasUVs);
	static void ParsingVertexNormals(std::string line, std::vector<Vec3>& normals, bool& outHasNormals);
	static void ParsingFaces(std::string line, std::vector<Face>& outFaces, Rgba8 faceColor = Rgba8::WHITE);
	static void ParsingMaterialFile(std::string line, std::unordered_map<std::string, Rgba8>& outMaterialLib);
	static void GenerateVerticesAndIndices(std::vector<Face> inFaces, std::vector<Vec3> positions, std::vector<Vec2> uvs, std::vector<Vec3> normals, std::vector<Vertex_PCUTBN>& outVertices, std::vector<unsigned int>& outIndices);

#if DX12_RENDERER

	static bool Load(std::string const& fileName, Mat44 const& transform, std::vector<MeshVertex_PCU>& outVertices, std::vector<unsigned int>& outIndices, bool& outHasNormals, bool& outHasUVs);
	static bool LoadXML(std::string const& fileName, Mat44 const& transform, std::vector<MeshVertex_PCU>& outVertices, std::vector<unsigned int>& outIndices, bool& outHasNormals, bool& outHasUVs);
	static void GenerateVerticesAndIndices(std::vector<Face> inFaces, std::vector<Vec3> positions, std::vector<Vec2> uvs, std::vector<Vec3> normals, std::vector<MeshVertex_PCU>& outVertices, std::vector<unsigned int>& outIndices);

#endif
};