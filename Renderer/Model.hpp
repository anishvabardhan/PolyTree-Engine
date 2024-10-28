#pragma once

#include "Game/EngineBuildPreferences.hpp"

#if DX12_RENDERER

#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Renderer/DX12Renderer.hpp"

#include <vector>
#include <iostream>
#include <unordered_map>

struct ID3D12DescriptorHeap;

struct Frustum;

class Image;
class MeshBuffer;
class IndexBuffer;
class VertexBuffer;
class ConstantBuffer;

constexpr int MAX_VERTICES_PER_MESHLET = 64;
constexpr int MAX_TRIANGLES_PER_MESHLET = 42;

struct Edge
{
	uint32_t m_startVert;
	uint32_t m_endVert;

	Edge(uint32_t v1, uint32_t v2) : m_startVert(std::min(v1, v2)), m_endVert(std::max(v1, v2)) {};
};

struct EdgeHash {
	size_t operator()(const Edge& edge) const {

		return std::hash<int>()(edge.m_startVert) ^ (std::hash<int>()(edge.m_endVert) << 1);
	}
};

struct EdgeEqual {
	bool operator()(const Edge& lhs, const Edge& rhs) const {
		return lhs.m_startVert == rhs.m_startVert && lhs.m_endVert == rhs.m_endVert;
	}
};

struct PackedPrimitive
{
	uint32_t m_i0;
	uint32_t m_i1;
	uint32_t m_i2;

	PackedPrimitive() = default;
	PackedPrimitive(uint32_t i0, uint32_t i1, uint32_t i2) : m_i0(i0), m_i1(i1), m_i2(i2) {}
};

struct BoundingSphere
{
	Vec3 m_center;
	float m_radius;

	BoundingSphere() = default;
	BoundingSphere(Vec3 center, float radius) : m_center(center), m_radius(radius) {};
};

struct InlineMeshlet
{
	std::vector<uint32_t>				m_uniqueVertexIndices;
	std::vector<PackedPrimitive>		m_primitiveIndices;
	Rgba8								m_color;
};

struct Meshlet
{
	uint32_t							m_vertexOffset;
	uint32_t							m_vertexCount;

	uint32_t							m_primitiveOffset;
	uint32_t							m_primitiveCount;
	float								m_color[4];
};

struct CullData
{
	BoundingSphere						m_boundingSphere;
	uint8_t								m_normalCone[4];
	float								m_apexOffset;
};

struct Mesh
{
	//std::vector<Vertex_PCU>			m_vertices;
	std::vector<MeshVertex_PCU>		m_meshVertices;
	std::vector<uint32_t>			m_indices;
	std::vector<InlineMeshlet>		m_inlineMeshlets;
	std::vector<Meshlet>			m_meshlets;
	std::vector<CullData>			m_cullData;
	std::vector<uint32_t>			m_uniqueVertexIndices;
	std::vector<PackedPrimitive>	m_primitiveIndices;

									Mesh() = default;
									~Mesh() {};

	void							BuildAdjacencyList( const uint32_t* indices, uint32_t indexCount, std::vector<uint32_t>& adjacency );
	bool							IsMeshletFull(InlineMeshlet& meshlet);
	int								ComputeReuseScore(const InlineMeshlet& meshlet, uint32_t (&triIndices)[3]);
	Vec3							ComputeNormals(Vec3* triVerts);
	BoundingSphere					ComputeMinimumBoundingSphere(std::vector<Vec3> verts, size_t count);
	float							ComputeScore(const InlineMeshlet& meshlet, BoundingSphere sphere, BoundingSphere normal, uint32_t (&triIndices)[3], Vec3* triVerts);
	bool							AddToMeshlet(InlineMeshlet& meshlet, uint32_t (&tri)[3]);
	void							Meshletize(std::vector<InlineMeshlet>& output, const uint32_t* indices, uint32_t indexCount, const std::vector<MeshVertex_PCU> positions, uint32_t vertexCount);
	std::vector<Meshlet>			ComputeMeshlets();
	std::vector<CullData>			ComputeMeshletCullData();
	std::vector<BoundingSphere>		ComputeBoundSphereData();
	std::vector<std::pair<uint8_t[4], float>>			ComputeNormalConeData();

	// FOR TESTING PURPOSE!!! NOT FINAL CODE!!! HAVE TO WRITE MY OWN VERSION!!!!
	void							BuildTestAdjacencyList(const uint32_t* indices, uint32_t indexCount, const MeshVertex_PCU* positions, uint32_t vertexCount, std::vector<uint32_t>& adjacency);
	//BoundingSphere					ComputeTestMinimumBoundingSphere(Vec3* verts, size_t count);
	std::vector<CullData>			ComputeTestMeshletCullData();
};

class Model
{
public:
	Mesh*							m_mesh					= nullptr;
	MeshBuffer*						m_mbo					= nullptr;
	MeshBuffer*						m_vertexIndices			= nullptr;
	MeshBuffer*						m_primitiveIndices		= nullptr;
	MeshBuffer*						m_meshCullData			= nullptr;
	IndexBuffer*					m_ibo					= nullptr;
	VertexBuffer*					m_vbo					= nullptr;
	ConstantBuffer*					m_modelCBO				= nullptr;
	ConstantBuffer*					m_FrustumCBO			= nullptr;
	ConstantBuffer*					m_meshInstanceCBO		= nullptr;
	ConstantBuffer*					m_meshInfoCBO			= nullptr;
	Texture*						m_texture				= nullptr;
	Image*							m_image					= nullptr;

	ID3D12DescriptorHeap*			m_modelDescHeap			= nullptr;

	D3D12_CPU_DESCRIPTOR_HANDLE		m_modelCPUDescHandle;
public:
									Model() = default;
									Model(std::vector<Vertex_PCU> modelVertices, std::vector<unsigned int> modelIndices);
									Model(std::vector<MeshVertex_PCU> modelVertices, std::vector<unsigned int> modelIndices, std::string* textureFilePath = nullptr);
									~Model();

	Texture*						CreateModelTexture(Image* image);
	void							InitializeGPUData();

	void							SetMeshInfoConstants(uint32_t meshletCount);
	void							SetMeshInstanceConstants(float instanceScale);
	void							SetFrustumConstants(Frustum* frustum, Vec3 cullCamPosition);
};

bool								CompareTestScores(const std::pair<uint32_t, float>& a, const std::pair<uint32_t, float>& b);

Vec4								QuantizeSNorm(Vec4 value);
Vec4								QuantizeUNorm(Vec4 value);

#endif