#include "GPUMesh.hpp"

#include "Engine/Renderer/CPUMesh.hpp"

#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Renderer/Shader.hpp"
#include "Engine/Renderer/IndexBuffer.hpp"
#include "Engine/Renderer/VertexBuffer.hpp"

GPUMesh::~GPUMesh()
{
	DELETE_PTR(m_vbo);
	DELETE_PTR(m_ibo);
}

void GPUMesh::Create(CPUMesh const& cpuMesh)
{
	m_vbo = g_theRenderer->CreateVertexBuffer((int)cpuMesh.m_vertices.size() * sizeof(Vertex_PCUTBN));
	m_ibo = g_theRenderer->CreateIndexBuffer((int)cpuMesh.m_indices.size() * sizeof(unsigned int));

	g_theRenderer->CopyCPUToGPU(cpuMesh.m_vertices.data(), cpuMesh.m_vertices.size() * sizeof(Vertex_PCUTBN), m_vbo);
	g_theRenderer->CopyCPUToGPU(cpuMesh.m_indices.data(), cpuMesh.m_indices.size() * sizeof(unsigned int), m_ibo);
}

void GPUMesh::Render(Shader* shader) const
{
	g_theRenderer->SetBlendMode(BlendMode::ALPHA);
	g_theRenderer->SetDepthMode(DepthMode::ENABLED);
	g_theRenderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
	g_theRenderer->SetRasterizerMode(RasterizerMode::SOLID_CULL_NONE);
	g_theRenderer->BindShader(shader);
	g_theRenderer->DrawVertexBufferIndexed(m_vbo, m_ibo, sizeof(Vertex_PCUTBN));
}
