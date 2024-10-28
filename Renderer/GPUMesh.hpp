#pragma once

#include <vector>
#include <string>

struct Mat44;
struct Vertex_PCUTBN;

class Shader;
class CPUMesh;
class IndexBuffer;
class VertexBuffer;

class GPUMesh
{
	VertexBuffer*	m_vbo = nullptr;
	IndexBuffer*	m_ibo = nullptr;
public:
					GPUMesh() = default;
					~GPUMesh();

	void			Create(CPUMesh const& cpuMesh);
	void			Render(Shader* shader) const;
};