#pragma once

#include "Engine/Core/Vertex_PCUTBN.hpp"

#include <vector>
#include <string>

struct Mat44;

class CPUMesh
{
public:
	std::vector<Vertex_PCUTBN>	m_vertices;
	std::vector<unsigned int>	m_indices;
	bool						m_hasNormals	= false;
	bool						m_hasUVs		= false;
public:
	CPUMesh() = default;
	~CPUMesh() = default;

	bool Load(std::string const& objFileName, Mat44& transform);
};