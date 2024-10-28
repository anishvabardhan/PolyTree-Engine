#include "CPUMesh.hpp"

#include "Engine/Renderer/ObjLoader.hpp"

bool CPUMesh::Load(std::string const& objFileName, Mat44& transform)
{
	return ObjLoader::Load(objFileName, transform, m_vertices, m_indices, m_hasNormals, m_hasUVs);
}
