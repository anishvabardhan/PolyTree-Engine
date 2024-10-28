#pragma once

#include "Engine/Math/Plane3D.hpp"
#include "Engine/Core/Vertex_PCU.hpp"

#include <vector>

struct Frustum
{
	Plane3D m_nearPlane;
	Plane3D m_farPlane;

	Plane3D m_rightPlane;
	Plane3D m_leftPlane;

	Plane3D m_topPlane;
	Plane3D m_bottomPlane;

	std::vector<Vertex_PCU> m_frustumVerts;

	Frustum() = default;
};