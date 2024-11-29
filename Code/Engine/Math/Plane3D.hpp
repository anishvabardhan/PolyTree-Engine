#pragma once

#include "Engine/Math/Vec3.hpp"

struct Plane3D
{
	Vec3		m_normal;
	float		m_distanceFromOriginAlongNormal		= 0.0f;
public:
	Plane3D() = default;
	Plane3D(Vec3 planeNormal, float distanceFromOrigin);
	~Plane3D();

	bool IsPointInFront(Vec3 const& referencePoint);

	void				operator=( Plane3D const& copyFrom );				// Plane3D = Plane3D

};