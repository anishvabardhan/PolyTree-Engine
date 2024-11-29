#include "Plane3D.hpp"

#include "Engine/Math/MathUtils.hpp"

Plane3D::Plane3D(Vec3 planeNormal, float distanceFromOrigin)
{
	m_normal = planeNormal;
	m_distanceFromOriginAlongNormal = distanceFromOrigin;
}

Plane3D::~Plane3D()
{
}

bool Plane3D::IsPointInFront(Vec3 const& referencePoint)
{
	Vec3 planeCenter = m_normal * m_distanceFromOriginAlongNormal;

	Vec3 displacement = referencePoint - planeCenter;

	float signValue = DotProduct3D(displacement, m_normal.GetNormalized());

	if(signValue > 0.0f)
		return true;

	return false;
}

void Plane3D::operator=(Plane3D const& copyFrom)
{
	m_distanceFromOriginAlongNormal = copyFrom.m_distanceFromOriginAlongNormal;
	m_normal = copyFrom.m_normal;
}
