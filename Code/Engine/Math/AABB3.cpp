#include "Engine/Math/AABB3.hpp"

#include "Engine/Math/MathUtils.hpp"

AABB3 const AABB3::ZERO_TO_ONE = AABB3(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);

AABB3::AABB3(AABB3 const& copyFrom)
{
	m_mins = copyFrom.m_mins;
	m_maxs = copyFrom.m_maxs;
}

AABB3::AABB3(float minX, float minY,float minZ, float maxX, float maxY, float maxZ)
{
	m_mins = Vec3(minX, minY, minZ);
	m_maxs = Vec3(maxX, maxY, maxZ);
}

AABB3::AABB3(Vec3 const& mins, Vec3 const& maxs)
{
	m_mins = mins;
	m_maxs = maxs;
}

void AABB3::StretchToIncludePoint(Vec3 const& point)
{
	if (point.x > m_maxs.x) m_maxs = Vec3(point.x, m_maxs.y, m_maxs.z);
	if (point.x < m_mins.x) m_mins = Vec3(point.x, m_mins.y, m_mins.z);
	if (point.y > m_maxs.y) m_maxs = Vec3(m_maxs.x, point.y, m_maxs.z);
	if (point.y < m_mins.y) m_mins = Vec3(m_mins.x, point.y, m_mins.z);
	if (point.z > m_maxs.z) m_maxs = Vec3(m_maxs.x, m_maxs.y, point.z);
	if (point.z < m_mins.z) m_mins = Vec3(m_mins.x, m_mins.y, point.z);
}

Vec3 const AABB3::GetNearestPoint(Vec3 const& referencePosition) const
{
	float nearestX = GetClamped(referencePosition.x, m_mins.x, m_maxs.x);
	float nearestY = GetClamped(referencePosition.y, m_mins.y, m_maxs.y);
	float nearestZ = GetClamped(referencePosition.z, m_mins.z, m_maxs.z);

	return Vec3(nearestX, nearestY, nearestZ);
}
