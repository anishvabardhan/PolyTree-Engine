#include "AABB2.hpp"

#include "Engine/Math/MathUtils.hpp"

AABB2 const AABB2::ZERO_TO_ONE = AABB2(0.0f, 0.0f, 1.0f, 1.0f);

AABB2::AABB2(AABB2 const& copyFrom)
{
	m_mins = copyFrom.m_mins;
	m_maxs = copyFrom.m_maxs;
}

AABB2::AABB2(float minX, float minY, float maxX, float maxY)
{
	m_mins = Vec2(minX, minY);
	m_maxs = Vec2(maxX, maxY);
}

AABB2::AABB2(Vec2 const& mins, Vec2 const& maxs)
{
	m_mins = mins;
	m_maxs = maxs;
}

bool AABB2::IsPointInside(Vec2 const& point) const
{
	bool insideX = false;
	bool insideY = false;

	if(point.x > m_mins.x && point.x < m_maxs.x)
		insideX = true;

	if (point.y > m_mins.y && point.y < m_maxs.y)
		insideY = true;

	return insideX && insideY;
}

Vec2 const AABB2::GetCenter() const
{
	float centerX = 0.5f * (m_maxs.x + m_mins.x); 
	float centerY = 0.5f * (m_maxs.y + m_mins.y);

	return Vec2(centerX, centerY);
}

Vec2 const AABB2::GetDimensions() const
{
	float dimX = m_maxs.x - m_mins.x;
	float dimY = m_maxs.y - m_mins.y;

	return Vec2(dimX, dimY);
}

Vec2 const AABB2::GetNearestPoint(Vec2 const& referencePosition) const
{
	float nearestX = GetClamped(referencePosition.x, m_mins.x, m_maxs.x);
	float nearestY = GetClamped(referencePosition.y, m_mins.y, m_maxs.y);

	return Vec2(nearestX, nearestY);
}

Vec2 const AABB2::GetPointAtUV(Vec2 const& uv) const
{
	float newX = Interpolate(m_mins.x, m_maxs.x, uv.x);
	float newY = Interpolate(m_mins.y, m_maxs.y, uv.y);

	return Vec2(newX, newY);
}

Vec2 const AABB2::GetUVForPoint(Vec2 const& point) const
{
	float u = GetFractionWithinRange(point.x, m_mins.x, m_maxs.x);
	float v = GetFractionWithinRange(point.y, m_mins.y, m_maxs.y);

	return Vec2(u, v);
}

void AABB2::Translate(Vec2 const& translationToApply)
{
	m_mins += translationToApply;
	m_maxs += translationToApply;
}

void AABB2::SetCenter(Vec2 const& newCenter)
{
	Vec2 oldCenter = GetCenter();

	Vec2 deltaCenter = newCenter - oldCenter;

	m_mins.x += deltaCenter.x;
	m_maxs.x += deltaCenter.x;

	m_mins.y += deltaCenter.y;
	m_maxs.y += deltaCenter.y;
}

void AABB2::SetDimensions(Vec2 const& newDimensions)
{
	Vec2 center = GetCenter();

	m_maxs.x = m_mins.x + newDimensions.x;
	m_maxs.y = m_mins.y + newDimensions.y;

	SetCenter(center);
}

void AABB2::StretchToIncludePoint(Vec2 const& point)
{
	if(point.x > m_maxs.x)
		m_maxs.x = point.x;

	if (point.x < m_mins.x)
		m_mins.x = point.x;

	if (point.y > m_maxs.y)
		m_maxs.y = point.y;

	if (point.y < m_mins.y)
		m_mins.y = point.y;
}
