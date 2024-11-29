#pragma once

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/OBB3.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/AABB3.hpp"
#include "Engine/Math/Plane3D.hpp"

struct RaycastResult2D
{
	// Basic raycast result information (required)
	bool	m_didImpact = false;
	float	m_impactDist = 0.f;
	Vec2	m_impactPos;
	Vec2	m_impactNormal;

	// Original raycast information (optional)
	Vec2	m_rayFwdNormal;
	Vec2	m_rayStartPos;
	float	m_rayMaxLength = 1.f;
};

struct RaycastResult3D
{
	// Basic raycast result information (required)
	bool	m_didImpact = false;
	float	m_impactDist = 0.f;
	Vec3	m_impactPos;
	Vec3	m_impactNormal;

	// Original raycast information (optional)
	Vec3	m_rayFwdNormal;
	Vec3	m_rayStartPos;
	float	m_rayMaxLength = 1.f;
};

RaycastResult2D RaycastVsDisc2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Vec2 discCenter, float discRadius);
RaycastResult2D RaycastVsLineSegment2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Vec2 lineStart, Vec2 lineEnd);
RaycastResult2D RaycastVsAABB2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, AABB2 const& bounds);
RaycastResult3D RaycastVsAABB3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, AABB3 const& bounds);
RaycastResult3D RaycastVsOBB3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, OBB3 const& box);
RaycastResult3D RaycastVsPlane3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, Plane3D const& plane);
RaycastResult3D RaycastVsSphere3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, Vec3 sphereCenter, float sphereRadius);
RaycastResult3D RaycastVsZCylinder3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, Vec3 cylinderStart, float height, float radius);