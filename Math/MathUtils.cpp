#include "MathUtils.hpp"

#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Math//Mat44.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/OBB2.hpp"
#include "Engine/Math/EulerAngles.hpp"
#include "Engine/Core/EngineCommon.hpp"

#include <stdio.h>

float ConvertDegreesToRadians(float degrees)
{
	float radians = degrees * (M_PI / 180.0f);

	return radians;
}

float ConvertRadiansToDegrees(float radians)
{
	float degrees = radians * (180.0f / M_PI);

	return degrees;
}

float CosDegrees(float degrees)
{
	float radians = ConvertDegreesToRadians(degrees);

	return cosf(radians);
}

float SinDegrees(float degrees)
{
	float radians = ConvertDegreesToRadians(degrees);

	return sinf(radians);
}

float TanDegrees(float degrees)
{
	float radians = ConvertDegreesToRadians(degrees);

	return tanf(radians);
}

float Atan2Degrees(float y, float x)
{
	float radians = atan2f(y, x);

	float degrees = ConvertRadiansToDegrees(radians);

	return degrees;
}

float GetShortestAngularDispDegrees(float startDegrees, float endDegrees)
{
	float displacement = endDegrees - startDegrees;

	while(displacement > 180)
		displacement -= 360;

	while (displacement < -180)
		displacement += 360;

	return displacement;
}

float GetTurnedTowardDegrees(float currentDegrees, float goalDegrees, float maxDeltaDegrees)
{
	while (currentDegrees > 360.0f)
	{
		currentDegrees -= 360.0f;
	}
	while (goalDegrees > 360.0f)
	{
		goalDegrees -= 360.0f;
	}

	float delta = GetShortestAngularDispDegrees(currentDegrees, goalDegrees);

	if (fabsf(delta) < maxDeltaDegrees)
	{
		return currentDegrees + delta;
	}

	if (delta < 0.0f)
	{
		return currentDegrees - maxDeltaDegrees;
	}

	return currentDegrees + maxDeltaDegrees;
}

float GetAngleDegreesBetweenVectors2D(Vec2 const& a, Vec2 const& b)
{
	float dotProduct = DotProduct2D(a, b);

	float aLength = a.GetLength();
	float bLength = b.GetLength();

	float abProd = aLength * bLength;

	float angle = ConvertRadiansToDegrees(acosf(dotProduct / abProd));

	return angle;
}

float NormalizeByte(unsigned char byte)
{
	float normalizedByte = static_cast<float>(byte) / 255.0f;

	return normalizedByte;
}

unsigned char DenormalizeByte(float normalizedValue)
{
	float value = GetClampedZeroToOne(normalizedValue);

	if(value == 1.0f)
		return 255;
	else if(value == 0.0f)
		return 0;

	float delimiter = 1.0f / 256.0f;

	unsigned char denormalizedValue = static_cast<unsigned char>(ceilf(value / delimiter) - 1);

	return denormalizedValue;
}

float DotProduct2D(Vec2 const& a, Vec2 const& b)
{
	return (a.x * b.x) + (a.y * b.y);
}

float DotProduct2D(Vec3 const& a, Vec3 const& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float DotProduct2D(Vec4 const& a, Vec4 const& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

float DotProduct3D(Vec3 const& a, Vec3 const& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float DotProduct4D(Vec4 const& a, Vec4 const& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

float CrossProduct2D(Vec2 const& a, Vec2 const& b)
{
	return (a.x * b.y) - (a.y * b.x);
}

float CrossProduct2D(Vec3 const& a, Vec3 const& b)
{
	return (a.x * b.y) - (a.y * b.x);
}

float CrossProduct2D(Vec4 const& a, Vec4 const& b)
{
	return (a.x * b.y) - (a.y * b.x);
}

Vec3 CrossProduct3D(Vec3 const& a, Vec3 const& b)
{
	Vec3 crossProd;

	crossProd.x = (a.y * b.z) - (a.z * b.y);
	crossProd.y = (a.z * b.x) - (a.x * b.z);
	crossProd.z = (a.x * b.y) - (a.y * b.x);

	return crossProd;
}

float GetDistance2D(Vec2 const& positionA, Vec2 const& positionB)
{
	float distX = positionA.x - positionB.x;
	float distY = positionA.y - positionB.y;

	float distance = sqrtf((distX * distX) + (distY * distY));

	return distance;
}

float GetDistanceSquared2D(Vec2 const& positionA, Vec2 const& positionB)
{
	float distX = positionA.x - positionB.x;
	float distY = positionA.y - positionB.y;

	float distance = (distX * distX) + (distY * distY);

	return distance;
}

float GetDistance3D(Vec3 const& positionA, Vec3 const& positionB)
{
	float distX = positionA.x - positionB.x;
	float distY = positionA.y - positionB.y;
	float distZ = positionA.z - positionB.z;

	float distance = sqrtf((distX * distX) + (distY * distY) + (distZ * distZ));

	return distance;
}

float GetDistanceSquared3D(Vec3 const& positionA, Vec3 const& positionB)
{
	float distX = positionA.x - positionB.x;
	float distY = positionA.y - positionB.y;
	float distZ = positionA.z - positionB.z;

	float distance = (distX * distX) + (distY * distY) + (distZ * distZ);

	return distance;
}

float GetDistanceXY3D(Vec3 const& positionA, Vec3 const& positionB)
{
	float distX = positionA.x - positionB.x;
	float distY = positionA.y - positionB.y;

	float distance = sqrtf((distX * distX) + (distY * distY));

	return distance;
}

float GetDistanceXYSquared3D(Vec3 const& positionA, Vec3 const& positionB)
{
	float distX = positionA.x - positionB.x;
	float distY = positionA.y - positionB.y;

	float distance = (distX * distX) + (distY * distY);

	return distance;
}

float GetProjectedLength2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto)
{
	Vec2 normal = vectorToProjectOnto.GetNormalized();

	float length = DotProduct2D(vectorToProject, normal);

	return length;
}

Vec2 const GetProjectedOnto2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto)
{
	Vec2 normal = vectorToProjectOnto.GetNormalized();

	float length = GetProjectedLength2D(vectorToProject, vectorToProjectOnto);

	Vec2 vec = normal * length;

	return vec;
}

int GetTaxicabDistance2D(IntVec2 const& pointA, IntVec2 const& pointB)
{
	int xLength = abs(pointA.x - pointB.x);
	int yLength = abs(pointA.y - pointB.y);

	return xLength + yLength;
}

bool IsPointInsideDisc2D(Vec2 const& point, Vec2 const& discCenter, float discRadius)
{
	float distance = GetDistance2D(point, discCenter);

	if(distance < discRadius)
		return true;

	return false;
}

bool IsPointInsideAABB2D(Vec2 const& point, AABB2 const& box)
{
	Vec2 boxPoint = box.GetNearestPoint(point);

	if(point == boxPoint)
		return true;

	return false;
}

bool IsPointInsideCapsule2D(Vec2 const& point, Vec2 const& boneStart, Vec2 const& boneEnd, float radius)
{
	Vec2 direction = boneEnd - boneStart;
	direction.Normalize();

	Vec2 refStartDirection = point - boneStart;

	float startProduct = DotProduct2D(refStartDirection, direction);

	if (startProduct <= 0.0f && refStartDirection.GetLength() < radius)
	{
		return true;
	}

	Vec2 refEndDirection = point - boneEnd;

	float endProduct = DotProduct2D(refEndDirection, direction);

	if (endProduct >= 0.0f && refEndDirection.GetLength() < radius)
	{
		return true;
	}

	if (startProduct > 0.0f && endProduct < 0.0f)
	{
		Vec2 tangent = GetProjectedOnto2D(refStartDirection, direction);
		Vec2 normal = refStartDirection - tangent;
		
		if(normal.GetLength() < radius)
			return true;
	}

	return false;
}

bool IsPointInsideOBB2D(Vec2 const& point, OBB2 const& orientedBox)
{
	Vec2 localPoint = point - orientedBox.m_center;

	Vec2 iDir = orientedBox.m_iBasisNormal.GetNormalized();
	Vec2 jDir = iDir.GetRotated90Degrees();
	
	float iPoint = DotProduct2D(localPoint, iDir);
	float jPoint = DotProduct2D(localPoint, jDir);

	if (iPoint >= orientedBox.m_halfDimensions.x)
	{
		return false;
	}

	if (iPoint <= -orientedBox.m_halfDimensions.y)
	{
		return false;
	}

	if (jPoint >= orientedBox.m_halfDimensions.y)
	{
		return false;
	}

	if (jPoint <= -orientedBox.m_halfDimensions.y)
	{
		return false;
	}

	return true;
}

bool IsPointInsideOrientedSector2D(Vec2 const& point, Vec2 const& sectorTip, float sectorForwardDegrees, float sectorApertureDegrees, float sectorRadius)
{
	Vec2 sectorForwardNormal = Vec2::MakeFromPolarDegrees(sectorForwardDegrees, sectorRadius);

	return IsPointInsideDirectedSector2D(point, sectorTip, sectorForwardNormal, sectorApertureDegrees, sectorRadius);
}

bool IsPointInsideDirectedSector2D(Vec2 const& point, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius)
{
	if (GetDistance2D(point, sectorTip) >= sectorRadius)
		return false;

	Vec2 direction = point - sectorTip;

	float rotatedDegrees = GetAngleDegreesBetweenVectors2D(direction, sectorForwardNormal);

	if (fabsf(rotatedDegrees) <= sectorApertureDegrees / 2)
		return true;

	return false;
}

bool DoDiscsOverlap(Vec2 const& centerA, float const& radiusA, Vec2 const& centerB, float const& radiusB)
{
	float discDistance = GetDistance2D(centerA, centerB);
	float sumOfRadii = radiusA + radiusB;

	return discDistance < sumOfRadii;
}

bool DoSpheresOverlap(Vec3 const& centerA, float const& radiusA, Vec3 const& centerB, float const& radiusB)
{
	float sphereDistance = GetDistance3D(centerA, centerB);
	float sumOfRadii = radiusA + radiusB;

	return sphereDistance < sumOfRadii;
}

bool DoSphereAABB3Overlap(Vec3 const& centerA, float const& radiusA, AABB3 const& box)
{
	Vec3 nearestPointToSphere = GetNearestPointOnAABB3D(centerA, box);

	if(IsPointInsideSphere3D(nearestPointToSphere, centerA, radiusA))
		return true;

	return false;
}

bool DoSphereZCylinderOverlap(Vec3 const& centerA, float const& radiusA, Vec3 const& cylinderStart, float const& cylinderHeight, float const& cylinderRadius)
{
	Vec3 nearestPointToSphere = GetNearestPointOnZCylinder3D(centerA, cylinderStart, cylinderRadius, cylinderHeight);

	if (IsPointInsideSphere3D(nearestPointToSphere, centerA, radiusA))
		return true;

	return false;
}

bool DoAABB3sOverlap(AABB3 const& box1, AABB3 const& box2)
{
	if(box1.m_maxs.x <= box2.m_mins.x || box1.m_mins.x >= box2.m_maxs.x)
	{
		return false;
	}

	if (box1.m_maxs.y <= box2.m_mins.y || box1.m_mins.y >= box2.m_maxs.y)
	{
		return false;
	}

	if (box1.m_maxs.z <= box2.m_mins.z || box1.m_mins.z >= box2.m_maxs.z)
	{
		return false;
	}

	return true;
}

bool DoAABB3ZCylinerOverlap(AABB3 const& box1, Vec3 const& cylinderStart, float const& cylinderHeight, float const& cylinderRadius)
{
	float zMinA = box1.m_mins.z;
	float zMaxA = box1.m_maxs.z;
	float zMinB = cylinderStart.z;
	float zMaxB = cylinderStart.z + cylinderHeight;

	if (zMinA < zMaxB && zMaxA > zMinB)
	{
		Vec2 nearestPointToCylinder = GetNearestPointOnAABB2D(Vec2(cylinderStart.x, cylinderStart.y), AABB2(box1.m_mins.x, box1.m_mins.y, box1.m_maxs.x, box1.m_maxs.y));

		if(IsPointInsideDisc2D(nearestPointToCylinder, Vec2(cylinderStart.x, cylinderStart.y), cylinderRadius))
			return true;
	}

	//if (zMinA < zMinB || zMaxA > zMaxB)
	//{
	//	Vec2 nearestPointToCylinder = GetNearestPointOnAABB2D(Vec2(cylinderStart.x, cylinderStart.y), AABB2(box1.m_mins.x, box1.m_mins.y, box1.m_maxs.x, box1.m_maxs.y));
	//
	//	if (IsPointInsideDisc2D(nearestPointToCylinder, Vec2(cylinderStart.x, cylinderStart.y), cylinderRadius))
	//		return true;
	//}

	return false;
}

bool DoZCylindersOverlap(Vec3 const& cylinderStartA, float const& cylinderHeightA, float const& cylinderRadiusA, Vec3 const& cylinderStartB, float const& cylinderHeightB, float const& cylinderRadiusB)
{
	float zMinA = cylinderStartA.z;
	float zMaxA = cylinderStartA.z + cylinderHeightA;
	float zMinB = cylinderStartB.z;
	float zMaxB = cylinderStartB.z + cylinderHeightB;

	if (zMinA < zMaxB && zMaxA > zMinB)
	{
		if(DoDiscsOverlap(Vec2(cylinderStartA.x, cylinderStartA.y), cylinderRadiusA, Vec2(cylinderStartB.x, cylinderStartB.y), cylinderRadiusB))
			return true;
	}

	return false;
}

bool DoOBB3PlaneOverlap(OBB3 const& box, Plane3D const& plane)
{
	Vec3 iBasis = box.m_iBasisNormal.GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis).GetNormalized();
	Vec3 kBasis = CrossProduct3D(jBasis, iBasis).GetNormalized();

	Vec3 planeCenter = plane.m_normal * plane.m_distanceFromOriginAlongNormal;

	Vec3 vertices1 = box.m_center + (box.m_halfDimensions.x * iBasis) + (box.m_halfDimensions.y * jBasis) + (box.m_halfDimensions.z * kBasis);
	Vec3 vertices2 = box.m_center + (box.m_halfDimensions.x * iBasis) - (box.m_halfDimensions.y * jBasis) + (box.m_halfDimensions.z * kBasis);
	Vec3 vertices3 = box.m_center + (box.m_halfDimensions.x * iBasis) - (box.m_halfDimensions.y * jBasis) - (box.m_halfDimensions.z * kBasis);
	Vec3 vertices4 = box.m_center + (box.m_halfDimensions.x * iBasis) + (box.m_halfDimensions.y * jBasis) - (box.m_halfDimensions.z * kBasis);
	Vec3 vertices5 = box.m_center - (box.m_halfDimensions.x * iBasis) + (box.m_halfDimensions.y * jBasis) + (box.m_halfDimensions.z * kBasis);
	Vec3 vertices6 = box.m_center - (box.m_halfDimensions.x * iBasis) - (box.m_halfDimensions.y * jBasis) + (box.m_halfDimensions.z * kBasis);
	Vec3 vertices7 = box.m_center - (box.m_halfDimensions.x * iBasis) - (box.m_halfDimensions.y * jBasis) - (box.m_halfDimensions.z * kBasis);
	Vec3 vertices8 = box.m_center - (box.m_halfDimensions.x * iBasis) + (box.m_halfDimensions.y * jBasis) - (box.m_halfDimensions.z * kBasis);

	Vec3 planeCenterVertexDisplacement1 = vertices1 - planeCenter;
	Vec3 planeCenterVertexDisplacement2 = vertices2 - planeCenter;
	Vec3 planeCenterVertexDisplacement3 = vertices3 - planeCenter;
	Vec3 planeCenterVertexDisplacement4 = vertices4 - planeCenter;
	Vec3 planeCenterVertexDisplacement5 = vertices5 - planeCenter;
	Vec3 planeCenterVertexDisplacement6 = vertices6 - planeCenter;
	Vec3 planeCenterVertexDisplacement7 = vertices7 - planeCenter;
	Vec3 planeCenterVertexDisplacement8 = vertices8 - planeCenter;

	float dotProdVerts1 = DotProduct3D(planeCenterVertexDisplacement1, plane.m_normal.GetNormalized());
	float dotProdVerts2 = DotProduct3D(planeCenterVertexDisplacement2, plane.m_normal.GetNormalized());
	float dotProdVerts3 = DotProduct3D(planeCenterVertexDisplacement3, plane.m_normal.GetNormalized());
	float dotProdVerts4 = DotProduct3D(planeCenterVertexDisplacement4, plane.m_normal.GetNormalized());
	float dotProdVerts5 = DotProduct3D(planeCenterVertexDisplacement5, plane.m_normal.GetNormalized());
	float dotProdVerts6 = DotProduct3D(planeCenterVertexDisplacement6, plane.m_normal.GetNormalized());
	float dotProdVerts7 = DotProduct3D(planeCenterVertexDisplacement7, plane.m_normal.GetNormalized());
	float dotProdVerts8 = DotProduct3D(planeCenterVertexDisplacement8, plane.m_normal.GetNormalized());

	float signOfTotalValue = dotProdVerts1 * dotProdVerts2 * dotProdVerts3 * dotProdVerts4 * dotProdVerts5 * dotProdVerts6 * dotProdVerts7 * dotProdVerts8;

	if(signOfTotalValue < 0.0f)
		return true;

	return false;
}

Vec2 const GetNearestPointOnDisc2D(Vec2 const& referencePosition, Vec2 const discCenter, float discRadius)
{
	if(IsPointInsideDisc2D(referencePosition, discCenter, discRadius))
		return referencePosition;

	Vec2 direction = (referencePosition - discCenter);
	direction.ClampLength(discRadius);

	return discCenter + direction;
}

Vec2 const GetNearestPointOnAABB2D(Vec2 const& referencePosition, AABB2 const& box)
{
	return box.GetNearestPoint(referencePosition);
}

Vec2 const GetNearestPointOnInfiniteLine2D(Vec2 const& referencePosition, Vec2 const& pointOnLine, Vec2 const& anotherPointOnLine)
{
	Vec2 direction = anotherPointOnLine - pointOnLine;
	direction.Normalize();

	Vec2 refStartDirection = referencePosition - pointOnLine;

	float startProduct = DotProduct2D(refStartDirection, direction);

	if (startProduct <= 0.0f)
	{
		Vec2 tangent = GetProjectedOnto2D(refStartDirection, direction);
		return pointOnLine + tangent;
	}

	Vec2 refEndDirection = referencePosition - anotherPointOnLine;

	float endProduct = DotProduct2D(refEndDirection, direction);

	if (endProduct >= 0.0f)
	{
		Vec2 tangent = GetProjectedOnto2D(refEndDirection, direction);
		return anotherPointOnLine + tangent;
	}

	Vec2 tangent = GetProjectedOnto2D(refStartDirection, direction);

	return pointOnLine + tangent;
}

Vec2 const GetNearestPointOnLineSegment2D(Vec2 const& referencePosition, Vec2 const& lineSegStart, Vec2 const& lineSegEnd)
{
	Vec2 direction = lineSegEnd - lineSegStart;
	direction.Normalize();

	Vec2 refStartDirection = referencePosition - lineSegStart;

	float startProduct = DotProduct2D(refStartDirection, direction);

	if (startProduct <= 0.0f)
	{
		return lineSegStart;
	}

	Vec2 refEndDirection = referencePosition - lineSegEnd;

	float endProduct = DotProduct2D(refEndDirection, direction);

	if (endProduct >= 0.0f)
	{
		return lineSegEnd;
	}

	Vec2 tangent = GetProjectedOnto2D(refStartDirection, direction);

	return lineSegStart + tangent;
}

Vec2 const GetNearestPointOnCapsule2D(Vec2 const& referencePosition, Vec2 const& boneStart, Vec2 const& boneEnd, float radius)
{
	if(IsPointInsideCapsule2D(referencePosition, boneStart, boneEnd, radius))
		return referencePosition;

	Vec2 direction = boneEnd - boneStart;
	direction.Normalize();

	Vec2 refStartDirection = referencePosition - boneStart;

	float startProduct = DotProduct2D(refStartDirection, direction);

	if (startProduct <= 0.0f)
	{
		refStartDirection.SetLength(radius);
		return boneStart + refStartDirection;
	}

	Vec2 refEndDirection = referencePosition - boneEnd;

	float endProduct = DotProduct2D(refEndDirection, direction);

	if (endProduct >= 0.0f)
	{
		refEndDirection.SetLength(radius);
		return boneEnd + refEndDirection;
	}

	Vec2 tangent = GetProjectedOnto2D(refStartDirection, direction);
	Vec2 normal = refStartDirection - tangent;
	normal.Normalize();
	normal.SetLength(radius);
	
	Vec2 pointOnBone = boneStart + tangent;

	return pointOnBone + normal;
}

Vec2 const GetNearestPointOnOBB2D(Vec2 const& referencePosition, OBB2 const& orientedBox)
{
	Vec2 localPoint = referencePosition - orientedBox.m_center;

	Vec2 iDir = orientedBox.m_iBasisNormal.GetNormalized();
	Vec2 jDir = iDir.GetRotated90Degrees();

	float iPoint = DotProduct2D(localPoint, iDir);
	float jPoint = DotProduct2D(localPoint, jDir);

	Vec2 nearestPoint;
	nearestPoint.x = GetClamped(iPoint, -orientedBox.m_halfDimensions.x, orientedBox.m_halfDimensions.x);
	nearestPoint.y = GetClamped(jPoint, -orientedBox.m_halfDimensions.y, orientedBox.m_halfDimensions.y);

	nearestPoint = orientedBox.m_center + (iDir * nearestPoint.x) + (jDir * nearestPoint.y);

	return nearestPoint;
}

//TODO: Finish this.
Vec2 const GetNearestPointOnSector2D(Vec2 const& referencePosition, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius)
{
	if(IsPointInsideDirectedSector2D(referencePosition, sectorTip, sectorForwardNormal, sectorApertureDegrees, sectorRadius))
		return referencePosition;

	Vec2 distanceToPoint = referencePosition - sectorTip;

	if (distanceToPoint.GetLength() > sectorRadius)
	{
		float angleBetweenPointAndForward = GetAngleDegreesBetweenVectors2D(distanceToPoint, sectorForwardNormal);

		if (angleBetweenPointAndForward < sectorApertureDegrees * 0.5f)
		{
			distanceToPoint.Normalize();
			distanceToPoint.SetLength(sectorRadius);

			return sectorTip + distanceToPoint;
		}
	}
	else
	{
		float angleBetweenPointAndForward = GetAngleDegreesBetweenVectors2D(distanceToPoint, sectorForwardNormal);

		if (angleBetweenPointAndForward > sectorApertureDegrees * 0.5f)
		{
			
		}
	}

	return Vec2();
}

bool IsPointInsideSphere3D(Vec3 const& point, Vec3 const& discCenter, float discRadius)
{
	Vec3 pointVector = point - discCenter;

	if(pointVector.GetLengthSquared() < discRadius * discRadius)
		return true;

	return false;
}

bool IsPointInsideAABB3D(Vec3 const& point, AABB3 const& box)
{
	if (point.x > box.m_mins.x && point.x < box.m_maxs.x)
	{
		if (point.y > box.m_mins.y && point.y < box.m_maxs.y)
		{
			if (point.z > box.m_mins.z && point.z < box.m_maxs.z)
			{
				return true;
			}
		}
	}

	return false;
}

bool IsPointInsideZCylinder3D(Vec3 const& point, Vec3 const start, float radius, float height)
{
	float zValue = point.z;

	if(IsPointInsideDisc2D(Vec2(point.x, point.y), Vec2(start.x, start.y), radius) && zValue > start.z && zValue < start.z + height)
		return true;

	return false;
}

Vec3 const GetNearestPointOnSphere3D(Vec3 const& referencePosition, Vec3 const sphereCenter, float discRadius)
{
	Vec3 refVector = referencePosition - sphereCenter;

	return sphereCenter + (refVector.GetNormalized() * discRadius);
}

Vec3 const GetNearestPointOnAABB3D(Vec3 const& referencePosition, AABB3 const& box)
{
	return box.GetNearestPoint(referencePosition);
}

Vec3 const GetNearestPointOnOBB3D(Vec3 const& referencePosition, OBB3 const& box)
{
	Vec3 iBasis = box.m_iBasisNormal.GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis).GetNormalized();
	Vec3 kBasis = CrossProduct3D(jBasis, iBasis).GetNormalized();

	Mat44 boxTransform;
	boxTransform.SetIJKT3D(iBasis, jBasis, kBasis, box.m_center);

	Mat44 worldToLocalOBBTransform = boxTransform.GetOrthonormalInverse();

	AABB3 localBox = AABB3(-1.0f * box.m_halfDimensions, box.m_halfDimensions);

	Vec3 localRefPosition = worldToLocalOBBTransform.TransformPosition3D(referencePosition);

	Vec3 localNearestPoint = localBox.GetNearestPoint(localRefPosition);

	Vec3 worldNearestPoint = boxTransform.TransformPosition3D(localNearestPoint);

	return worldNearestPoint;
}

Vec3 const GetNearestPointOnZCylinder3D(Vec3 const& referencePosition, Vec3 const start, float radius, float height)
{
	Vec2 nearestPoint = GetNearestPointOnDisc2D(Vec2(referencePosition.x, referencePosition.y), Vec2(start.x, start.y), radius);

	float zMin = start.z;
	float zMax = zMin + height;

	float zValue = referencePosition.z;
	zValue = GetClamped(zValue, zMin, zMax);

	return Vec3(nearestPoint.x, nearestPoint.y, zValue);
}

Vec3 const GetNearestPointOnPlane3D(Vec3 const& referencePosition, Plane3D const& plane)
{
	Vec3 pointOnPlane = plane.m_normal * plane.m_distanceFromOriginAlongNormal;

	Vec3 displacement = referencePosition - pointOnPlane;

	Vec3 nearestPoint = referencePosition + (DotProduct3D(displacement, plane.m_normal.GetNormalized()) * -1.0f * plane.m_normal);

	return nearestPoint;
}

Mat44 GetBillboardMatrix(BillboardType type, Mat44 const& cameraMatrix, Vec3 const& billboardPosition, Vec2 const& billboardScale)
{
	UNUSED(billboardScale);

	Vec3 cameraPos = cameraMatrix.GetTranslation3D();

	if (type == BillboardType::FULL_CAMERA_OPPOSING)
	{
		Mat44 transform;

		transform.SetIJK3D(-1.0f * cameraMatrix.GetIBasis3D(), -1.0f * cameraMatrix.GetJBasis3D(), cameraMatrix.GetKBasis3D());
		transform.SetTranslation3D(billboardPosition);

		return transform;
	}
	else if (type == BillboardType::FULL_CAMERA_FACING)
	{
		Vec3 iBasis = (cameraPos - billboardPosition).GetNormalized();
		Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis).GetNormalized();
		Vec3 kBasis = CrossProduct3D(iBasis, jBasis);

		Mat44 transform;

		transform.SetIJK3D(iBasis, jBasis, kBasis);
		transform.SetTranslation3D(billboardPosition);

		return transform;
	}
	else if (type == BillboardType::WORLD_UP_CAMERA_OPPOSING)
	{
		Vec3 kBasis = Vec3(0.0f, 0.0f, 1.0f);
		Vec3 iBasis = -1.0f * CrossProduct3D(cameraMatrix.GetJBasis3D(), Vec3(0.0f, 0.0f, 1.0f)).GetNormalized();
		Vec3 jBasis = -1.0f * CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis).GetNormalized();

		Mat44 transform;

		transform.SetIJK3D(iBasis, jBasis, kBasis);
		transform.SetTranslation3D(billboardPosition);

		return transform;
	}
	else if (type == BillboardType::WORLD_UP_CAMERA_FACING)
	{
		Vec3 displacement = Vec3(cameraPos - billboardPosition).GetNormalized();
		Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), displacement).GetNormalized();
		Vec3 kBasis = Vec3(0.0f, 0.0f, 1.0f);
		Vec3 iBasis = CrossProduct3D(jBasis, kBasis).GetNormalized();

		Mat44 transform;

		transform.SetIJK3D(iBasis, jBasis, kBasis);
		transform.SetTranslation3D(billboardPosition);

		return transform;
	}

	return Mat44();
}

bool PushDiscOutOfPoint2D(Vec2& mobileDiscCenter, float discRadius, Vec2 const& fixedPoint)
{
	if(!IsPointInsideDisc2D(fixedPoint, mobileDiscCenter, discRadius))
		return false;

	Vec2 direction = mobileDiscCenter - fixedPoint;
	float displacement = discRadius - direction.GetLength();

	direction.ClampLength(displacement);

	mobileDiscCenter += direction;

	return true;
}

bool PushDiscOutOfDisc2D(Vec2& mobileDiscCenter, float mobileDiscRadius, Vec2 const& fixedDiscCenter, float const& fixedDiscRadius)
{
	float distance = GetDistance2D(mobileDiscCenter, fixedDiscCenter);

	float sumOfRadii = mobileDiscRadius + fixedDiscRadius;

	float displacement = sumOfRadii - distance;

	if(distance > sumOfRadii)
		return false;

	Vec2 direction = mobileDiscCenter - fixedDiscCenter;
	direction.ClampLength(displacement);

	mobileDiscCenter += direction;

	return true;
}

bool PushDiscsOutOfEachOther2D(Vec2& aCenter, float aRadius, Vec2& bCenter, float bRadius)
{
	float distance = GetDistance2D(aCenter, bCenter);

	float sumOfRadii = aRadius + bRadius;

	float displacement = sumOfRadii - distance;

	if (distance > sumOfRadii)
		return false;

	Vec2 direction = aCenter - bCenter;
	direction.ClampLength(displacement * 0.5f);

	aCenter += direction;
	bCenter -= direction;

	return true;
}

bool PushDiscOutOfAABB2D(Vec2& mobileDiscCenter, float discRadius, AABB2 const& fixedBox)
{
	Vec2 nearestPoint = fixedBox.GetNearestPoint(mobileDiscCenter);

	float distance = GetDistance2D(mobileDiscCenter, nearestPoint);

	if(distance > discRadius)
		return false;

	float displacement = discRadius - distance;

	Vec2 direction = mobileDiscCenter - nearestPoint;
	direction.ClampLength(displacement);

	mobileDiscCenter += direction;

	return true;
}

void TransformPosition2D(Vec2& posToTransform, float uniformScale, float rotationDegrees, Vec2 const& translation)
{
	posToTransform *= uniformScale;

	float thetaDegrees = Atan2Degrees(posToTransform.y, posToTransform.x);
	thetaDegrees += rotationDegrees;

	float distance = sqrtf((posToTransform.x * posToTransform.x) + (posToTransform.y * posToTransform.y));

	posToTransform.x = distance * CosDegrees(thetaDegrees);
	posToTransform.y = distance * SinDegrees(thetaDegrees);

	posToTransform.x += translation.x;
	posToTransform.y += translation.y;
}

void TransformPosition2D(Vec2& posToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation)
{
	Vec2 localPos = posToTransform;

	posToTransform.x = translation.x + (localPos.x * iBasis.x) + (localPos.y * jBasis.x);
	posToTransform.y = translation.y + (localPos.x * iBasis.y) + (localPos.y * jBasis.y);
}

void TransformPositionXY3D(Vec3& positionToTransform, float scaleXY, float zRotationDegrees, Vec2 const& translationXY)
{
	positionToTransform.x *= scaleXY;
	positionToTransform.y *= scaleXY;

	float thetaDegrees = Atan2Degrees(positionToTransform.y, positionToTransform.x);
	thetaDegrees += zRotationDegrees;

	float distance = sqrtf((positionToTransform.x * positionToTransform.x) + (positionToTransform.y * positionToTransform.y));

	positionToTransform.x = distance * CosDegrees(thetaDegrees);
	positionToTransform.y = distance * SinDegrees(thetaDegrees);

	positionToTransform.x += translationXY.x;
	positionToTransform.y += translationXY.y;
}

void TransformPositionXY3D(Vec3& positionToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation)
{
	Vec3 localPos = positionToTransform;

	positionToTransform.x = translation.x + (localPos.x * iBasis.x) + (localPos.y * jBasis.x);
	positionToTransform.y = translation.y + (localPos.x * iBasis.y) + (localPos.y * jBasis.y);
}

float SmoothStart2(float t)
{
	return t * t;
}

float SmoothStart3(float t)
{
	return t * t * t;
}

float SmoothStart4(float t)
{
	return t * t * t * t;
}

float SmoothStart5(float t)
{
	return t * t * t * t * t;
}

float SmoothStart6(float t)
{
	return t * t * t * t * t * t;
}

float SmoothStop2(float t)
{
	float s = 1.0f - t;

	return 1.0f - (s * s);
}

float SmoothStop3(float t)
{
	float s = 1.0f - t;

	return 1.0f - (s * s * s);
}

float SmoothStop4(float t)
{
	float s = 1.0f - t;

	return 1.0f - (s * s * s * s);
}

float SmoothStop5(float t)
{
	float s = 1.0f - t;

	return 1.0f - (s * s * s * s * s);
}

float SmoothStop6(float t)
{
	float s = 1.0f - t;

	return 1.0f - (s * s * s * s * s * s);
}

float SmoothStep3(float t)
{
	return Interpolate(SmoothStart2(t), SmoothStop2(t), t);
}

float SmoothStep5(float t)
{
	return Interpolate(SmoothStart3(t), SmoothStop3(t), t);
}

float Hesitate3(float t)
{
	return Interpolate(SmoothStop2(t), SmoothStart2(t), t);
}

float Hesitate5(float t)
{
	return Interpolate(SmoothStop3(t), SmoothStart3(t), t);
}

float CustomFunkyEasingFunction(float t)
{
	return SmoothStep3(t) * SmoothStart5(SmoothStep3(SmoothStop4(t)));
}

float ComputeCubicBezier1D(float A, float B, float C, float D, float t)
{
	float ab = Interpolate(A, B, t);
	float bc = Interpolate(B, C, t);
	float cd = Interpolate(C, D, t);
	
	float abc = Interpolate(ab, bc, t);
	float bcd = Interpolate(bc, cd, t);

	return Interpolate(abc, bcd, t);
}

float ComputeQuinticBezier1D(float A, float B, float C, float D, float E, float F, float t)
{
	float ab = Interpolate(A, B, t);
	float bc = Interpolate(B, C, t);
	float cd = Interpolate(C, D, t);
	float de = Interpolate(D, E, t);
	float ef = Interpolate(E, F, t);

	float abc = Interpolate(ab, bc, t);
	float bcd = Interpolate(bc, cd, t);
	float cde = Interpolate(cd, de, t);
	float def = Interpolate(de, ef, t);
	
	float abcd = Interpolate(abc, bcd, t);
	float bcde = Interpolate(bcd, cde, t);
	float cdef = Interpolate(cde, def, t);

	float abcde = Interpolate(abcd, bcde, t);
	float bcdef = Interpolate(bcde, cdef, t);

	return Interpolate(abcde, bcdef, t);
}

float GetClamped(float value, float minValue, float maxValue)
{
	if(value < minValue)
		return minValue;

	if(value > maxValue)
		return maxValue;

	return value;
}

float GetClampedZeroToOne(float value)
{
	if (value < 0.0f)
		return 0.0f;

	if (value > 1.0f)
		return 1.0f;

	return value;
}

float Interpolate(float start, float end, float fractionTowardEnd)
{
	float range = end - start;
	float lerpedPosition = start + (fractionTowardEnd * range);

	return lerpedPosition;
}

float GetFractionWithinRange(float value, float rangeStart, float rangeEnd)
{
	float fraction = (value - rangeStart) / (rangeEnd - rangeStart);

	return fraction;
}

float RangeMap(float inValue, float inStart, float inEnd, float outStart, float outEnd)
{
	float t = GetFractionWithinRange(inValue, inStart, inEnd);

	return Interpolate(outStart, outEnd, t);
}

float RangeMapClamped(float inValue, float inStart, float inEnd, float outStart, float outEnd)
{
	float t = GetFractionWithinRange(inValue, inStart, inEnd);

	t = GetClampedZeroToOne(t);

	return Interpolate(outStart, outEnd, t);
}

int RoundDownToInt(float value)
{
	float roundedNum = floorf(value);

	return static_cast<int>(roundedNum);
}
