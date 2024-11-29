#include "Engine/Math/RaycastUtils.hpp"

#include "Engine/Math/Mat44.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/FloatRange.hpp"

#include <vector>
#include <algorithm>

RaycastResult2D RaycastVsDisc2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Vec2 discCenter, float discRadius)
{
	RaycastResult2D result;

	Vec2 iBasis = fwdNormal.GetNormalized();
	Vec2 jBasis = iBasis.GetRotated90Degrees();

	Vec2 centerToStartPos = discCenter - startPos;
	float centerToPosJMagnitude = DotProduct2D(centerToStartPos, jBasis);

	if(centerToPosJMagnitude >= discRadius)
		return result;

	if(centerToPosJMagnitude <= -discRadius)
		return result;

	float a = sqrtf((discRadius * discRadius) - (centerToPosJMagnitude * centerToPosJMagnitude));
	float centerToPosIMagnitude = DotProduct2D(centerToStartPos, iBasis);
	float impactDistance = centerToPosIMagnitude - a;

	if(maxDist <= impactDistance)
		return result;

	if (IsPointInsideDisc2D(startPos, discCenter, discRadius))
	{
		Vec2 normal = iBasis.GetRotatedDegrees(180.0f);

		result.m_didImpact = true;
		result.m_impactNormal = normal;
		result.m_impactPos = startPos;

		return result;
	}

	float dotProductCentertoRaycast = DotProduct2D(centerToStartPos, iBasis);

	if(dotProductCentertoRaycast <= 0.0f)
		return result;

	Vec2 impactVector = iBasis;
	impactVector.SetLength(impactDistance);

	Vec2 impactPosition = startPos + impactVector;

	Vec2 impactNormal = (impactPosition - discCenter).GetNormalized();

	result.m_didImpact = true;
	result.m_impactDist = impactDistance;
	result.m_impactPos = impactPosition;
	result.m_impactNormal = impactNormal;

	return result;
}

RaycastResult2D RaycastVsLineSegment2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, Vec2 lineStart, Vec2 lineEnd)
{
	RaycastResult2D result;

	Vec2 jBasis = fwdNormal.GetRotated90Degrees();

	float rayLineStart = DotProduct2D(lineStart - startPos, jBasis);
	float rayLineEnd = DotProduct2D(lineEnd - startPos, jBasis);

	if(rayLineStart * rayLineEnd > 0)
		return result;

	float t = rayLineStart / (rayLineStart - rayLineEnd);

	Vec2 impactPoint = lineStart + t * (lineEnd - lineStart);

	float impactDist = DotProduct2D(impactPoint - startPos, fwdNormal);

	if(impactDist >= maxDist || impactDist <= 0.0f)
		return result;

	Vec2 impactNormal = (lineEnd - lineStart).GetRotated90Degrees().GetNormalized();

	if(rayLineEnd < 0.0f)
		impactNormal *= -1.0f;

	result.m_didImpact = true;
	result.m_impactDist = impactDist;
	result.m_impactNormal = impactNormal;
	result.m_impactPos = impactPoint;
	result.m_rayFwdNormal = fwdNormal;
	result.m_rayMaxLength = maxDist;
	result.m_rayStartPos = startPos;

	return result;
}

RaycastResult2D RaycastVsAABB2D(Vec2 startPos, Vec2 fwdNormal, float maxDist, AABB2 const& bounds)
{
	if (IsPointInsideAABB2D(startPos, bounds))
	{
		RaycastResult2D result;

		result.m_didImpact = true;
		result.m_rayFwdNormal = fwdNormal;
		result.m_impactNormal = -1.0f * fwdNormal;
		result.m_impactPos = startPos;
		result.m_rayMaxLength = maxDist;
		result.m_rayStartPos = startPos;

		return result;
	}

	Vec2 minXminY = bounds.m_mins;
	Vec2 maxXmaxY = bounds.m_maxs;
	Vec2 minXmaxY = Vec2(bounds.m_mins.x, bounds.m_maxs.y);
	Vec2 maxXminY = Vec2(bounds.m_maxs.x, bounds.m_mins.y);

	RaycastResult2D resultminXminY = RaycastVsLineSegment2D(startPos, fwdNormal, maxDist, minXminY, maxXminY);
	RaycastResult2D resultmaxXminY = RaycastVsLineSegment2D(startPos, fwdNormal, maxDist, maxXminY, maxXmaxY);
	RaycastResult2D resultmaxXmaxY = RaycastVsLineSegment2D(startPos, fwdNormal, maxDist, maxXmaxY, minXmaxY);
	RaycastResult2D resultminXmaxY = RaycastVsLineSegment2D(startPos, fwdNormal, maxDist, minXmaxY, minXminY);

	if (resultminXminY.m_didImpact)
	{
		if (resultmaxXmaxY.m_didImpact)
		{
			return resultminXminY.m_impactDist < resultmaxXmaxY.m_impactDist ? resultminXminY : resultmaxXmaxY;
		}

		if (resultmaxXminY.m_didImpact)
		{
			return resultminXminY.m_impactDist < resultmaxXminY.m_impactDist ? resultminXminY : resultmaxXminY;
		}

		if (resultminXmaxY.m_didImpact)
		{
			return resultminXminY.m_impactDist < resultminXmaxY.m_impactDist ? resultminXminY : resultminXmaxY;
		}

		return resultminXminY;
	}

	if (resultmaxXminY.m_didImpact)
	{
		if (resultmaxXmaxY.m_didImpact)
		{
			return resultmaxXminY.m_impactDist < resultmaxXmaxY.m_impactDist ? resultmaxXminY : resultmaxXmaxY;
		}

		if (resultminXminY.m_didImpact)
		{
			return resultmaxXminY.m_impactDist < resultminXminY.m_impactDist ? resultmaxXminY : resultminXminY;
		}

		if (resultminXmaxY.m_didImpact)
		{
			return resultmaxXminY.m_impactDist < resultminXmaxY.m_impactDist ? resultmaxXminY : resultminXmaxY;
		}

		return resultmaxXminY;
	}

	if (resultmaxXmaxY.m_didImpact)
	{
		if (resultmaxXminY.m_didImpact)
		{
			return resultmaxXminY.m_impactDist < resultmaxXmaxY.m_impactDist ? resultmaxXminY : resultmaxXmaxY;
		}

		if (resultminXminY.m_didImpact)
		{
			return resultmaxXmaxY.m_impactDist < resultminXminY.m_impactDist ? resultmaxXmaxY : resultminXminY;
		}

		if (resultminXmaxY.m_didImpact)
		{
			return resultmaxXmaxY.m_impactDist < resultminXmaxY.m_impactDist ? resultmaxXmaxY : resultminXmaxY;
		}

		return resultmaxXmaxY;
	}

	if (resultminXmaxY.m_didImpact)
	{
		if (resultmaxXminY.m_didImpact)
		{
			return resultmaxXminY.m_impactDist < resultminXmaxY.m_impactDist ? resultmaxXminY : resultminXmaxY;
		}

		if (resultminXminY.m_didImpact)
		{
			return resultminXmaxY.m_impactDist < resultminXminY.m_impactDist ? resultminXmaxY : resultminXminY;
		}

		if (resultmaxXmaxY.m_didImpact)
		{
			return resultmaxXmaxY.m_impactDist < resultminXmaxY.m_impactDist ? resultmaxXmaxY : resultminXmaxY;
		}

		return resultminXmaxY;
	}

	return RaycastResult2D();
}

RaycastResult3D RaycastVsAABB3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, AABB3 const& bounds)
{
	RaycastResult3D raycast;
	raycast.m_rayStartPos = startPos;
	raycast.m_rayFwdNormal = fwdNormal;
	raycast.m_rayMaxLength = maxDist;
	raycast.m_impactPos = startPos + (maxDist * fwdNormal);

	Vec3 endPos = startPos + (fwdNormal * maxDist);

	AABB3 rayAABB3 = AABB3(startPos, startPos);
	rayAABB3.StretchToIncludePoint(endPos);

	if(!DoAABB3sOverlap(bounds, rayAABB3))
		return raycast;

	if (IsPointInsideAABB3D(startPos, bounds))
	{
		raycast.m_didImpact = true;
		raycast.m_impactNormal = -1.0f * fwdNormal;
		raycast.m_impactDist = 0.0f;
		raycast.m_impactPos = startPos;
		raycast.m_rayFwdNormal = fwdNormal;
		raycast.m_rayMaxLength = maxDist;
		raycast.m_rayStartPos = startPos;

		return raycast;
	}

	float tXFirst = (bounds.m_mins.x - startPos.x) / (endPos.x - startPos.x);
	float tXSecond = (bounds.m_maxs.x - startPos.x) / (endPos.x - startPos.x);

	float tXMin = std::min(tXFirst, tXSecond);
	float tXMax = std::max(tXFirst, tXSecond);

	FloatRange tX = FloatRange(tXMin, tXMax);

	float tYFirst = (bounds.m_mins.y - startPos.y) / (endPos.y - startPos.y);
	float tYSecond = (bounds.m_maxs.y - startPos.y) / (endPos.y - startPos.y);

	float tYMin = std::min(tYFirst, tYSecond);
	float tYMax = std::max(tYFirst, tYSecond);

	FloatRange tY = FloatRange(tYMin, tYMax);

	float tZFirst = (bounds.m_mins.z - startPos.z) / (endPos.z - startPos.z);
	float tZSecond = (bounds.m_maxs.z - startPos.z) / (endPos.z - startPos.z);

	float tZMin = std::min(tZFirst, tZSecond);
	float tZMax = std::max(tZFirst, tZSecond);

	FloatRange tZ = FloatRange(tZMin, tZMax);

	FloatRange overlapRange;

	bool xOverlapY = tX.IsOverlappingWith(tY);
	bool yOverlapZ = tY.IsOverlappingWith(tZ);
	bool zOverlapX = tZ.IsOverlappingWith(tX);

	if (xOverlapY && yOverlapZ && zOverlapX)
	{
		float t = FLT_MAX;

		if (tY.IsOnRange(tXMin))
		{
			if (tXMin < t)
			{
				t = tXMin;
			}
		}

		if (tY.IsOnRange(tXMax))
		{
			if (tXMax < t)
			{
				t = tXMax;
			}
		}

		if (tX.IsOnRange(tYMin))
		{
			if (tYMin < t)
			{
				t = tYMin;
			}
		}

		if (tX.IsOnRange(tYMax))
		{
			if (tYMax < t)
			{
				t = tYMax;
			}
		}

		if (tX.IsOnRange(tZMax))
		{
			if (tZMax < t)
			{
				t = tZMax;
			}
		}

		if (tX.IsOnRange(tZMin))
		{
			if (tZMin < t)
			{
				t = tZMin;
			}
		}

		if (tY.IsOnRange(tZMax))
		{
			if (tZMax < t)
			{
				t = tZMax;
			}
		}

		if (tY.IsOnRange(tZMin))
		{
			if (tZMin < t)
			{
				t = tZMin;
			}
		}

		if (t == tXMin)
		{
			raycast.m_impactNormal = Vec3(-1.0f, 0.0f, 0.0f);
		}
		else if (t == tXMax)
		{
			raycast.m_impactNormal = Vec3(1.0f, 0.0f, 0.0f);
		}
		else if (t == tYMin)
		{
			raycast.m_impactNormal = Vec3(0.0f, -1.0f, 0.0f);
		}
		else if (t == tYMax)
		{
			raycast.m_impactNormal = Vec3(0.0f, 1.0f, 0.0f);
		}

		else if (t == tZMin)
		{
			raycast.m_impactNormal = Vec3(0.0f, 0.0f, -1.0f);
		}
		else if (t == tZMax)
		{
			raycast.m_impactNormal = Vec3(0.0f, 0.0f, 1.0f);
		}

		if (tX.m_min < tY.m_min)
		{
			if (tY.m_min < tZ.m_min)
			{
				overlapRange.m_min = tZ.m_min;
			}
			else
			{
				overlapRange.m_min = tY.m_min;
			}
		}
		else
		{
			if (tX.m_min < tZ.m_min)
			{
				overlapRange.m_min = tZ.m_min;
			}
			else
			{
				overlapRange.m_min = tX.m_min;
			}
		}

		if (tX.m_max < tY.m_max)
		{
			if (tX.m_max < tZ.m_max)
			{
				overlapRange.m_max = tX.m_max;
			}
			else
			{
				overlapRange.m_max = tZ.m_max;
			}
		}
		else
		{
			if (tY.m_max < tZ.m_max)
			{
				overlapRange.m_max = tY.m_max;
			}
			else
			{
				overlapRange.m_max = tZ.m_max;
			}
		}

		Vec3 impactPoint = startPos + (overlapRange.m_min * fwdNormal * maxDist);

		raycast.m_didImpact = true;
		raycast.m_impactDist = GetDistance3D(impactPoint, startPos);
		raycast.m_impactPos = impactPoint;
		return raycast;
	}

	return raycast;
}

RaycastResult3D RaycastVsOBB3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, OBB3 const& box)
{
	Vec3 iBasis = box.m_iBasisNormal.GetNormalized();
	Vec3 jBasis = CrossProduct3D(Vec3(0.0f, 0.0f, 1.0f), iBasis).GetNormalized();
	Vec3 kBasis = CrossProduct3D(jBasis, iBasis).GetNormalized();

	Mat44 boxTransform;
	boxTransform.SetIJKT3D(iBasis, jBasis, kBasis, box.m_center);

	Mat44 worldToLocalOBBTransform = boxTransform.GetOrthonormalInverse();

	Vec3 localStartPos = worldToLocalOBBTransform.TransformPosition3D(startPos);
	Vec3 localFwdNormal = worldToLocalOBBTransform.TransformVectorQuantity3D(fwdNormal);

	AABB3 localBox = AABB3(-1.0f * box.m_halfDimensions, box.m_halfDimensions);

	RaycastResult3D raycast = RaycastVsAABB3D(localStartPos, localFwdNormal, maxDist, localBox);

	raycast.m_rayStartPos = startPos;
	raycast.m_rayFwdNormal = fwdNormal;
	raycast.m_impactPos = boxTransform.TransformPosition3D(raycast.m_impactPos);
	raycast.m_impactNormal = boxTransform.TransformVectorQuantity3D(raycast.m_impactNormal);

	return raycast;
}

RaycastResult3D RaycastVsPlane3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, Plane3D const& plane)
{
	Vec3 planeCenter = plane.m_normal * plane.m_distanceFromOriginAlongNormal;

	RaycastResult3D raycast;
	raycast.m_rayStartPos = startPos;
	raycast.m_rayFwdNormal = fwdNormal;
	raycast.m_rayMaxLength = maxDist;

	Vec3 rayStart = startPos;
	Vec3 rayEnd = startPos + (fwdNormal * maxDist);

	Vec3 planeCenterVertexDisplacement1 = rayStart - planeCenter;
	Vec3 planeCenterVertexDisplacement2 = rayEnd - planeCenter;

	float dotProdVerts1 = DotProduct3D(planeCenterVertexDisplacement1, plane.m_normal.GetNormalized());
	float dotProdVerts2 = DotProduct3D(planeCenterVertexDisplacement2, plane.m_normal.GetNormalized());

	float signOfTotalValue = dotProdVerts1 * dotProdVerts2;

	if (signOfTotalValue < 0.0f)
	{
		raycast.m_didImpact = true;

		float rayStartAltitude = DotProduct3D(planeCenterVertexDisplacement1, plane.m_normal.GetNormalized());
		float rayEndAltitude = DotProduct3D(planeCenterVertexDisplacement2, plane.m_normal.GetNormalized());

		float totalAlt = fabsf(rayStartAltitude) + fabsf(rayEndAltitude);

		float parametric = fabsf(rayStartAltitude) / totalAlt;

		raycast.m_impactDist = maxDist * parametric;
		raycast.m_impactPos = startPos + (raycast.m_impactDist * fwdNormal);

		if (dotProdVerts1 > 0.0f)
		{
			raycast.m_impactNormal = plane.m_normal.GetNormalized();
		}
		else
		{
			raycast.m_impactNormal = -1.0f * plane.m_normal.GetNormalized();
		}
	}

	return raycast;
}

RaycastResult3D RaycastVsSphere3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, Vec3 sphereCenter, float sphereRadius)
{
	RaycastResult3D raycast;
	raycast.m_rayStartPos = startPos;
	raycast.m_rayFwdNormal = fwdNormal;
	raycast.m_rayMaxLength = maxDist;
	raycast.m_impactPos = startPos + (maxDist * fwdNormal);

	Vec3 centerVec = sphereCenter - startPos;

	float centerDistance = DotProduct3D(centerVec, fwdNormal);

	if(centerDistance <= 0.0f || centerDistance >= maxDist + sphereRadius)
		return raycast;

	Vec3 centerVecIBasis = centerDistance * fwdNormal;

	Vec3 centerVecJKBasis = centerVec - centerVecIBasis;

	if(centerVecJKBasis.GetLengthSquared() >= (sphereRadius * sphereRadius))
		return raycast;

	if (IsPointInsideSphere3D(startPos, sphereCenter, sphereRadius))
	{
		raycast.m_didImpact = true;
		raycast.m_impactDist = 0.0f;
		raycast.m_impactPos = startPos;
		raycast.m_impactNormal = -1.0f * fwdNormal;

		return raycast;
	}

	float a = sqrtf((sphereRadius * sphereRadius) - centerVecJKBasis.GetLengthSquared());

	float impactDist = centerDistance - a;

	if(impactDist >= maxDist)
		return raycast;

	raycast.m_didImpact = true;
	raycast.m_impactDist = centerDistance - a;
	raycast.m_impactPos = startPos + (raycast.m_impactDist * fwdNormal);
	raycast.m_impactNormal = (raycast.m_impactPos - sphereCenter).GetNormalized();

	return raycast;
}

RaycastResult3D RaycastVsZCylinder3D(Vec3 startPos, Vec3 fwdNormal, float maxDist, Vec3 cylinderStart, float height, float radius)
{
	RaycastResult3D raycast;
	raycast.m_rayStartPos = startPos;
	raycast.m_rayFwdNormal = fwdNormal;
	raycast.m_rayMaxLength = maxDist;
	raycast.m_impactPos = startPos + (maxDist * fwdNormal);

	RaycastResult2D raycast2D;

	raycast2D = RaycastVsDisc2D(Vec2(startPos.x, startPos.y), Vec2(fwdNormal.x, fwdNormal.y).GetNormalized(), Vec2((maxDist * fwdNormal).x, (maxDist * fwdNormal).y).GetLength(), Vec2(cylinderStart.x, cylinderStart.y), radius);

	if (raycast2D.m_didImpact)
	{
		float tXY = (raycast2D.m_impactPos - Vec2(startPos.x, startPos.y)).GetLength() / Vec2((maxDist * fwdNormal).x, (maxDist * fwdNormal).y).GetLength();

		float tZ;

		if (startPos.z < cylinderStart.z)
		{
			if (fwdNormal.z < 0.0f)
				return raycast;

			if (maxDist >= (cylinderStart.z - startPos.z))
			{
				tZ = (cylinderStart.z - startPos.z) / DotProduct3D((maxDist * fwdNormal), Vec3(0.0f, 0.0f, 1.0f));

				if (tZ > tXY)
				{
					Vec3 impactPoint;

						impactPoint.x = startPos.x + (tZ * (fwdNormal * maxDist).x);
					impactPoint.y = startPos.y + (tZ * (fwdNormal * maxDist).y);
					impactPoint.z = startPos.z + (tZ * (fwdNormal * maxDist).z);

					raycast.m_didImpact = true;
					raycast.m_impactDist = (impactPoint - startPos).GetLength();
					raycast.m_impactNormal = Vec3(0.0f, 0.0f, -1.0f);
					raycast.m_impactPos = impactPoint;

					return raycast;
				}
				else
				{
					Vec3 impactPoint;

					impactPoint.x = startPos.x + (tXY * (fwdNormal * maxDist).x);
					impactPoint.y = startPos.y + (tXY * (fwdNormal * maxDist).y);
					impactPoint.z = startPos.z + (tXY * (fwdNormal * maxDist).z);

					raycast.m_didImpact = true;
					raycast.m_impactDist = (impactPoint - startPos).GetLength();
					raycast.m_impactNormal = (impactPoint - Vec3(cylinderStart.x, cylinderStart.y, impactPoint.z)).GetNormalized();
					raycast.m_impactPos = impactPoint;

					return raycast;
				}
			}

			
		}
		else if (startPos.z > cylinderStart.z + height)
		{
			if (fwdNormal.z > 0.0f)
				return raycast;
			
			if (maxDist >= (cylinderStart.z + height - startPos.z))
			{
				tZ = fabsf(cylinderStart.z + height - startPos.z) / fabsf(DotProduct3D((maxDist * fwdNormal), Vec3(0.0f, 0.0f, -1.0f)));

				if (tZ > tXY)
				{
					Vec3 impactPoint;

					impactPoint.x = startPos.x + (tZ * (fwdNormal * maxDist).x);
					impactPoint.y = startPos.y + (tZ * (fwdNormal * maxDist).y);
					impactPoint.z = startPos.z + (tZ * (fwdNormal * maxDist).z);

					raycast.m_didImpact = true;
					raycast.m_impactDist = (impactPoint - startPos).GetLength();
					raycast.m_impactNormal = Vec3(0.0f, 0.0f, 1.0f);
					raycast.m_impactPos = impactPoint;

					return raycast;
				}
				else
				{
					Vec3 impactPoint;

					impactPoint.x = startPos.x + (tXY * (fwdNormal * maxDist).x);
					impactPoint.y = startPos.y + (tXY * (fwdNormal * maxDist).y);
					impactPoint.z = startPos.z + (tXY * (fwdNormal * maxDist).z);

					raycast.m_didImpact = true;
					raycast.m_impactDist = (impactPoint - startPos).GetLength();
					raycast.m_impactNormal = (impactPoint - Vec3(cylinderStart.x, cylinderStart.y, impactPoint.z)).GetNormalized();
					raycast.m_impactPos = impactPoint;

					return raycast;
				}
			}
		}
		else
		{
			Vec3 impactPoint;

			impactPoint.x = startPos.x + (tXY * (fwdNormal * maxDist).x);
			impactPoint.y = startPos.y + (tXY * (fwdNormal * maxDist).y);
			impactPoint.z = startPos.z + (tXY * (fwdNormal * maxDist).z);

			raycast.m_didImpact = true;
			raycast.m_impactDist = (impactPoint - startPos).GetLength();
			raycast.m_impactNormal = (impactPoint - Vec3(cylinderStart.x, cylinderStart.y, impactPoint.z)).GetNormalized();
			raycast.m_impactPos = impactPoint;

			return raycast;
		}
	}

	return raycast;
}
