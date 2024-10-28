#pragma once

#include <math.h>

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/AABB3.hpp"
#include "Engine/Math/OBB3.hpp"
#include "Engine/Math/Plane3D.hpp"

struct Vec3;
struct Vec4;
struct Mat44;
struct IntVec2;
struct AABB2;
struct OBB2;

enum class BillboardType
{
	NONE = -1,
	WORLD_UP_CAMERA_FACING,
	WORLD_UP_CAMERA_OPPOSING,
	FULL_CAMERA_FACING,
	FULL_CAMERA_OPPOSING,
	COUNT
};

constexpr float M_PI = 3.14159f;

float			GetClamped(float value, float minValue, float maxValue);
float			GetClampedZeroToOne(float value);
float			Interpolate(float start, float end, float fractionTowardEnd);
float			GetFractionWithinRange(float value, float rangeStart, float rangeEnd);
float			RangeMap(float inValue, float inStart, float inEnd, float outStart, float outEnd);
float			RangeMapClamped(float inValue, float inStart, float inEnd, float outStart, float outEnd);
int				RoundDownToInt(float value);

float			ConvertDegreesToRadians(float degrees);
float			ConvertRadiansToDegrees(float radians);
float			CosDegrees(float degrees);
float			SinDegrees(float degrees);
float			TanDegrees(float degrees);
float			Atan2Degrees(float y, float x);
float			GetShortestAngularDispDegrees(float startDegrees, float endDegrees);
float			GetTurnedTowardDegrees(float currentDegrees, float goalDegrees, float maxDeltaDegrees);
float			GetAngleDegreesBetweenVectors2D(Vec2 const& a, Vec2 const& b);

float			NormalizeByte(unsigned char byte);
unsigned char	DenormalizeByte(float normalizedValue);

float			DotProduct2D(Vec2 const& a, Vec2 const& b);
float			DotProduct2D(Vec3 const& a, Vec3 const& b);
float			DotProduct2D(Vec4 const& a, Vec4 const& b);
float			DotProduct3D(Vec3 const& a, Vec3 const& b);
float			DotProduct4D(Vec4 const& a, Vec4 const& b);
float			CrossProduct2D(Vec2 const& a, Vec2 const& b);
float			CrossProduct2D(Vec3 const& a, Vec3 const& b);
float			CrossProduct2D(Vec4 const& a, Vec4 const& b);
Vec3			CrossProduct3D(Vec3 const& a, Vec3 const& b);

float			GetDistance2D(Vec2 const& positionA, Vec2 const& positionB);
float			GetDistanceSquared2D(Vec2 const& positionA, Vec2 const& positionB);
float			GetDistance3D(Vec3 const& positionA, Vec3 const& positionB);
float			GetDistanceSquared3D(Vec3 const& positionA, Vec3 const& positionB);
float			GetDistanceXY3D(Vec3 const& positionA, Vec3 const& positionB);
float			GetDistanceXYSquared3D(Vec3 const& positionA, Vec3 const& positionB);
float			GetProjectedLength2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto);
Vec2 const		GetProjectedOnto2D(Vec2 const& vectorToProject, Vec2 const& vectorToProjectOnto);
int				GetTaxicabDistance2D(IntVec2 const& pointA, IntVec2 const& pointB);

bool			IsPointInsideDisc2D(Vec2 const& point, Vec2 const& discCenter, float discRadius);
bool			IsPointInsideAABB2D(Vec2 const& point, AABB2 const& box);
bool			IsPointInsideCapsule2D(Vec2 const& point, Vec2 const& boneStart, Vec2 const& boneEnd, float radius);
bool			IsPointInsideOBB2D(Vec2 const& point, OBB2 const& orientedBox);
bool			IsPointInsideOrientedSector2D(Vec2 const& point, Vec2 const& sectorTip, float sectorForwardDegrees, float sectorApertureDegrees, float sectorRadius);
bool			IsPointInsideDirectedSector2D(Vec2 const& point, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius);

bool			DoDiscsOverlap(Vec2 const& centerA, float const& radiusA, Vec2 const& centerB, float const& radiusB);
bool			DoSpheresOverlap(Vec3 const& centerA, float const& radiusA, Vec3 const& centerB, float const& radiusB);
bool			DoSphereAABB3Overlap(Vec3 const& centerA, float const& radiusA, AABB3 const& box);
bool			DoSphereZCylinderOverlap(Vec3 const& centerA, float const& radiusA, Vec3 const& cylinderStart, float const& cylinderHeight, float const& cylinderRadius);
bool			DoAABB3sOverlap(AABB3 const& box1, AABB3 const& box2);
bool			DoAABB3ZCylinerOverlap(AABB3 const& box1, Vec3 const& cylinderStart, float const& cylinderHeight, float const& cylinderRadius);
bool			DoZCylindersOverlap(Vec3 const& cylinderStartA, float const& cylinderHeightA, float const& cylinderRadiusA, Vec3 const& cylinderStartB, float const& cylinderHeightB, float const& cylinderRadiusB);
bool			DoOBB3PlaneOverlap(OBB3 const& box, Plane3D const& plane);

Vec2 const		GetNearestPointOnDisc2D(Vec2 const& referencePosition, Vec2 const discCenter, float discRadius);
Vec2 const		GetNearestPointOnAABB2D(Vec2 const& referencePosition, AABB2 const& box);
Vec2 const		GetNearestPointOnInfiniteLine2D(Vec2 const& referencePosition, Vec2 const& pointOnLine, Vec2 const& anotherPointOnLine);
Vec2 const		GetNearestPointOnLineSegment2D(Vec2 const& referencePosition, Vec2 const& lineSegStart, Vec2 const& lineSegEnd);
Vec2 const		GetNearestPointOnCapsule2D(Vec2 const& referencePosition, Vec2 const& boneStart, Vec2 const& boneEnd, float radius);
Vec2 const		GetNearestPointOnOBB2D(Vec2 const& referencePosition, OBB2 const& orientedBox);
Vec2 const		GetNearestPointOnSector2D(Vec2 const& referencePosition, Vec2 const& sectorTip, Vec2 const& sectorForwardNormal, float sectorApertureDegrees, float sectorRadius);

bool			IsPointInsideSphere3D(Vec3 const& point, Vec3 const& discCenter, float discRadius);
bool			IsPointInsideAABB3D(Vec3 const& point, AABB3 const& box);
bool			IsPointInsideZCylinder3D(Vec3 const& point, Vec3 const start, float radius, float height);

Vec3 const		GetNearestPointOnSphere3D(Vec3 const& referencePosition, Vec3 const sphereCenter, float discRadius);
Vec3 const		GetNearestPointOnAABB3D(Vec3 const& referencePosition, AABB3 const& box);
Vec3 const		GetNearestPointOnOBB3D(Vec3 const& referencePosition, OBB3 const& box);
Vec3 const		GetNearestPointOnZCylinder3D(Vec3 const& referencePosition, Vec3 const start, float radius, float height);
Vec3 const		GetNearestPointOnPlane3D(Vec3 const& referencePosition, Plane3D const& plane);

Mat44			GetBillboardMatrix(BillboardType type, Mat44 const& cameraMatrix, Vec3 const& billboardPosition, Vec2 const& billboardScale = Vec2(1.0f, 1.0f));

bool			PushDiscOutOfPoint2D(Vec2& mobileDiscCenter, float discRadius, Vec2 const& fixedPoint);
bool			PushDiscOutOfDisc2D(Vec2& mobileDiscCenter, float mobileDiscRadius, Vec2 const& fixedDiscCenter, float const& fixedDiscRadius);
bool			PushDiscsOutOfEachOther2D(Vec2& aCenter, float aRadius, Vec2& bCenter, float bRadius);
bool			PushDiscOutOfAABB2D(Vec2& mobileDiscCenter, float discRadius, AABB2 const& fixedBox);

void			TransformPosition2D(Vec2& posToTransform, float uniformScale, float rotationDegrees, Vec2 const& translation);
void			TransformPosition2D(Vec2& posToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation);
void			TransformPositionXY3D(Vec3& positionToTransform, float scaleXY, float zRotationDegrees, Vec2 const& translationXY);
void			TransformPositionXY3D(Vec3& positionToTransform, Vec2 const& iBasis, Vec2 const& jBasis, Vec2 const& translation);

float			SmoothStart2(float t);
float			SmoothStart3(float t);
float			SmoothStart4(float t);
float			SmoothStart5(float t);
float			SmoothStart6(float t);


float			SmoothStop2(float t);
float			SmoothStop3(float t);
float			SmoothStop4(float t);
float			SmoothStop5(float t);
float			SmoothStop6(float t);

float			SmoothStep3(float t);
float			SmoothStep5(float t);
float			Hesitate3(float t);
float			Hesitate5(float t);
float			CustomFunkyEasingFunction(float t);

float			ComputeCubicBezier1D(float A, float B, float C, float D, float t);
float			ComputeQuinticBezier1D(float A, float B, float C, float D, float E, float F, float t);