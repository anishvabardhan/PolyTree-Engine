#pragma once

#include "Engine/Math/Vec2.hpp"

class CubicBezierCurve2D;

class CubicHermiteCurve2D
{
public:
	Vec2 m_startPos;
	Vec2 m_startVelocity;
	Vec2 m_endPos;
	Vec2 m_endVelocity;
public:
	CubicHermiteCurve2D(Vec2 startPos, Vec2 startVel, Vec2 endPos, Vec2 endVel);
	explicit CubicHermiteCurve2D(CubicBezierCurve2D const& fromBezier);

	Vec2 EvaluateParametric(float parametricZeroToOne) const;
	float GetApproximateLength(int numSubDivisions = 16) const;
	Vec2 EvaluateApproximateLength(float distanceAlongCurve, int numSubDivisions = 16) const;
};