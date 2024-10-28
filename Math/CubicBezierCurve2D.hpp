#pragma once

#include "Engine/Math/Vec2.hpp"

class CubicHermiteCurve2D;

class CubicBezierCurve2D
{
public:
	Vec2 m_a;
	Vec2 m_b;
	Vec2 m_c;
	Vec2 m_d;
public:
	CubicBezierCurve2D(Vec2 startPos, Vec2 guidePos1, Vec2 guidePos2, Vec2 endPos);
	explicit CubicBezierCurve2D(CubicHermiteCurve2D const& fromHermite);

	Vec2 EvaluateParametric(float parametricZeroToOne) const;
	float GetApproximateLength(int numSubDivisions = 64) const;
	Vec2 EvaluateApproximateLength(float distanceAlongCurve, int numSubDivisions = 64) const;
};