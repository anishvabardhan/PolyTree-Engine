#pragma once

#include "Engine/Math/Vec2.hpp"

#include <vector>

class Spline
{
public:
	std::vector<Vec2> m_splinePoints;
	std::vector<Vec2> m_splineVelocities;
public:
	Spline(std::vector<Vec2> splinePoints);
	~Spline();

	Vec2 EvaluateParametric(float parametricZeroToOne) const;
	float GetApproximateLength(int numSubDivisions = 64) const;
	Vec2 EvaluateApproximateLength(float distanceAlongCurve, int numSubDivisions = 64) const;
};