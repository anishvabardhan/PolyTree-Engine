#include "Spline.hpp"

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/CubicHermiteCurve2D.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Core/EngineCommon.hpp"

Spline::Spline(std::vector<Vec2> splinePoints)
{
	m_splinePoints = splinePoints;

	for (size_t index = 0; index < m_splinePoints.size(); index++)
	{
		m_splineVelocities.push_back(Vec2());
	}

	for (size_t index = 1; index < m_splineVelocities.size() - (size_t)1; index++)
	{
		Vec2 velocity = (m_splinePoints[index + 1] - m_splinePoints[index - 1]) * 0.5f;
		m_splineVelocities[index] = velocity;
	}
}

Spline::~Spline()
{
}

Vec2 Spline::EvaluateParametric(float parametricZeroToOne) const
{
	int index = RoundDownToInt(parametricZeroToOne);
	float t  = parametricZeroToOne - (float)index;

	Vec2 parametric;

	if (index > (int)m_splinePoints.size() - 2)
	{
		index = (int)m_splinePoints.size() - 2;
	}

	CubicHermiteCurve2D hermite = CubicHermiteCurve2D(m_splinePoints[(size_t)index], m_splineVelocities[(size_t)index], m_splinePoints[size_t(index + 1)], m_splineVelocities[size_t(index + 1)]);

	parametric = hermite.EvaluateParametric(t);

	return parametric;
}

float Spline::GetApproximateLength(int numSubDivisions) const
{
	UNUSED(numSubDivisions);

	return 0.0f;
}

Vec2 Spline::EvaluateApproximateLength(float distanceAlongCurve, int numSubDivisions) const
{
	UNUSED(distanceAlongCurve);
	UNUSED(numSubDivisions);

	return Vec2();
}
