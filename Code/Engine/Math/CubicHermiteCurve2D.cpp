#include "CubicHermiteCurve2D.hpp"

#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/CubicBezierCurve2D.hpp"
#include "Engine/Core/EngineCommon.hpp"

CubicHermiteCurve2D::CubicHermiteCurve2D(Vec2 startPos, Vec2 startVel, Vec2 endPos, Vec2 endVel)
{
	m_startPos = startPos;
	m_startVelocity = startVel;
	m_endPos = endPos;
	m_endVelocity = endVel;
}

CubicHermiteCurve2D::CubicHermiteCurve2D(CubicBezierCurve2D const& fromBezier)
{
	m_startPos = fromBezier.m_a;
	m_startVelocity = fromBezier.m_b - fromBezier.m_a;
	m_endPos = fromBezier.m_d;
	m_endVelocity = fromBezier.m_d - fromBezier.m_c;
}

Vec2 CubicHermiteCurve2D::EvaluateParametric(float parametricZeroToOne) const
{
	Vec2 guidePos1 = m_startPos + (0.33f * m_startVelocity);
	Vec2 guidePos2 = m_endPos - (0.33f * m_endVelocity);

	Vec2 parametric;

	parametric.x = ComputeCubicBezier1D(m_startPos.x, guidePos1.x, guidePos2.x, m_endPos.x, parametricZeroToOne);
	parametric.y = ComputeCubicBezier1D(m_startPos.y, guidePos1.y, guidePos2.y, m_endPos.y, parametricZeroToOne);

	return parametric;
}

float CubicHermiteCurve2D::GetApproximateLength(int numSubDivisions) const
{
	UNUSED(numSubDivisions);

	return 0.0f;
}

Vec2 CubicHermiteCurve2D::EvaluateApproximateLength(float distanceAlongCurve, int numSubDivisions) const
{
	UNUSED(distanceAlongCurve);
	UNUSED(numSubDivisions);

	return Vec2();
}
