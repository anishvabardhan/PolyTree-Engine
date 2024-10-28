#include "Engine/Math/CubicBezierCurve2D.hpp"

#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/CubicHermiteCurve2D.hpp"
#include "Engine/Core/EngineCommon.hpp"

CubicBezierCurve2D::CubicBezierCurve2D(Vec2 startPos, Vec2 guidePos1, Vec2 guidePos2, Vec2 endPos)
{
	m_a = startPos;
	m_b = guidePos1;
	m_c = guidePos2;
	m_d = endPos;
}

CubicBezierCurve2D::CubicBezierCurve2D(CubicHermiteCurve2D const& fromHermite)
{
	m_a = fromHermite.m_startPos;
	m_b = fromHermite.m_startPos + fromHermite.m_startVelocity;
	m_c = fromHermite.m_endPos - fromHermite.m_endVelocity;
	m_d = fromHermite.m_endPos;
}

Vec2 CubicBezierCurve2D::EvaluateParametric(float parametricZeroToOne) const
{
	Vec2 parametric;
	
	parametric.x = ComputeCubicBezier1D(m_a.x, m_b.x, m_c.x, m_d.x, parametricZeroToOne);
	parametric.y = ComputeCubicBezier1D(m_a.y, m_b.y, m_c.y, m_d.y, parametricZeroToOne);

	return parametric;
}

float CubicBezierCurve2D::GetApproximateLength(int numSubDivisions) const
{
	float length = 0.0f;
	float scale = 1.0f / (float)numSubDivisions;
	Vec2 first = m_a;

	for (int index = 0; index < numSubDivisions; index++)
	{
		Vec2 temp = EvaluateParametric(index * scale);
		length += (temp - first).GetLength();
		first = temp;
	}

	return length;
}

Vec2 CubicBezierCurve2D::EvaluateApproximateLength(float distanceAlongCurve, int numSubDivisions) const
{
	UNUSED(distanceAlongCurve);
	UNUSED(numSubDivisions);

	return Vec2();
}
