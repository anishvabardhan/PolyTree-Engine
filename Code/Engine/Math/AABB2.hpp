#pragma once

#include "Engine/Math/Vec2.hpp"

struct AABB2
{
public:
	Vec2			m_mins;
	Vec2			m_maxs;
	static AABB2 const ZERO_TO_ONE;
public:
					AABB2() {}
					AABB2(AABB2 const& copyFrom);
	explicit		AABB2(float minX, float minY, float maxX, float maxY);
	explicit		AABB2(Vec2 const& mins, Vec2 const& maxs);
					~AABB2() {}

	bool			IsPointInside(Vec2 const& point) const;
	Vec2 const		GetCenter() const;
	Vec2 const		GetDimensions() const;
	Vec2 const		GetNearestPoint(Vec2 const& referencePosition) const;
	Vec2 const		GetPointAtUV(Vec2 const& uv) const;
	Vec2 const		GetUVForPoint(Vec2 const& point) const;

	void			Translate(Vec2 const& translationToApply);
	void			SetCenter(Vec2 const& newCenter);
	void			SetDimensions(Vec2 const& newDimensions);
	void			StretchToIncludePoint(Vec2 const& point);
};