#pragma once

#include "Engine/Math/Vec2.hpp"

struct OBB2
{
	Vec2 m_center;
	Vec2 m_iBasisNormal;
	Vec2 m_halfDimensions;
public:
	OBB2() = default;
	OBB2(Vec2 const& center, Vec2 const& iBasis, Vec2 const& halfDimensions);
	~OBB2();
};