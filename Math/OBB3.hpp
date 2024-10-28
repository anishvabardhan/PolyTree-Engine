#pragma once

#include "Engine/Math/Vec3.hpp"

struct OBB3
{
	Vec3 m_center;
	Vec3 m_iBasisNormal;
	Vec3 m_halfDimensions;
public:
	OBB3() = default;
	OBB3(Vec3 const& center, Vec3 const& iBasis, Vec3 const& halfDimensions);
	~OBB3();
};