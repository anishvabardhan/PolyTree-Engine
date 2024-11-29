#include "Engine/Math/OBB2.hpp"

OBB2::OBB2(Vec2 const& center, Vec2 const& iBasis, Vec2 const& halfDimensions)
	: m_center(center), m_iBasisNormal(iBasis), m_halfDimensions(halfDimensions)
{
}

OBB2::~OBB2()
{
}
