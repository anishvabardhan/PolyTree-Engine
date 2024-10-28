#include "OBB3.hpp"

OBB3::OBB3(Vec3 const& center, Vec3 const& iBasis, Vec3 const& halfDimensions)
	: m_center(center), m_iBasisNormal(iBasis), m_halfDimensions(halfDimensions)
{
}

OBB3::~OBB3()
{
}
