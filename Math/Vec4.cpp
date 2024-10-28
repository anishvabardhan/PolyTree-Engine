#include "Engine/Math/Vec4.hpp"

Vec4 const Vec4::ZERO = Vec4(0.0f, 0.0f, 0.0f, 0.0f);
Vec4 const Vec4::ONE = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

Vec4::Vec4(float initialX, float initialY, float initialZ, float initialW)
{
	x = initialX;
	y = initialY;
	z = initialZ;
	w = initialW;
}

bool Vec4::operator==(Vec4 const& compare) const
{
	return (x == compare.x && y == compare.y && z == compare.z && w == compare.w);
}

bool Vec4::operator!=(Vec4 const& compare) const
{
	return (x != compare.x || y != compare.y || z != compare.z || w != compare.w);
}

Vec4 const Vec4::operator+(Vec4 const& vecToAdd) const
{
	Vec4 vec;

	vec.x = x + vecToAdd.x;
	vec.y = y + vecToAdd.y;
	vec.z = z + vecToAdd.z;
	vec.w = w + vecToAdd.w;

	return vec;
}

Vec4 const Vec4::operator-(Vec4 const& vecToSubstract) const
{
	Vec4 vec;

	vec.x = x - vecToSubstract.x;
	vec.y = y - vecToSubstract.y;
	vec.z = z - vecToSubstract.z;
	vec.w = w - vecToSubstract.w;

	return vec;
}

Vec4 const Vec4::operator*(float const& uniformScale) const
{
	Vec4 vec;

	vec.x = x * uniformScale;
	vec.y = y * uniformScale;
	vec.z = z * uniformScale;
	vec.w = w * uniformScale;

	return vec;
}

Vec4 const Vec4::operator/(float const& inverseScale) const
{
	Vec4 vec;

	vec.x = x / inverseScale;
	vec.y = y / inverseScale;
	vec.z = z / inverseScale;
	vec.w = w / inverseScale;

	return vec;
}

void Vec4::operator*=(float const& uniformScale)
{
	x *= uniformScale;
	y *= uniformScale;
	z *= uniformScale;
	w *= uniformScale;
}
