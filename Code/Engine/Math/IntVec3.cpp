#include "IntVec3.hpp"

#include "Engine/Core/StringUtils.hpp"
#include "Engine/Math/MathUtils.hpp"

#include <math.h>

IntVec3 const IntVec3::ZERO			= IntVec3(0, 0, 0);
IntVec3 const IntVec3::ONE			= IntVec3(1, 1, 1);
IntVec3 const IntVec3::NORTH		= IntVec3(0, 1, 0);
IntVec3 const IntVec3::SOUTH		= IntVec3(0, -1, 0);
IntVec3 const IntVec3::EAST			= IntVec3(1, 0, 0);
IntVec3 const IntVec3::WEST			= IntVec3(-1, 0, 0);
IntVec3 const IntVec3::UP			= IntVec3(0, 0, 1);
IntVec3 const IntVec3::DOWN			= IntVec3(0, 0, -1);

IntVec3::IntVec3(IntVec3 const& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
	z = copyFrom.z;
}

IntVec3::IntVec3(int initialX, int initialY, int initialZ)
{
	x = initialX;
	y = initialY;
	z = initialZ;
}

float IntVec3::GetLength() const
{
	 float length = sqrtf(static_cast<float>((x * x) + (y * y) + (z * z)));
	return length;
}

IntVec3 const IntVec3::operator+(IntVec3 const& copyFrom)
{
	return IntVec3(x + copyFrom.x, y + copyFrom.y, z + copyFrom.z);
}

IntVec3 const IntVec3::operator-(IntVec3 const& copyFrom)
{
	return IntVec3(x - copyFrom.x, y - copyFrom.y, z - copyFrom.z);
}

bool IntVec3::operator==(IntVec3 const& compare) const
{
	return (x == compare.x && y == compare.y && z == compare.z);
}
