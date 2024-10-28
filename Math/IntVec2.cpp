#include "IntVec2.hpp"

#include "Engine/Core/StringUtils.hpp"
#include "Engine/Math/MathUtils.hpp"

#include <math.h>

IntVec2 const IntVec2::ZERO			= IntVec2(0, 0);
IntVec2 const IntVec2::ONE			= IntVec2(1, 1);
IntVec2 const IntVec2::NORTH		= IntVec2(0, 1);
IntVec2 const IntVec2::SOUTH		= IntVec2(0, -1);
IntVec2 const IntVec2::EAST			= IntVec2(1, 0);
IntVec2 const IntVec2::WEST			= IntVec2(-1, 0);
IntVec2 const IntVec2::NORTH_EAST	= IntVec2(1, 1);
IntVec2 const IntVec2::NORTH_WEST	= IntVec2(1, -1);
IntVec2 const IntVec2::SOUTH_EAST	= IntVec2(-1, 1);
IntVec2 const IntVec2::SOUTH_WEST	= IntVec2(-1, -1);

IntVec2::IntVec2(IntVec2 const& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
}

IntVec2::IntVec2(int initialX, int initialY)
{
	x = initialX;
	y = initialY;
}

float IntVec2::GetLength() const
{
	float length = sqrtf(static_cast<float>((x * x) + (y * y)));
	return length;
}

int IntVec2::GetTaxicabLength() const
{
	int length = abs(x) + abs(y);
	return length;
}

int IntVec2::GetLengthSquared() const
{
	int length = (x * x) + (y * y);
	return length;
}

float IntVec2::GetOrientationRadians() const
{
	float degrees = Atan2Degrees(static_cast<float>(y), static_cast<float>(x));

	float radians = ConvertDegreesToRadians(degrees);

	return radians;
}

float IntVec2::GetOrientationDegrees() const
{
	float degrees = Atan2Degrees(static_cast<float>(y), static_cast<float>(x));

	return degrees;
}

IntVec2 const IntVec2::GetRotated90Degrees() const
{
	return IntVec2(-y, x);
}

IntVec2 const IntVec2::GetRotatedMinus90Degrees() const
{
	return IntVec2(y, -x);
}

void IntVec2::Rotate90Degrees()
{
	IntVec2 temp = IntVec2(-y, x);

	x = temp.x;
	y = temp.y;
}

void IntVec2::RotateMinus90Degrees()
{
	IntVec2 temp = IntVec2(y, -x);

	x = temp.x;
	y = temp.y;
}

IntVec2 IntVec2::SetFromText(char const* text)
{
	Strings splitValues = SplitStringOnDelimiter(text, ',');

	int xValue = std::atoi(splitValues[0].c_str());
	int yValue = std::atoi(splitValues[1].c_str());

	x = xValue;
	y = yValue;

	return IntVec2(x, y);
}

IntVec2 const IntVec2::operator+(IntVec2 const& copyFrom)
{
	return IntVec2(x + copyFrom.x, y + copyFrom.y);
}

IntVec2 const IntVec2::operator-(IntVec2 const& copyFrom)
{
	return IntVec2(x - copyFrom.x, y - copyFrom.y);
}

bool IntVec2::operator==(IntVec2 const& compare) const
{
	if (x == compare.x && y == compare.y)
		return true;

	return false;
}

void IntVec2::operator=(IntVec2 const& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
}
