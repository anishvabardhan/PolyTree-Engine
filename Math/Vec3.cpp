#include "Engine/Math/Vec3.hpp"

#include "Engine/Core/StringUtils.hpp"
#include "Engine/Math/MathUtils.hpp"

#include <math.h>

Vec3 const Vec3::ZERO = Vec3(0.0f, 0.0f, 0.0f);
Vec3 const Vec3::ONE = Vec3(1.0f, 1.0f, 1.0f);

Vec3::Vec3(float initialX, float initialY, float initialZ)
	: x(initialX), y(initialY), z(initialZ)
{
}

Vec3 const Vec3::MakeFromPolarRadians(float latitudeRadians, float longitudeRadians, float length)
{
	float latitudeDegrees = ConvertRadiansToDegrees(latitudeRadians);
	float longitudeDegrees = ConvertRadiansToDegrees(longitudeRadians);

	float cy = CosDegrees(longitudeDegrees);
	float sy = SinDegrees(longitudeDegrees);

	float cp = CosDegrees(latitudeDegrees);
	float sp = SinDegrees(latitudeDegrees);

	Vec3 result;

	result.x = cy * cp * length;
	result.y = sy * cp * length;
	result.z = -1.0f * sp * length;

	return result;
}

Vec3 const Vec3::MakeFromPolarDegrees(float latitudeDegrees, float longitudeDegrees, float length)
{
	float cy = CosDegrees(longitudeDegrees);
	float sy = SinDegrees(longitudeDegrees);

	float cp = CosDegrees(latitudeDegrees);
	float sp = SinDegrees(latitudeDegrees);

	Vec3 result;

	result.x = cy * cp * length;
	result.y = sy * cp * length;
	result.z = -1.0f * sp * length;

	return result;
}

float Vec3::GetLength() const
{
	float length = sqrtf((x * x) + (y * y) + (z * z));

	return length;
}

float Vec3::GetLengthXY() const
{
	float length = sqrtf((x * x) + (y * y));

	return length;
}

float Vec3::GetLengthSquared() const
{
	float length = (x * x) + (y * y) + (z * z);

	return length;
}

float Vec3::GetLengthXYSquared() const
{
	float length = (x * x) + (y * y);

	return length;
}

float Vec3::GetAngleAboutZRadians() const
{
	float degrees = Atan2Degrees(y, x);

	float radians = ConvertDegreesToRadians(degrees);

	return radians;
}

float Vec3::GetAngleAboutZDegrees() const
{
	float degrees = Atan2Degrees(y, x);

	return degrees;
}

Vec3 const Vec3::GetRotatedAboutZRadians(float deltaRadians) const
{
	float degrees = Atan2Degrees(y, x);
	float radians = ConvertDegreesToRadians(degrees);

	radians += deltaRadians;

	degrees = ConvertRadiansToDegrees(radians);

	float length = GetLengthXY();

	return Vec3(length * CosDegrees(degrees), length * SinDegrees(degrees), z);
}

Vec3 const Vec3::GetRotatedAboutZDegrees(float deltaDegrees) const
{
	float degrees = Atan2Degrees(y, x);

	degrees += deltaDegrees;

	float length = GetLengthXY();

	return Vec3(length * CosDegrees(degrees), length * SinDegrees(degrees), z);
}

Vec3 const Vec3::GetClamped(float maxLength) const
{
	Vec3 temp;

	float length = GetLength();

	if(length > maxLength)
		length = maxLength;

	temp.x = length * CosDegrees(GetAngleAboutZDegrees());
	temp.y = length * SinDegrees(GetAngleAboutZDegrees());

	return temp;
}

Vec3 const Vec3::GetNormalized() const
{
	float rootOfSumOfSquares = GetLength();

	if ( rootOfSumOfSquares != 0 )
	{
		float normalizedX = x / rootOfSumOfSquares;
		float normalizedY = y / rootOfSumOfSquares;
		float normalizedZ = z / rootOfSumOfSquares;

		return Vec3( normalizedX, normalizedY, normalizedZ );
	}
	else
	{
		return Vec3( 0.0f, 0.0f, 0.0f );
	}
}

void Vec3::SetLength(float newLength)
{
	float length = newLength;

	float latDegrees = Atan2Degrees(z, x);
	float longDegrees = Atan2Degrees(y, x);

	Vec3 temp = MakeFromPolarDegrees(latDegrees, longDegrees, length);

	x = temp.x;
	y = temp.y;
	z = temp.z;
}

Vec3 Vec3::SetFromText(char const* text)
{
	Strings splitValues = SplitStringOnDelimiter(text, ',');

	float xValue = static_cast<float>(std::atof(splitValues[0].c_str()));
	float yValue = static_cast<float>(std::atof(splitValues[1].c_str()));
	float zValue = static_cast<float>(std::atof(splitValues[2].c_str()));

	x = xValue;
	y = yValue;
	z = zValue;

	return Vec3(x, y, z);
}

bool Vec3::operator==(Vec3 const& compare) const
{
	return (x == compare.x && y == compare.y && z == compare.z);
}

bool Vec3::operator!=(Vec3 const& compare) const
{
	return (x != compare.x || y != compare.y || z != compare.z);
}

Vec3 const Vec3::operator+(Vec3 const& vecToAdd) const
{
	Vec3 vec;

	vec.x = x + vecToAdd.x;
	vec.y = y + vecToAdd.y;
	vec.z = z + vecToAdd.z;

	return vec;
}

Vec3 const Vec3::operator-(Vec3 const& vecToSubstract) const
{
	Vec3 vec;

	vec.x = x - vecToSubstract.x;
	vec.y = y - vecToSubstract.y;
	vec.z = z - vecToSubstract.z;

	return vec;
}

Vec3 const Vec3::operator*(float const& uniformScale) const
{
	Vec3 vec;

	vec.x = x * uniformScale;
	vec.y = y * uniformScale;
	vec.z = z * uniformScale;

	return vec;
}

Vec3 const Vec3::operator*(Vec3 const& vecToMultiply) const
{
	Vec3 vec;

	vec.x = x * vecToMultiply.x;
	vec.y = y * vecToMultiply.y;
	vec.z = z * vecToMultiply.z;

	return vec;
}

Vec3 const Vec3::operator/(float const& inverseScale) const
{
	Vec3 vec;

	vec.x = x / inverseScale;
	vec.y = y / inverseScale;
	vec.z = z / inverseScale;

	return vec;
}

void Vec3::operator+=(Vec3 const& vecToAdd)
{
	x += vecToAdd.x;
	y += vecToAdd.y;
	z += vecToAdd.z;
}

void Vec3::operator-=(Vec3 const& vecToSubstract)
{
	x -= vecToSubstract.x;
	y -= vecToSubstract.y;
	z -= vecToSubstract.z;
}

void Vec3::operator*=(float const& uniformScale)
{
	x *= uniformScale;
	y *= uniformScale;
	z *= uniformScale;
}

void Vec3::operator/=(float const& uniformDivisor)
{
	x /= uniformDivisor;
	y /= uniformDivisor;
	z /= uniformDivisor;
}

void Vec3::operator=(Vec3 const& copyFrom)
{
	x = copyFrom.x;
	y = copyFrom.y;
	z = copyFrom.z;
}

Vec3 const operator*(float uniformScale, Vec3 const& vecToScale)
{
	return Vec3(vecToScale.x * uniformScale, vecToScale.y * uniformScale, vecToScale.z * uniformScale);
}
