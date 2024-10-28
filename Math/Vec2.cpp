#include "Engine/Math/Vec2.hpp"

#include "Engine/Core/StringUtils.hpp"
#include "Engine/Math/MathUtils.hpp"

#include <math.h>

Vec2 const Vec2::ZERO = Vec2(0.0f, 0.0f);
Vec2 const Vec2::ONE = Vec2(1.0f, 1.0f);

//-----------------------------------------------------------------------------------------------
Vec2::Vec2( Vec2 const& copy )
	: x( copy.x ), y( copy.y )
{
}

//-----------------------------------------------------------------------------------------------
Vec2::Vec2( float initialX, float initialY )
	: x( initialX ), y( initialY )
{
}

Vec2 const Vec2::MakeFromPolarRadians(float orientationRadians, float length)
{
	Vec2 vec;

	float degrees = ConvertRadiansToDegrees(orientationRadians);

	vec.x = length * CosDegrees(degrees);
	vec.y = length * SinDegrees(degrees);

	return vec;
}

Vec2 const Vec2::MakeFromPolarDegrees(float orientationDegrees, float length)
{
	Vec2 vec;

	vec.x = length * CosDegrees(orientationDegrees);
	vec.y = length * SinDegrees(orientationDegrees);

	return vec;
}

float Vec2::GetLength() const
{
	float length = sqrtf((x * x) + (y * y));

	return length;
}

float Vec2::GetLengthSquared() const
{
	float length = (x * x) + (y * y);

	return length;
}

float Vec2::GetOrientationRadians() const
{
	float degrees = Atan2Degrees(y, x);

	float radians = ConvertDegreesToRadians(degrees);

	return radians;
}

float Vec2::GetOrientationDegrees() const
{
	float degrees = Atan2Degrees(y, x);

	return degrees;
}

Vec2 const Vec2::GetRotated90Degrees() const
{
	return Vec2(-y, x);
}

Vec2 const Vec2::GetRotatedMinus90Degrees() const
{
	return Vec2(y, -x);
}

Vec2 const Vec2::GetRotatedRadians(float deltaRadians) const
{
	float degrees = Atan2Degrees(y, x);
	float radians = ConvertDegreesToRadians(degrees);

	radians += deltaRadians;

	degrees = ConvertRadiansToDegrees(radians);

	float length = GetLength();

	return Vec2(length * CosDegrees(degrees), length * SinDegrees(degrees));
}

Vec2 const Vec2::GetRotatedDegrees(float deltaDegrees) const
{
	float degrees = Atan2Degrees(y, x);

	degrees += deltaDegrees;

	float length = GetLength();

	return Vec2(length * CosDegrees(degrees), length * SinDegrees(degrees));
}

Vec2 const Vec2::GetClamped(float maxLength) const
{
	Vec2 vec;

	float degrees = Atan2Degrees(y, x);

	float length = GetLength();

	if(length > maxLength)
		length = maxLength;

	vec.x = length * CosDegrees(degrees);
	vec.y = length * SinDegrees(degrees);

	return vec;
}

Vec2 const Vec2::GetNormalized() const
{
	Vec2 vec;

	float length = GetLengthSquared();

	if (length > 0.0f)
	{
		float scale = 1.0f / sqrtf(length);

		vec.x = x * scale;
		vec.y = y * scale;
	}
	else
	{
		vec = Vec2(1.0f, 0.0f);
	}


	return vec;
}

Vec2 const Vec2::GetReflected(Vec2 const& impactSurfacNormal) const
{
	Vec2 vec = *this;
	Vec2 normalVec = impactSurfacNormal * DotProduct2D(vec, impactSurfacNormal);

	Vec2 tangentVec = vec - normalVec;

	Vec2 normalReflected = -1.0f * normalVec;

	return normalReflected + tangentVec;
}

void Vec2::Reflect(Vec2 const& impactSurfaceNormal)
{
	Vec2 vec = GetReflected(impactSurfaceNormal);

	x = vec.x;
	y = vec.y;
}

void Vec2::SetOrientationRadians(float newOrientationRadians)
{
	float degrees = ConvertRadiansToDegrees(newOrientationRadians);

	float length = GetLength();

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

void Vec2::SetOrientationDegrees(float newOrientationDegrees)
{
	float length = GetLength();

	x = length * CosDegrees(newOrientationDegrees);
	y = length * SinDegrees(newOrientationDegrees);
}

void Vec2::SetPolarRadians(float newOrientationRadians, float length)
{
	float radians = newOrientationRadians;

	float degrees = ConvertRadiansToDegrees(radians);

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

void Vec2::SetPolarDegrees(float newOrientationDegrees, float length)
{
	float degrees = newOrientationDegrees;

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

Vec2 Vec2::SetFromText(char const* text)
{
	Strings splitValues = SplitStringOnDelimiter(text, ',');

	float xValue = static_cast<float>(std::atof(splitValues[0].c_str()));
	float yValue = static_cast<float>(std::atof(splitValues[1].c_str()));

	x = xValue;
	y = yValue;

	return Vec2(x, y);
}

void Vec2::Rotate90Degrees()
{
	float degrees = Atan2Degrees(y, x);

	degrees += 90.0f;

	float length = GetLength();

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

void Vec2::RotateMinus90Degrees()
{
	float degrees = Atan2Degrees(y, x);

	degrees -= 90.0f;

	float length = GetLength();

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

void Vec2::RotateRadians(float deltaRadians)
{
	float degrees = Atan2Degrees(y, x);

	float radians = ConvertDegreesToRadians(degrees);

	radians += deltaRadians;

	degrees = ConvertRadiansToDegrees(radians);

	float length = GetLength();

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

void Vec2::RotateDegrees(float deltaDegrees)
{
	float degrees = Atan2Degrees(y, x);

	degrees += deltaDegrees;

	float length = GetLength();

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

void Vec2::SetLength(float newLength)
{
	float length = newLength;

	float degrees = Atan2Degrees(y, x);

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

void Vec2::ClampLength(float maxLength)
{
	float length = GetLength();

	if(length > maxLength)
		length = maxLength;

	float degrees = Atan2Degrees(y, x);

	x = length * CosDegrees(degrees);
	y = length * SinDegrees(degrees);
}

void Vec2::Normalize()
{
	float length = GetLength();

	x /= length;
	y /= length;
}

float Vec2::NormalizeAndGetPreviousLength()
{
	float length = GetLength();

	x /= length;
	y /= length;

	return length;
}


//-----------------------------------------------------------------------------------------------
const Vec2 Vec2::operator + ( Vec2 const& vecToAdd ) const
{
	return Vec2( x + vecToAdd.x, y + vecToAdd.y );
}


//-----------------------------------------------------------------------------------------------
const Vec2 Vec2::operator-( Vec2 const& vecToSubtract ) const
{
	return Vec2( x - vecToSubtract.x, y - vecToSubtract.y );
}


//------------------------------------------------------------------------------------------------
const Vec2 Vec2::operator-() const
{
	return Vec2( -x, -y );
}


//-----------------------------------------------------------------------------------------------
const Vec2 Vec2::operator*( float uniformScale ) const
{
	return Vec2( x * uniformScale, y * uniformScale );
}


//------------------------------------------------------------------------------------------------
const Vec2 Vec2::operator*( Vec2 const& vecToMultiply ) const
{
	return Vec2( x * vecToMultiply.x, y * vecToMultiply.y );
}


//-----------------------------------------------------------------------------------------------
const Vec2 Vec2::operator/( float inverseScale ) const
{
	return Vec2( x / inverseScale, y / inverseScale );
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator+=( Vec2 const& vecToAdd )
{
	x += vecToAdd.x;
	y += vecToAdd.y;
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator-=( Vec2 const& vecToSubtract )
{
	x -= vecToSubtract.x;
	y -= vecToSubtract.y;
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator*=( float const uniformScale )
{
	x *= uniformScale;
	y *= uniformScale;
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator/=( float const uniformDivisor )
{
	x /= uniformDivisor;
	y /= uniformDivisor;
}


//-----------------------------------------------------------------------------------------------
void Vec2::operator=( Vec2 const& copyFrom )
{
	x = copyFrom.x;
	y = copyFrom.y;
}


//-----------------------------------------------------------------------------------------------
const Vec2 operator*( float uniformScale, Vec2 const& vecToScale )
{
	return Vec2( vecToScale.x * uniformScale, vecToScale.y * uniformScale );
}


//-----------------------------------------------------------------------------------------------
bool Vec2::operator==( Vec2 const& compare ) const
{
	if (x == compare.x && y == compare.y)
		return true;

	return false;
}


//-----------------------------------------------------------------------------------------------
bool Vec2::operator!=( Vec2 const& compare ) const
{
	if (x != compare.x || y != compare.y)
		return true;

	return false;
}


