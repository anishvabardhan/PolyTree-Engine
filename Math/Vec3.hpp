#pragma once

//-----------------------------------------------------------------------------------------------
struct Vec3
{
public: // NOTE: this is one of the few cases where we break both the "m_" naming rule AND the avoid-public-members rule
	float				x = 0.0f;
	float				y = 0.0f;
	float				z = 0.0f;

	static Vec3 const ZERO;
	static Vec3 const ONE;
public:
						Vec3() = default;
	explicit			Vec3(float initialX, float initialY, float initialZ);

	static Vec3 const	MakeFromPolarRadians(float latitudeRadians, float longitudeRadians, float length = 1.0f);
	static Vec3 const	MakeFromPolarDegrees(float latitudeDegrees, float longitudeDegrees, float length = 1.0f);

	float				GetLength() const;
	float				GetLengthXY() const;
	float				GetLengthSquared() const;
	float				GetLengthXYSquared() const;
	float				GetAngleAboutZRadians() const;
	float				GetAngleAboutZDegrees() const;
	Vec3 const			GetRotatedAboutZRadians(float deltaRadians) const;
	Vec3 const			GetRotatedAboutZDegrees(float deltaDegrees) const;
	Vec3 const			GetClamped(float maxLength) const;
	Vec3 const			GetNormalized() const;
	void				SetLength(float newLength);
	Vec3				SetFromText(char const* text);

	bool				operator==(Vec3 const& compare) const;
	bool				operator!=(Vec3 const& compare) const;
	Vec3 const			operator+(Vec3 const& vecToAdd) const;
	Vec3 const			operator-(Vec3 const& vecToSubstract) const;
	Vec3 const			operator*(float const& uniformScale) const;
	Vec3 const			operator*(Vec3 const& vecToMultiply) const;
	Vec3 const			operator/(float const& inverseScale) const;

	void				operator+=(Vec3 const& vecToAdd);
	void				operator-=(Vec3 const& vecToSubstract);
	void				operator*=(float const& uniformScale);
	void				operator/=(float const& uniformDivisor);
	void				operator=(Vec3 const& copyFrom);

	friend Vec3 const	operator*(float uniformScale, Vec3 const& vecToScale);
};