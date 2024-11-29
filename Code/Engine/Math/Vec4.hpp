#pragma once

//-----------------------------------------------------------------------------------------------
struct Vec4
{
public: // NOTE: this is one of the few cases where we break both the "m_" naming rule AND the avoid-public-members rule
	float				x = 0.0f;
	float				y = 0.0f;
	float				z = 0.0f;
	float				w = 0.0f;

	static Vec4 const ZERO;
	static Vec4 const ONE;
public:
	Vec4() = default;
	explicit			Vec4(float initialX, float initialY, float initialZ, float initialW);

	bool				operator==(Vec4 const& compare) const;
	bool				operator!=(Vec4 const& compare) const;
	Vec4 const			operator+(Vec4 const& vecToAdd) const;
	Vec4 const			operator-(Vec4 const& vecToSubstract) const;
	Vec4 const			operator*(float const& uniformScale) const;
	Vec4 const			operator/(float const& inverseScale) const;
	void				operator*=(float const& uniformScale);
};