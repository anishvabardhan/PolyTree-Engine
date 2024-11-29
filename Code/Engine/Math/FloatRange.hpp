#pragma once

struct FloatRange
{
	float m_min = 0.0f;
	float m_max = 0.0f;

	static FloatRange const ZERO;
	static FloatRange const ONE;
	static FloatRange const ZERO_TO_ONE;
public:
	FloatRange();
	FloatRange(float min, float max);
	~FloatRange();

	bool IsOnRange(float const& referenceValue);
	bool IsOverlappingWith(FloatRange const& range);

	bool operator==(FloatRange const& compare) const;
	bool operator!=(FloatRange const& compare) const;

	void operator=(FloatRange const& copyFrom);
};