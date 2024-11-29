#pragma once

struct IntRange
{
	float m_min = 0.0f;
	float m_max = 0.0f;

	static IntRange const ZERO;
	static IntRange const ONE;
	static IntRange const ZERO_TO_ONE;
public:
	IntRange();
	IntRange(float min, float max);
	~IntRange();

	bool IsOnRange(float const& referenceValue);
	bool IsOverlappingWith(IntRange const& range);

	bool operator==(IntRange const& compare) const;
	bool operator!=(IntRange const& compare) const;

	void operator=(IntRange const& copyFrom);
};