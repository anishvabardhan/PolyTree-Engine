#include "Engine/Math/FloatRange.hpp"

FloatRange const FloatRange::ZERO = FloatRange(0.0f, 0.0f);
FloatRange const FloatRange::ONE = FloatRange(1.0f, 1.0f);
FloatRange const FloatRange::ZERO_TO_ONE = FloatRange(0.0f, 1.0f);

FloatRange::FloatRange()
	: m_min(0.0f), m_max(0.0f)
{
}

FloatRange::FloatRange(float min, float max)
	: m_min(min), m_max(max)
{
}

FloatRange::~FloatRange()
{
}

bool FloatRange::IsOnRange(float const& referenceValue)
{
	if(referenceValue >= m_min && referenceValue <= m_max)
		return true;

	return false;
}

bool FloatRange::IsOverlappingWith(FloatRange const& range)
{
	return m_min <= range.m_max && m_max >= range.m_min;
}

bool FloatRange::operator==(FloatRange const& compare) const
{
	if(m_min == compare.m_min && m_max == compare.m_max)
		return true;

	return false;
}

bool FloatRange::operator!=(FloatRange const& compare) const
{
	if (m_min != compare.m_min && m_max != compare.m_max)
		return true;

	return false;
}

void FloatRange::operator=(FloatRange const& copyFrom)
{
	m_min = copyFrom.m_min;
	m_max = copyFrom.m_max;
}
