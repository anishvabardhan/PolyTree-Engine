#include "IntRange.hpp"

IntRange const IntRange::ZERO = IntRange(0, 0);
IntRange const IntRange::ONE = IntRange(1, 1);
IntRange const IntRange::ZERO_TO_ONE = IntRange(0, 1);

IntRange::IntRange()
	: m_min(0), m_max(0)
{
}

IntRange::IntRange(float min, float max)
	: m_min(min), m_max(max)
{
}

IntRange::~IntRange()
{
}

bool IntRange::IsOnRange(float const& referenceValue)
{
	if(referenceValue >= m_min && referenceValue <= m_max)
		return true;

	return false;
}

bool IntRange::IsOverlappingWith(IntRange const& range)
{
	if(IsOnRange(range.m_min) || IsOnRange(range.m_max))
		return true;

	return false;
}

bool IntRange::operator==(IntRange const& compare) const
{
	if(m_min == compare.m_min && m_max == compare.m_max)
		return true;

	return false;
}

bool IntRange::operator!=(IntRange const& compare) const
{
	if (m_min != compare.m_min && m_max != compare.m_max)
		return true;

	return false;
}

void IntRange::operator=(IntRange const& copyFrom)
{
	m_min = copyFrom.m_min;
	m_max = copyFrom.m_max;
}
