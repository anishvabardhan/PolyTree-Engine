#pragma once

class RandomNumberGenerator
{
	//unsigned int m_seed = 0;
	//int m_position = 0;
public:
				RandomNumberGenerator() = default;

	int			RollRandomIntLessThan(int maxNotInclusive);
	int			RollRandomIntInRange(int minInclusive, int maxInclusive);
	float		RollRandomFloatZeroToOne();
	float		RollRandomFloatInRange(float minInclusive, float maxInclusive);
};