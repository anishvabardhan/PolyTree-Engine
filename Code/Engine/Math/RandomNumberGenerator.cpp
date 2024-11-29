#include "RandomNumberGenerator.hpp"

#include <cstdlib>

int RandomNumberGenerator::RollRandomIntLessThan(int maxNotInclusive)
{
	return rand() % maxNotInclusive;
}

int RandomNumberGenerator::RollRandomIntInRange(int minInclusive, int maxInclusive)
{
	int range = 1 + maxInclusive - minInclusive;
 	return static_cast<int>(rand() % range) + minInclusive;
}

float RandomNumberGenerator::RollRandomFloatZeroToOne()
{
	return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

float RandomNumberGenerator::RollRandomFloatInRange(float minInclusive, float maxInclusive)
{  
	int randomInt = rand();
	float randFloat = static_cast<float>(randomInt) / static_cast<float>(RAND_MAX);
	float range = maxInclusive - minInclusive;
	return (randFloat * range) + minInclusive;
}
