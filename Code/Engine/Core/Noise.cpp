#include "Noise.hpp"

#include "ThirdParty/Squirrel/SmoothNoise.hpp"

#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/RandomNumberGenerator.hpp"

NoiseMap Noise::GenerateNoiseMap(int mapWidth, int mapHeight, int seed, float scale, int octaves, float persistance, float lacunarity, Vec2 offset)
{
	if(mapWidth < 1)
		mapWidth = 1;

	if(mapHeight < 1)
		mapHeight = 1;

	if(lacunarity < 1)
		lacunarity = 1;

	if(octaves < 0)
		octaves = 0;

	// Initialize noise map array;
	NoiseMap noiseMap;
	noiseMap.resize(mapHeight);

	for (int height = 0; height < mapHeight; height++)
	{
		noiseMap[height].resize(mapWidth);
	}

	RandomNumberGenerator rng = RandomNumberGenerator();

	Vec2* octaveOffsets = new Vec2[octaves];

	for (int i = 0; i < octaves; i++)
	{
		float offsetX = rng.RollRandomFloatInRange(-1000 * (float)seed, 1000 * (float)seed) + offset.x;
		float offsetY = rng.RollRandomFloatInRange(-1000 * (float)seed, 1000 * (float)seed) + offset.y;
		octaveOffsets[i] = Vec2(offsetX, offsetY);
	}

	// If scale is less than 0, clamp it to a minute value
	if(scale <= 0)
		scale = 0.0001f;

	float maxNoiseHeight = FLT_MIN;
	float minNoiseHeight = FLT_MAX;

	float halfWidth = mapWidth * 0.5f;
	float halfHeight = mapHeight * 0.5f;

	float factor = 1.0f / scale;

	// Generate the noise map values;
	for (int y = 0; y < mapHeight; y++)
	{
		for (int x = 0; x < mapWidth; x++)
		{
			float amplitude = 1.0f;
			float frequency = 1.0f;
			float noiseHeight = 0.0f;

			for (int i = 0; i < octaves; i++)
			{
				float sampleX = (x - halfWidth) * factor * frequency + octaveOffsets[i].x;
				float sampleY = (y - halfHeight) * factor * frequency + octaveOffsets[i].y;

				float perlinValue = Compute2dPerlinNoise(sampleX, sampleY) * 2.0f - 1.0f;
				noiseHeight += perlinValue * amplitude;

				amplitude *= persistance;
				frequency *= lacunarity;
			}

			if (noiseHeight > maxNoiseHeight)
			{
				maxNoiseHeight = noiseHeight;
			}
			else if (noiseHeight < minNoiseHeight)
			{
				minNoiseHeight = noiseHeight;
			}

			noiseMap[y][x] = noiseHeight;
		}
	}

	for (int y = 0; y < mapHeight; y++)
	{
		for (int x = 0; x < mapWidth; x++)
		{
			noiseMap[y][x] = RangeMapClamped(noiseMap[y][x], minNoiseHeight, maxNoiseHeight, 0.0f, 1.0f);
		}
	}

	return noiseMap;
}
