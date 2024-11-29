#pragma once

#include <vector>

#include "Engine/Math/Vec2.hpp"

typedef std::vector<std::vector<float>> NoiseMap;

struct Noise
{
	Noise() = default;
	~Noise() = default;

	static NoiseMap GenerateNoiseMap(int mapWidth, int mapHeight, int seed, float scale, int octaves, float persistance, float lacunarity, Vec2 offset);
};