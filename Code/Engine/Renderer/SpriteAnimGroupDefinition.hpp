#pragma once

#include "Engine/Math/Vec3.hpp"

#include <vector>
#include <string>

class SpriteAnimDefinition;

class SpriteAnimGroupDefinition
{
public:
	std::string m_name;
	bool m_scaleBySpeed = false;
	float m_secondsPerFrame;
	std::string m_playbackMode;
	std::vector<SpriteAnimDefinition*> m_spriteAnimDefs;
	std::vector<Vec3> m_spriteDirection;
public:
	SpriteAnimGroupDefinition();
	~SpriteAnimGroupDefinition();
};
