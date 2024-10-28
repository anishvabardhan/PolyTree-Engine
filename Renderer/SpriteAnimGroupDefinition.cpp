#include "Engine/Renderer/SpriteAnimGroupDefinition.hpp"

#include "Engine/Renderer/SpriteAnimDefinition.hpp"

SpriteAnimGroupDefinition::SpriteAnimGroupDefinition()
{
	m_name = "";
	m_scaleBySpeed = false;
	m_secondsPerFrame = 0.0f;
	m_playbackMode = "";
}

SpriteAnimGroupDefinition::~SpriteAnimGroupDefinition()
{
}
