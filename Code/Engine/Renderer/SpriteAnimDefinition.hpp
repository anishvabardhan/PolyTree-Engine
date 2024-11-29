#pragma once

#include "Engine/Renderer/SpriteSheet.hpp"
#include "Engine/Core/XmlUtils.hpp"

enum class SpriteAnimPlaybackType
{
	ONCE,
	LOOP,
	PINGPONG,
};

class SpriteAnimDefinition
{
	const SpriteSheet&			m_spriteSheet;
	int							m_startSpriteIndex		= -1;
	int							m_endSpriteIndex		= -1;
	bool						m_isReverseAnim			= false;
	float						m_durationSeconds		= 1.f;
	float						m_elapsedTime			= 0.0f;
public:
	SpriteAnimPlaybackType		m_playbackType			= SpriteAnimPlaybackType::LOOP;
								SpriteAnimDefinition(const SpriteSheet& sheet, int startSpriteIndex, int endSpriteIndex,
								float durationSeconds, SpriteAnimPlaybackType playbackType = SpriteAnimPlaybackType::LOOP);

	void						Update(float deltaseconds);
	void						AddElapsedTime(float seconds);
	float						GetElapsedTime();
	float						GetDuration();
	void						SetElapsedTime(float seconds);
	SpriteDefinition const&		GetSpriteDefAtTime(float seconds) const;
	bool						LoadFromXmlElement(XmlElement const& element);
};
