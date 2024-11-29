#include "SpriteAnimDefinition.hpp"

SpriteAnimDefinition::SpriteAnimDefinition(const SpriteSheet& sheet, int startSpriteIndex, int endSpriteIndex, float durationSeconds, SpriteAnimPlaybackType playbackType)
	: m_spriteSheet(sheet), m_startSpriteIndex(startSpriteIndex), m_endSpriteIndex(endSpriteIndex), m_durationSeconds(durationSeconds), m_playbackType(playbackType)
{
	m_elapsedTime = 0.0f;
	m_isReverseAnim = false;
}

void SpriteAnimDefinition::Update(float deltaseconds)
{
	switch (m_playbackType)
	{
	case SpriteAnimPlaybackType::ONCE:
		if (m_elapsedTime < m_durationSeconds - deltaseconds)
		{
			m_elapsedTime += deltaseconds;
		}
		break;
	case SpriteAnimPlaybackType::LOOP:
		if (m_elapsedTime < m_durationSeconds - deltaseconds)
		{
			m_elapsedTime += deltaseconds;
		}
		else
		{
			m_elapsedTime = 0.0f;
		}
		break;
	case SpriteAnimPlaybackType::PINGPONG:
		if (!m_isReverseAnim)
		{
			if (m_elapsedTime < m_durationSeconds - deltaseconds)
			{
				m_elapsedTime += deltaseconds;
			}
			else
			{
				m_isReverseAnim = true;
			}
		}
		else if (m_isReverseAnim)
		{
			if (m_elapsedTime > deltaseconds)
			{
				m_elapsedTime -= deltaseconds;
			}
			else
			{
				m_isReverseAnim = false;
			}
		}
	}
}

void SpriteAnimDefinition::AddElapsedTime(float seconds)
{
	m_elapsedTime += seconds;
}

float SpriteAnimDefinition::GetElapsedTime()
{
	return m_elapsedTime;
}

float SpriteAnimDefinition::GetDuration()
{
	return m_durationSeconds;
}

void SpriteAnimDefinition::SetElapsedTime(float seconds)
{
	m_elapsedTime = seconds;
}

SpriteDefinition const& SpriteAnimDefinition::GetSpriteDefAtTime(float seconds) const
{
	int totalSprites = m_endSpriteIndex - m_startSpriteIndex + 1;

	float secondsPerSprite = m_durationSeconds / static_cast<float>(totalSprites);

	int currentSpriteIndex = m_startSpriteIndex + static_cast<int>(seconds / secondsPerSprite);

	if(currentSpriteIndex <= (int)m_spriteSheet.m_spriteDefs.size() - 1)
		return m_spriteSheet.GetSpriteDef(currentSpriteIndex);

	return m_spriteSheet.GetSpriteDef(m_startSpriteIndex);
}

bool SpriteAnimDefinition::LoadFromXmlElement(XmlElement const& element)
{
	m_startSpriteIndex = ParseXmlAttribute(element, "startFrame", -1);
	m_endSpriteIndex = ParseXmlAttribute(element, "endFrame", -1);

	if(m_startSpriteIndex != -1 && m_endSpriteIndex != -1)
		return true;

	return false;
}
