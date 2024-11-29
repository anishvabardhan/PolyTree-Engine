#include "Clock.hpp"

#include "Engine/Core/Time.hpp"
#include "Engine/Math/MathUtils.hpp"

static Clock* s_theSystemClock = new Clock();

Clock::Clock()
{
	if(this == s_theSystemClock)
		m_parent = nullptr;
	else
		m_parent = s_theSystemClock;
}

Clock::Clock(Clock& parent)
	: m_parent(&parent)
{
	m_parent->AddChild(this);
}

Clock::~Clock()
{
}

void Clock::Reset()
{
	m_lastUpdateTimeInSeconds = static_cast<float>(GetCurrentTimeSeconds());
	m_totalSeconds = 0.0f;
	m_deltaSeconds = 0.0f;
	m_frameCount = 0;

	m_timeScale = 1.0f;

	m_isPaused = false;
	m_stepSingleFrame = false;

	m_maxDeltaSeconds = 0.1f;
}

bool Clock::IsPaused() const
{
	return m_isPaused;
}

void Clock::Pause()
{
	m_isPaused = true;
}

void Clock::Unpause()
{
	m_isPaused = false;
}

void Clock::TogglePause()
{
	m_isPaused = !m_isPaused;
}

void Clock::StepSingleFrame()
{
	m_stepSingleFrame = true;
}

void Clock::SetTimeScale(float timeScale)
{
	m_timeScale = timeScale;
}

float Clock::GetTimeScale() const
{
	return m_timeScale;
}

float Clock::GetDeltaSeconds() const
{
	return m_deltaSeconds;
}

float Clock::GetTotalSeconds() const
{
	return m_totalSeconds;
}

size_t Clock::GetFrameCount() const
{
	return m_frameCount;
}

Clock& Clock::GetSystemClock()
{
	return *s_theSystemClock;
}

void Clock::TickSystemClock()
{
	s_theSystemClock->Tick();
}

void Clock::Tick()
{
	float currentTime = (float)GetCurrentTimeSeconds();

	float frameTime = currentTime - m_lastUpdateTimeInSeconds;

	frameTime = GetClamped(frameTime, 0.0f, m_maxDeltaSeconds);

	Advance(frameTime);

	m_lastUpdateTimeInSeconds = currentTime;
}

void Clock::Advance(float deltaTimeSeconds)
{
	if (m_stepSingleFrame)
	{
		m_isPaused = false;
	}

	if (m_isPaused)
	{
		deltaTimeSeconds = 0.0f;
	}
	
	deltaTimeSeconds *= m_timeScale;

	m_totalSeconds += deltaTimeSeconds;

	m_deltaSeconds = deltaTimeSeconds;

	m_frameCount++;

	for (size_t index = 0; index < m_children.size(); index++)
	{
		m_children[index]->Advance(m_deltaSeconds);
	}

	if (m_stepSingleFrame)
	{
		m_isPaused = true;
		m_stepSingleFrame = false;
	}
}

void Clock::AddChild(Clock* childClock)
{
	m_children.push_back(childClock);
}

void Clock::RemoveChild(Clock* childClock)
{
	std::vector<Clock*>::iterator position = std::find(m_children.begin(), m_children.end(), childClock);
	if (position != m_children.end())
		m_children.erase(position);
}