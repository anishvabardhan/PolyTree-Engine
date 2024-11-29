#include "Timer.hpp"

#include "Engine/Core/Clock.hpp"

Timer::Timer(float period, Clock const* clock)
	: m_clock(clock), m_period(period)
{
}

void Timer::Start()
{
	m_startTime = m_clock->GetTotalSeconds();
}

void Timer::Stop()
{
	m_startTime = 0.0f;
}

float Timer::GetElapsedTime() const
{
	return m_clock->GetTotalSeconds() - m_startTime;
}

float Timer::GetElapsedFraction() const
{
	return float(GetElapsedTime() / m_period);
}

bool Timer::IsStopped() const
{
	if(m_startTime == 0.0f)
		return true;

	return false;
}

bool Timer::HasPeriodElapsed() const
{
	if(GetElapsedTime() > m_period/* && !IsStopped()*/)
		return true;

	return false;
}

bool Timer::DecrementPeriodIfElapsed()
{
	if (HasPeriodElapsed() && !IsStopped())
	{
		m_startTime += m_period;
		return true;
	}

	return false;
}
