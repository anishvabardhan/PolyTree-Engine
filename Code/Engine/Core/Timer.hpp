#pragma once

class Clock;

class Timer
{
	Clock const* m_clock = nullptr;

	float m_startTime = 0.0f;
	float m_period = 0.0f;
public:
	explicit Timer(float period, Clock const* clock = nullptr);

	void Start();
	void Stop();

	float GetElapsedTime() const;
	float GetElapsedFraction() const;
	bool IsStopped() const;
	bool HasPeriodElapsed() const;
	bool DecrementPeriodIfElapsed();
};