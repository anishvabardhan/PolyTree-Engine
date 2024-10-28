#pragma once

#include <vector>

class Clock
{
	Clock* m_parent = nullptr;

	std::vector<Clock*> m_children;

	float m_lastUpdateTimeInSeconds = 0.0f;
	float m_totalSeconds = 0.0f;
	float m_deltaSeconds = 0.0f;
	size_t m_frameCount = 0;

	float m_timeScale = 1.0f;

	bool m_isPaused = false;
	bool m_stepSingleFrame = false;

	float m_maxDeltaSeconds = 0.1f;
public:
	Clock();
	explicit Clock(Clock& parent);
	Clock(Clock const& copy) = delete;
	~Clock();

	void Reset();
	bool IsPaused() const;
	void Pause();
	void Unpause();
	void TogglePause();

	void StepSingleFrame();

	void SetTimeScale(float timeScale);
	float GetTimeScale() const;

	float GetDeltaSeconds() const;
	float GetTotalSeconds() const;
	size_t GetFrameCount() const;
public:
	static Clock& GetSystemClock();
	static void TickSystemClock();
protected:
	void Tick();
	void Advance(float deltaTimeSeconds);
	void AddChild(Clock* childClock);
	void RemoveChild(Clock* childClock);
};