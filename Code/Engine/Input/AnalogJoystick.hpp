#pragma once

#include "Engine/Math/Vec2.hpp"

class AnalogJoystick
{
protected:
	Vec2		m_rawPosition;
	Vec2		m_correctedPosition;
	float		m_innerDeadZoneFraction = 0.0f;
	float		m_outerDeadZoneFraction = 1.0f;
public:
				AnalogJoystick();
				~AnalogJoystick();

	void		Reset();
	void		SetDeadZoneThresholds(float normalizedInnerDeadzoneThreshold, float normalizedOuterDeadzoneThreshold);
	void		UpdatePosition(float rawNormalizedX, float rawNormalizedY);

	Vec2		GetPosition() const;
	float		GetMagnitude() const;
	float		GetOrientationDegrees() const;

	Vec2		GetRawUncorrectedPosition() const;
	float		GetInnerDeadZoneFraction() const;
	float		GetOuterDeadZoneFraction() const;
};