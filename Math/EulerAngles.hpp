#pragma once

struct Vec3;
struct Mat44;

class EulerAngles
{
public:
	float m_yawDegrees = 0.0f;
	float m_pitchDegrees = 0.0f;
	float m_rollDegrees = 0.0f;
public:
	EulerAngles() = default;
	EulerAngles(float yaw, float pitch, float roll);
	~EulerAngles();

	float GetYaw() const;
	float GetPitch() const;
	float GetRoll() const;

	EulerAngles SetFromText(char const* text);

	void GetAsVectors_XFwd_YLeft_ZUp(Vec3& iBasisForward, Vec3& jBasisLeft, Vec3& kBasisUp);
	Mat44 GetAsMatrix_XFwd_YLeft_ZUp() const;
};