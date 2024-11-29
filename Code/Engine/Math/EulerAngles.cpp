#include "Engine/Math/EulerAngles.hpp"

#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Math/MathUtils.hpp"

#include <math.h>

EulerAngles::EulerAngles(float yaw, float pitch, float roll)
	: m_yawDegrees(yaw), m_pitchDegrees(pitch), m_rollDegrees(roll)
{
}

EulerAngles::~EulerAngles()
{
}

float EulerAngles::GetYaw() const
{
	return m_yawDegrees;
}

float EulerAngles::GetPitch() const
{
	return m_pitchDegrees;
}

float EulerAngles::GetRoll() const
{
	return m_rollDegrees;
}

EulerAngles EulerAngles::SetFromText(char const* text)
{
	Strings splitValues = SplitStringOnDelimiter(text, ',');

	float yawValue		= static_cast<float>(std::atof(splitValues[0].c_str()));
	float pitchValue	= static_cast<float>(std::atof(splitValues[1].c_str()));
	float rollValue		= static_cast<float>(std::atof(splitValues[2].c_str()));

	m_yawDegrees = yawValue;
	m_pitchDegrees = pitchValue;
	m_rollDegrees = rollValue;

	return EulerAngles(m_yawDegrees, m_pitchDegrees, m_rollDegrees);
}

void EulerAngles::GetAsVectors_XFwd_YLeft_ZUp(Vec3& iBasisForward, Vec3& jBasisLeft, Vec3& kBasisUp)
{
	float cy = CosDegrees(m_yawDegrees);
	float sy = SinDegrees(m_yawDegrees);

	float cp = CosDegrees(m_pitchDegrees);
	float sp = SinDegrees(m_pitchDegrees);

	float cr = CosDegrees(m_rollDegrees);
	float sr = SinDegrees(m_rollDegrees);

	Mat44 result;

	result.m_values[Mat44::Ix] = (cy * cp);
	result.m_values[Mat44::Iy] = (sy * cp);
	result.m_values[Mat44::Iz] = -1.0f * sp;
	result.m_values[Mat44::Iw] = 0;
	result.m_values[Mat44::Jx] = ((sp * sr * cy) + (cr * sy * -1.0f));
	result.m_values[Mat44::Jy] = ((cy * cr) + (sy * sp * sr));
	result.m_values[Mat44::Jz] = (sr * cp);
	result.m_values[Mat44::Jw] = 0;
	result.m_values[Mat44::Kx] = ((sy * sr) + (cy * sp * cr));
	result.m_values[Mat44::Ky] = ((-1.0f * sr * cy) + (sy * sp * cr));
	result.m_values[Mat44::Kz] = (cp * cr);


	iBasisForward = Vec3(result.m_values[Mat44::Ix], result.m_values[Mat44::Iy], result.m_values[Mat44::Iz]);
	jBasisLeft = Vec3(result.m_values[Mat44::Jx], result.m_values[Mat44::Jy], result.m_values[Mat44::Jz]);
	kBasisUp = Vec3(result.m_values[Mat44::Kx], result.m_values[Mat44::Ky], result.m_values[Mat44::Kz]);
}

Mat44 EulerAngles::GetAsMatrix_XFwd_YLeft_ZUp() const
{
	float cy = CosDegrees(m_yawDegrees);
	float sy = SinDegrees(m_yawDegrees);

	float cp = CosDegrees(m_pitchDegrees);
	float sp = SinDegrees(m_pitchDegrees);

	float cr = CosDegrees(m_rollDegrees);
	float sr = SinDegrees(m_rollDegrees);

	Mat44 result;

	result.m_values[Mat44::Ix] = (cy * cp);
	result.m_values[Mat44::Iy] = (sy * cp);
	result.m_values[Mat44::Iz] = -1.0f * sp;


	result.m_values[Mat44::Jx] = ((sp * sr * cy) + (cr * sy * -1.0f));
	result.m_values[Mat44::Jy] = ((cy * cr) + (sy * sp * sr));
	result.m_values[Mat44::Jz] = (sr * cp);

	result.m_values[Mat44::Kx] = ((sy * sr) + (cy * sp * cr));
	result.m_values[Mat44::Ky] = ((-1.0f * sr * cy) + (sy * sp * cr));
	result.m_values[Mat44::Kz] = (cp * cr);

	return result;
}
