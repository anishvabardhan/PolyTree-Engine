#pragma once

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/AABB2.hpp"
#include "Engine/Math/Mat44.hpp"
#include "Engine/Math/EulerAngles.hpp"

class ConstantBuffer;

class Camera
{
public:
	enum Mode
	{
		ORTHOGRAPHIC,
		PERSPECTIVE,
		COUNT
	};

	Vec3			m_position;
	EulerAngles		m_orientation;

	AABB2			m_normalizedViewport		= AABB2::ZERO_TO_ONE;
	ConstantBuffer* m_cameraCBO					= nullptr;
private:

	Mode			m_mode						= ORTHOGRAPHIC;

	AABB2			m_Bounds;

	Vec2			m_orthoBottomLeft;
	Vec2			m_orthoTopRight;
	float			m_orthoNear;
	float			m_orthoFar;

	float			m_perspectiveAspect;
	float			m_perspectiveFOV;
	float			m_perspectiveNear;
	float			m_perspectiveFar;

	Vec3			m_renderIBasis				= Vec3(1.0f, 0.0f, 0.0f);
	Vec3			m_renderJBasis				= Vec3(0.0f, 1.0f, 0.0f);
	Vec3			m_renderKBasis				= Vec3(0.0f, 0.0f, 1.0f);
public:
												Camera();
												~Camera();

	void										SetOrthoView(AABB2 const& bounds);
	void										SetOrthoView(Vec2 const& bottomLeft, Vec2 const& topRight, float near = 0.0f, float far = 1.0f);
	void										SetPerspectiveView(float aspect, float fov, float near, float far);

	void										SetRenderBasis(Vec3 const& iBasis, Vec3 const& jBasis, Vec3 const& kBasis);

	void										SetCenter(Vec2 const& center);
	void										SetTransform(Vec3 const& position, EulerAngles const& orientation);
	void										Translate2D(Vec2 const& translation);
	Vec2										GetOrthoBottomLeft() const;
	Vec2										GetOrthoTopRight() const;

	AABB2										GetDXViewport() const;

	Mat44										GetOrthoMatrix() const;
	Mat44										GetPerspectiveMatrix() const;
	Mat44										GetProjectionMatrix() const;
	Mat44										GetViewMatrix() const;
	Mat44										GetModelMatrix() const;
	Mat44										GetRenderMatrix() const;
};