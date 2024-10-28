#include "Engine/Renderer/Camera.hpp"

#include "Engine/Window/Window.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Renderer/ConstantBuffer.hpp"

extern Renderer* g_theRenderer;

struct CameraConstants
{
	Mat44 ViewMatrix;
	Mat44 ProjectionMatrix;
};

Camera::Camera()
{
	m_cameraCBO = g_theRenderer->CreateConstantBuffer(sizeof(CameraConstants), std::wstring(L"Camera"));
}

Camera::~Camera()
{
	DELETE_PTR(m_cameraCBO);
}

void Camera::SetOrthoView(AABB2 const& bounds)
{
	m_Bounds = bounds;
}

void Camera::SetOrthoView(Vec2 const& bottomLeft, Vec2 const& topRight, float near, float far)
{
	m_mode = Mode::ORTHOGRAPHIC;

	m_orthoBottomLeft = bottomLeft;
	m_orthoTopRight = topRight;
	m_orthoNear = near;
	m_orthoFar = far;
}

void Camera::SetPerspectiveView(float aspect, float fov, float near, float far)
{
	m_mode = Mode::PERSPECTIVE;

	m_perspectiveAspect = aspect;
	m_perspectiveFOV = fov;
	m_perspectiveNear = near;
	m_perspectiveFar = far;
}

void Camera::SetRenderBasis(Vec3 const& iBasis, Vec3 const& jBasis, Vec3 const& kBasis)
{
	m_renderIBasis = iBasis;
	m_renderJBasis = jBasis;
	m_renderKBasis = kBasis;
}

void Camera::SetCenter(Vec2 const& center)
{
	m_Bounds.SetCenter(center);
}

void Camera::SetTransform(Vec3 const& position, EulerAngles const& orientation)
{
	m_position = position;
	m_orientation = orientation;
}

void Camera::Translate2D(Vec2 const& translation)
{
	m_orthoBottomLeft += translation;
	m_orthoTopRight += translation;
}

Vec2 Camera::GetOrthoBottomLeft() const
{
	return m_orthoBottomLeft;
}

Vec2 Camera::GetOrthoTopRight() const
{
	return m_orthoTopRight;
}

Mat44 Camera::GetOrthoMatrix() const
{
	return Mat44::CreateOrthoProjection(m_orthoBottomLeft.x, m_orthoTopRight.x, m_orthoBottomLeft.y, m_orthoTopRight.y, m_orthoNear, m_orthoFar);
}

Mat44 Camera::GetPerspectiveMatrix() const
{
	return Mat44::CreatePerspectiveProjection(m_perspectiveFOV, m_perspectiveAspect, m_perspectiveNear, m_perspectiveFar);
}

Mat44 Camera::GetProjectionMatrix() const
{
	if (m_mode == ORTHOGRAPHIC)
	{
		Mat44 projectionMatrix = GetOrthoMatrix();

		projectionMatrix.Append(GetRenderMatrix());

		return projectionMatrix;
	}
	else if(m_mode == PERSPECTIVE)
	{
		Mat44 projectionMatrix = GetPerspectiveMatrix();

		projectionMatrix.Append(GetRenderMatrix());

		return projectionMatrix;
	}

	return Mat44();
}

Mat44 Camera::GetViewMatrix() const
{
	Mat44 viewMatrix;

	viewMatrix.SetTranslation3D(m_position);
	viewMatrix.Append(m_orientation.GetAsMatrix_XFwd_YLeft_ZUp());

	return viewMatrix.GetOrthonormalInverse();
}

Mat44 Camera::GetModelMatrix() const
{
	Mat44 matrix;
	matrix.SetTranslation3D(m_position);
	matrix.Append(m_orientation.GetAsMatrix_XFwd_YLeft_ZUp());

	return matrix;
}

Mat44 Camera::GetRenderMatrix() const
{
	Mat44 renderMatrix;

	renderMatrix.SetIJK3D(m_renderIBasis, m_renderJBasis, m_renderKBasis);

	return renderMatrix;
}

AABB2 Camera::GetDXViewport() const
{
	AABB2 normalizedDXViewport;

	normalizedDXViewport.m_mins.x = m_normalizedViewport.m_mins.x;
	normalizedDXViewport.m_mins.y = 1.0f - m_normalizedViewport.m_maxs.y;
	normalizedDXViewport.m_maxs.x = m_normalizedViewport.m_maxs.x;
	normalizedDXViewport.m_maxs.y = 1.0f - m_normalizedViewport.m_mins.y;

	normalizedDXViewport.m_mins.x *= Window::GetWindowContext()->GetClientDimensions().x;
	normalizedDXViewport.m_mins.y *= Window::GetWindowContext()->GetClientDimensions().y;
	normalizedDXViewport.m_maxs.x *= Window::GetWindowContext()->GetClientDimensions().x;
	normalizedDXViewport.m_maxs.y *= Window::GetWindowContext()->GetClientDimensions().y;

	return normalizedDXViewport;
}