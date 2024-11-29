#pragma once

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Core/Rgba8.hpp"

struct MeshVertex_PCU
{
	Vec3 m_position;
	Vec4 m_color;
	Vec2 m_uv;

	MeshVertex_PCU() = default;
	MeshVertex_PCU(Vec3 pos, Vec4 color, Vec2 uv) : m_position(pos), m_color(color), m_uv(uv) {}
	~MeshVertex_PCU() {}
};

struct MeshVertex_PCUTBN
{
	Vec3 m_position;
	Vec4 m_color;
	Vec2 m_uv;
	Vec3 m_tangent;
	Vec3 m_biTangent;
	Vec3 m_normal;

	MeshVertex_PCUTBN() = default;
	MeshVertex_PCUTBN(Vec3 pos, Vec4 color, Vec2 uv, Vec3 normal) : m_position(pos), m_color(color), m_uv(uv), m_normal(normal) {}
	~MeshVertex_PCUTBN() {}
};