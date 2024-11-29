#pragma once

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Core/Rgba8.hpp"

struct Vertex_PCUTBN
{
public:
	Vec3		m_position;
	Rgba8		m_color;
	Vec2		m_uvTexCoords;
	Vec3		m_tangent;
	Vec3		m_biTangent;
	Vec3		m_normal;
public:
				Vertex_PCUTBN();
				Vertex_PCUTBN(float x, float y, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
				Vertex_PCUTBN(Vec3 const& position, Rgba8 const& color, Vec2 const& uvTexCoords, Vec3 const& tangent, Vec3 const& biTangent, Vec3 normal);
				~Vertex_PCUTBN();
};