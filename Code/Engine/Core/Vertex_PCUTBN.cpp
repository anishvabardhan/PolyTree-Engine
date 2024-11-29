#include "Engine/Core/Vertex_PCUTBN.hpp"

Vertex_PCUTBN::Vertex_PCUTBN()
	: m_position(Vec3::ZERO), m_color(0, 0, 0, 0), m_uvTexCoords(Vec2::ZERO), m_tangent(Vec3::ZERO), m_biTangent(Vec3::ZERO), m_normal(Vec3::ZERO)
{

}

Vertex_PCUTBN::Vertex_PCUTBN(float x, float y, unsigned char r, unsigned char g, unsigned char b, unsigned char a)
	: m_position(x, y, 0.0f), m_color(r, g, b, a), m_uvTexCoords(Vec2::ZERO), m_tangent(Vec3::ZERO), m_biTangent(Vec3::ZERO), m_normal(Vec3::ZERO)
{
}

Vertex_PCUTBN::Vertex_PCUTBN(Vec3 const& position, Rgba8 const& color, Vec2 const& uvTexCoords, Vec3 const& tangent, Vec3 const& biTangent, Vec3 normal)
	: m_position(position), m_color(color), m_uvTexCoords(uvTexCoords), m_tangent(tangent), m_biTangent(biTangent), m_normal(normal)
{
}

Vertex_PCUTBN::~Vertex_PCUTBN()
{
}
