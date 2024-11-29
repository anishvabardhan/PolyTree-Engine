#include "Engine/Core/Vertex_PCU.hpp"

Vertex_PCU::Vertex_PCU()
	: m_position(0.0f, 0.0f, 0.0f), m_color(0, 0, 0, 0), m_uvTexCoords(0.0f, 0.0f)
{

}

Vertex_PCU::Vertex_PCU(float x, float y, unsigned char r, unsigned char g, unsigned char b, unsigned char a)
	: m_position(x, y, 0.0f), m_color(r, g, b, a), m_uvTexCoords(0.0f, 0.0f)
{
}

Vertex_PCU::Vertex_PCU(Vec3 const& position, Rgba8 const& color, Vec2 const& uvTexCoords)
	: m_position(position), m_color(color), m_uvTexCoords(uvTexCoords)
{
}

Vertex_PCU::~Vertex_PCU()
{
}
