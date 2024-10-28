#pragma once

#include "Engine/Core/Rgba8.hpp"
#include "Engine/Renderer/Renderer.hpp"

#include <string>

class Shader;
class Texture;

class Material
{
public:
	Rgba8			m_color;
	Shader*			m_shader				= nullptr;
	VertexType		m_vertexType;
	Texture*		m_diffuseTexture		= nullptr;
	Texture*		m_normalTexture			= nullptr;
	Texture*		m_specGlosEmitTexture	= nullptr;
public:
					Material()				= default;
					~Material();

	bool			Load(std::string const& xmlFilename);
};