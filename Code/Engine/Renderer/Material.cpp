#include "Engine/Renderer/Material.hpp"

#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Renderer/Shader.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Renderer/DX11Renderer.hpp"

Material::~Material()
{
	DELETE_PTR(m_shader);
}

bool Material::Load(std::string const& xmlFilename)
{
	tinyxml2::XMLDocument document;
	
	if (document.LoadFile(xmlFilename.c_str()) == tinyxml2::XML_SUCCESS)
	{
		// Get the root element 
		tinyxml2::XMLElement* rootElement = document.RootElement();
		GUARANTEE_OR_DIE(rootElement, "XML couldn't be loaded");
		
		std::string shaderName = ParseXmlAttribute(*rootElement, "shader", std::string());
		std::string vertexTypeName = ParseXmlAttribute(*rootElement, "vertexType", std::string());
		std::string diffuseTextureName = ParseXmlAttribute(*rootElement, "diffuseTexture", std::string());
		std::string normalTextureName = ParseXmlAttribute(*rootElement, "normalTexture", std::string());
		std::string specGlossEmitTextureName = ParseXmlAttribute(*rootElement, "specGlossEmitTexture", std::string());
		m_color = ParseXmlAttribute(*rootElement, "color", Rgba8(255, 255, 255, 255));

		if (vertexTypeName == "Vertex_PCUTBN")
		{
			m_vertexType = VertexType::PCUTBN;
		}
		else if (vertexTypeName == "Vertex_PCU")
		{
			m_vertexType = VertexType::PCU;
		}

		m_shader = g_theRenderer->CreateShader(shaderName.c_str(), m_vertexType);
		m_diffuseTexture = g_theRenderer->CreateOrGetTextureFromFile(diffuseTextureName.c_str());
		m_normalTexture = g_theRenderer->CreateOrGetTextureFromFile(normalTextureName.c_str());
		m_specGlosEmitTexture = g_theRenderer->CreateOrGetTextureFromFile(specGlossEmitTextureName.c_str());

		return true;
	}
	return false;
}
