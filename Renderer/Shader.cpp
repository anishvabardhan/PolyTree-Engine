#include "Engine/Renderer/Shader.hpp"

#include "Engine/Renderer/Renderer.hpp"

#if DX11_RENDERER
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>
#elif DX12_RENDERER
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#endif

#define DX_SAFE_RELEASE(dxObject)	\
{									\
	if (dxObject)					\
	{								\
		(dxObject)->Release();		\
		(dxObject) = nullptr;		\
	}								\
}									

Shader::Shader(ShaderConfig const& config)
	: m_config(config)
{
}

Shader::~Shader()
{
#if DX11_RENDERER
	DX_SAFE_RELEASE(m_vertexShader);
	DX_SAFE_RELEASE(m_pixelShader);
	DX_SAFE_RELEASE(m_inputLayoutForVertex_PCU);
#elif DX12_RENDERER
	DX_SAFE_RELEASE(m_vertexShader_5);
	DX_SAFE_RELEASE(m_pixelShader_5);

	DX_SAFE_RELEASE(m_meshShader_6);
	DX_SAFE_RELEASE(m_ampShader_6);
	DX_SAFE_RELEASE(m_pixelShader_6);
#endif
}

std::string const& Shader::GetName() const
{
	return m_config.m_name;
}
