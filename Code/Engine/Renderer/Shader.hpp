#pragma once

#include "Engine/Core/EngineCommon.hpp"

#include "Game/EngineBuildPreferences.hpp"


#include <d3dcommon.h>
#include <d3d12.h>

#include "ThirdParty/DXC/dxcapi.h"

#include <string>

struct ID3D11VertexShader;
struct ID3D11PixelShader;
struct ID3D11InputLayout;

struct ShaderConfig
{
	std::string m_name;
	std::string  m_vertexEntryPoint = "VertexMain";
	std::string  m_pixelEntryPoint = "PixelMain";
};

class Shader
{
public:
	ShaderConfig m_config;
#if DX11_RENDERER
	ID3D11VertexShader* m_vertexShader = nullptr;
	ID3D11PixelShader* m_pixelShader = nullptr;
	ID3D11InputLayout* m_inputLayoutForVertex_PCU = nullptr;
#elif DX12_RENDERER
	ID3DBlob* m_vertexShader_5 = nullptr;
	ID3DBlob* m_pixelShader_5 = nullptr;

	IDxcBlob* m_computeShader_6 = nullptr;

	IDxcBlob* m_meshShader_6 = nullptr;
	IDxcBlob* m_ampShader_6 = nullptr;
	IDxcBlob* m_pixelShader_6 = nullptr;
#endif
public:
	Shader(ShaderConfig const& config);
	Shader(Shader const& copy) = delete;
	~Shader();
	
	std::string const& GetName() const;
private:
	friend class Renderer;
	friend class DX11Renderer;
	friend class DX12Renderer;
};