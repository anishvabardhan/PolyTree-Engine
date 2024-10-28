
#include "Engine/Renderer/DX11Renderer.hpp"
#if DX11_RENDERER

#include "Engine/Window/Window.hpp"
#include "Engine/Core/Image.hpp"
#include "Engine/Core/FileUtils.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Renderer/Shader.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Renderer/BitmapFont.hpp"
#include "Engine/Renderer/IndexBuffer.hpp"
#include "Engine/Renderer/VertexBuffer.hpp"
#include "Engine/Renderer/ConstantBuffer.hpp"

#include "ThirdParty/stb_image/stb_image.h"	

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

#if defined(OPAQUE)
#undef OPAQUE
#endif

#if defined(ENGINE_DEBUG_RENDER)
#include <dxgidebug.h>
#pragma comment(lib, "dxguid.lib")
#endif

#define UNUSED(x) (void)x
#define DELETE_PTR(x) if(x) { delete x; x = nullptr; }

struct ModelConstants
{
	Vec4 ModelColor;
	Mat44 ModelMatrix;
};

struct CameraConstants
{
	Mat44 ViewMatrix;
	Mat44 ProjectionMatrix;
};

struct DirectionalLightConstants
{
	Vec3 SunDirection;
	float SunIntensity;
	float AmbientIntensity;
	Vec3 WorldEyePosition;
	float MinFalloff;
	float MaxFalloff;
	float MinFalloffMultiplier;
	float MaxFalloffMultiplier;
	int renderAmbientFlag;
	int renderDiffuseFlag;
	int renderSpecularFlag;
	int renderEmissiveFlag;
	int useDiffuseMap;
	int useNormalMap;
	int useSpecularMap;
	int useGlossinessMap;
	int useEmissiveMap;
	Vec3 padding;
};

struct PointLightConstants
{
	Vec3 PointPosition;
	float pad0 = 0.0f;
	Vec3 PointColor;
	float pad1 = 0.0f;
};

struct SpotLightConstants
{
	Vec3 SpotPosition;
	float Cutoff;
	Vec3 SpotDirection;
	float pad0 = 0.0f;
	Vec3 SpotColor;
	float pad1 = 0.0f;
};

struct BlurSample
{
	Vec2		m_offset;
	float		m_weight;
	int			m_padding;
};

static int const k_blurMaxSamples = 64;

struct BlurConstants
{
	Vec2 m_texelSize;
	float m_lerpT;
	int m_numSamples;
	BlurSample m_samples[k_blurMaxSamples];
};

static int const k_directionalLightConstantSlot = 1;
static int const k_cameraConstantSlot = 2;
static int const k_modelConstantSlot = 3;
static int const k_pointLightConstantSlot = 4;
static int const k_spotLightConstantSlot = 5;
static int const k_blurConstantSlot = 6;

DX11Renderer::DX11Renderer(RenderConfig const& config)
{
	m_config = config;
}

DX11Renderer::~DX11Renderer()
{
}

void DX11Renderer::StartUp()
{
#if DX11_RENDERER
	unsigned int deviceFlags = 0;
#if defined(ENGINE_DEBUG_RENDER)
	deviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

#if defined(ENGINE_DEBUG_RENDER)
	m_dxgiDebugModule = (void*)::LoadLibraryA("dxgidebug.dll");

	if (m_dxgiDebugModule == nullptr)
	{
		ERROR_AND_DIE("Could not load dxgidebug.dll");
	}

	typedef HRESULT(WINAPI* GetDebugModelCB)(REFIID, void**);
	((GetDebugModelCB)::GetProcAddress((HMODULE)m_dxgiDebugModule, "DXGIGetDebugInterface"))(__uuidof(IDXGIDebug), &m_dxgiDebug);

	if (m_dxgiDebug == nullptr)
	{
		ERROR_AND_DIE("Could not load debug module");
	}
#endif

	DXGI_SWAP_CHAIN_DESC swapChainDesc = { 0 };
	swapChainDesc.BufferDesc.Width = m_config.m_window->GetClientDimensions().x;
	swapChainDesc.BufferDesc.Height = m_config.m_window->GetClientDimensions().y;
	swapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	swapChainDesc.SampleDesc.Count = 1;
	swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT | DXGI_USAGE_UNORDERED_ACCESS | DXGI_USAGE_SHADER_INPUT;
	swapChainDesc.BufferCount = 2;
	swapChainDesc.OutputWindow = static_cast<HWND>(m_config.m_window->GetHwnd());
	swapChainDesc.Windowed = true;
	swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

	HRESULT hr;
	hr = D3D11CreateDeviceAndSwapChain(
		nullptr,
		D3D_DRIVER_TYPE_HARDWARE,
		NULL,
		deviceFlags,
		nullptr,
		0,
		D3D11_SDK_VERSION,
		&swapChainDesc,
		&m_swapChain,
		&m_device,
		nullptr,
		&m_deviceContext
	);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create D3D 11 device and swap chain");
	}

	ID3D11Texture2D* backBuffer;
	hr = m_swapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (void**)&backBuffer);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not get swap chain buffer");
	}

	hr = m_device->CreateRenderTargetView(backBuffer, NULL, &m_renderTargetView);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create render target view for swap chain buffer");
	}

	DX_SAFE_RELEASE(backBuffer);

	// create bloom textures

	m_emissiveRenderTexture = CreateRenderTexture(g_theWindow->GetClientDimensions(), "EmissiveRT");
	m_emissiveBlurredRenderTexture = CreateRenderTexture(g_theWindow->GetClientDimensions(), "EmissiveBlurredRT");

	float minTextureHeight = 32.0f;
	float currentTextureHeight = (float)g_theWindow->GetClientDimensions().y * 0.5f;

	while (currentTextureHeight >= minTextureHeight)
	{
		// BLUR DOWN TEXTURE INFO

		Texture* blurDownRenderTexture = CreateRenderTexture(IntVec2((int)(currentTextureHeight * 2.0f), (int)currentTextureHeight), "BlurDownRT");

		m_blurDownTextures.push_back(blurDownRenderTexture);

		// BLUR UP TEXTURE INFO

		if (currentTextureHeight > 45.0f)
		{
			Texture* blurUpRenderTexture = CreateRenderTexture(IntVec2((int)(currentTextureHeight * 2.0f), (int)currentTextureHeight), "BlurUpRT");

			m_blurUpTextures.push_back(blurUpRenderTexture);
		}
		
		blurTextureViewportSizes.push_back(IntVec2(int(currentTextureHeight * 2.0f), (int)currentTextureHeight));

		currentTextureHeight *= 0.5f;
	}

	std::vector<Vertex_PCU> fullScreenVerts;
	AABB2 fullScreenQuad = AABB2(-1.0f, -1.0f, 1.0f, 1.0f);

	AddVertsForAABB2D(fullScreenVerts, fullScreenQuad, Rgba8::WHITE, Vec2(0.0f, 1.0f), Vec2(1.0f, 0.0f));

	m_fullScreenQuadVBO_PCU = CreateVertexBuffer((int)fullScreenVerts.size() * sizeof(Vertex_PCU));
	CopyCPUToGPU(fullScreenVerts.data(), (int)fullScreenVerts.size() * sizeof(Vertex_PCU), m_fullScreenQuadVBO_PCU);

	m_immediateVBO = CreateVertexBuffer(sizeof(Vertex_PCU));
	BindVertexBuffer(m_immediateVBO, sizeof(Vertex_PCU), PrimitiveType::TRIANGLE_LIST);

	m_immediateIBO = CreateIndexBuffer(sizeof(unsigned int));
	BindIndexBuffer(m_immediateIBO);

	m_directionalLightCBO = CreateConstantBuffer(sizeof(DirectionalLightConstants));
	BindConstantBuffer(k_directionalLightConstantSlot, m_directionalLightCBO);

	m_pointLightCBO = CreateConstantBuffer(sizeof(PointLightConstants) * 10);
	BindConstantBuffer(k_pointLightConstantSlot, m_pointLightCBO);

	m_spotLightCBO = CreateConstantBuffer(sizeof(SpotLightConstants) * 2);
	BindConstantBuffer(k_spotLightConstantSlot, m_spotLightCBO);

	m_cameraCBO = CreateConstantBuffer(sizeof(CameraConstants));
	BindConstantBuffer(k_cameraConstantSlot, m_cameraCBO);

	m_modelCBO = CreateConstantBuffer(sizeof(ModelConstants));
	BindConstantBuffer(k_modelConstantSlot, m_modelCBO);

	m_blurCBO = CreateConstantBuffer(sizeof(BlurConstants));
	BindConstantBuffer(k_blurConstantSlot, m_blurCBO);

	D3D11_RASTERIZER_DESC solidBackRasterizerDesc = { 0 };
	solidBackRasterizerDesc.FillMode = D3D11_FILL_SOLID;
	solidBackRasterizerDesc.CullMode = D3D11_CULL_BACK;
	solidBackRasterizerDesc.FrontCounterClockwise = true;
	solidBackRasterizerDesc.DepthBias = 0;
	solidBackRasterizerDesc.DepthBiasClamp = 0.0f;
	solidBackRasterizerDesc.SlopeScaledDepthBias = 0.0f;
	solidBackRasterizerDesc.DepthClipEnable = true;
	solidBackRasterizerDesc.ScissorEnable = false;
	solidBackRasterizerDesc.MultisampleEnable = false;
	solidBackRasterizerDesc.AntialiasedLineEnable = true;

	hr = m_device->CreateRasterizerState(&solidBackRasterizerDesc, &m_rasterizerStates[(int)RasterizerMode::SOLID_CULL_BACK]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create solid back rasterizer state");
	}

	D3D11_RASTERIZER_DESC solidNoneRasterizerDesc = { 0 };
	solidNoneRasterizerDesc.FillMode = D3D11_FILL_SOLID;
	solidNoneRasterizerDesc.CullMode = D3D11_CULL_NONE;
	solidNoneRasterizerDesc.FrontCounterClockwise = true;
	solidNoneRasterizerDesc.DepthBias = 0;
	solidNoneRasterizerDesc.DepthBiasClamp = 0.0f;
	solidNoneRasterizerDesc.SlopeScaledDepthBias = 0.0f;
	solidNoneRasterizerDesc.DepthClipEnable = true;
	solidNoneRasterizerDesc.ScissorEnable = false;
	solidNoneRasterizerDesc.MultisampleEnable = false;
	solidNoneRasterizerDesc.AntialiasedLineEnable = true;

	hr = m_device->CreateRasterizerState(&solidNoneRasterizerDesc, &m_rasterizerStates[(int)RasterizerMode::SOLID_CULL_NONE]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create solid none rasterizer state");
	}

	D3D11_RASTERIZER_DESC wireframeBackRasterizerDesc = { 0 };
	wireframeBackRasterizerDesc.FillMode = D3D11_FILL_WIREFRAME;
	wireframeBackRasterizerDesc.CullMode = D3D11_CULL_BACK;
	wireframeBackRasterizerDesc.FrontCounterClockwise = true;
	wireframeBackRasterizerDesc.DepthBias = 0;
	wireframeBackRasterizerDesc.DepthBiasClamp = 0.0f;
	wireframeBackRasterizerDesc.SlopeScaledDepthBias = 0.0f;
	wireframeBackRasterizerDesc.DepthClipEnable = true;
	wireframeBackRasterizerDesc.ScissorEnable = false;
	wireframeBackRasterizerDesc.MultisampleEnable = false;
	wireframeBackRasterizerDesc.AntialiasedLineEnable = true;

	hr = m_device->CreateRasterizerState(&wireframeBackRasterizerDesc, &m_rasterizerStates[(int)RasterizerMode::WIREFRAME_CULL_BACK]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create wireframe back rasterizer state");
	}

	D3D11_RASTERIZER_DESC wireframeNoneRasterizerDesc = { 0 };
	wireframeNoneRasterizerDesc.FillMode = D3D11_FILL_WIREFRAME;
	wireframeNoneRasterizerDesc.CullMode = D3D11_CULL_NONE;
	wireframeNoneRasterizerDesc.FrontCounterClockwise = true;
	wireframeNoneRasterizerDesc.DepthBias = 0;
	wireframeNoneRasterizerDesc.DepthBiasClamp = 0.0f;
	wireframeNoneRasterizerDesc.SlopeScaledDepthBias = 0.0f;
	wireframeNoneRasterizerDesc.DepthClipEnable = true;
	wireframeNoneRasterizerDesc.ScissorEnable = false;
	wireframeNoneRasterizerDesc.MultisampleEnable = false;
	wireframeNoneRasterizerDesc.AntialiasedLineEnable = true;

	hr = m_device->CreateRasterizerState(&wireframeNoneRasterizerDesc, &m_rasterizerStates[(int)RasterizerMode::WIREFRAME_CULL_NONE]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create wireframe none rasterizer state");
	}

	m_rasterizerState = m_rasterizerStates[(int)m_desiredRasterizerMode];

	m_deviceContext->RSSetState(m_rasterizerState);

	D3D11_BLEND_DESC alphaBlendDesc = {};
	alphaBlendDesc.RenderTarget[0].BlendEnable = TRUE;
	alphaBlendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
	alphaBlendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
	alphaBlendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
	alphaBlendDesc.RenderTarget[0].SrcBlendAlpha = alphaBlendDesc.RenderTarget[0].SrcBlend;
	alphaBlendDesc.RenderTarget[0].DestBlendAlpha = alphaBlendDesc.RenderTarget[0].DestBlend;
	alphaBlendDesc.RenderTarget[0].BlendOpAlpha = alphaBlendDesc.RenderTarget[0].BlendOp;
	alphaBlendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

	hr = m_device->CreateBlendState(&alphaBlendDesc, &m_blendStates[(int)BlendMode::ALPHA]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("CreateBlendState for ALPHA failed");
	}

	D3D11_BLEND_DESC additiveBlendDesc = {};
	additiveBlendDesc.RenderTarget[0].BlendEnable = TRUE;
	additiveBlendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;
	additiveBlendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_ONE;
	additiveBlendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
	additiveBlendDesc.RenderTarget[0].SrcBlendAlpha = additiveBlendDesc.RenderTarget[0].SrcBlend;
	additiveBlendDesc.RenderTarget[0].DestBlendAlpha = additiveBlendDesc.RenderTarget[0].DestBlend;
	additiveBlendDesc.RenderTarget[0].BlendOpAlpha = additiveBlendDesc.RenderTarget[0].BlendOp;
	additiveBlendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

	hr = m_device->CreateBlendState(&additiveBlendDesc, &m_blendStates[(int)BlendMode::ADDITIVE]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("CreateBlendState for ADDITIVE failed");
	}

	D3D11_BLEND_DESC opaqueBlendDesc = {};
	opaqueBlendDesc.RenderTarget[0].BlendEnable = TRUE;
	opaqueBlendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;
	opaqueBlendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_ZERO;
	opaqueBlendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
	opaqueBlendDesc.RenderTarget[0].SrcBlendAlpha = opaqueBlendDesc.RenderTarget[0].SrcBlend;
	opaqueBlendDesc.RenderTarget[0].DestBlendAlpha = opaqueBlendDesc.RenderTarget[0].DestBlend;
	opaqueBlendDesc.RenderTarget[0].BlendOpAlpha = opaqueBlendDesc.RenderTarget[0].BlendOp;
	opaqueBlendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

	hr = m_device->CreateBlendState(&opaqueBlendDesc, &m_blendStates[(int)BlendMode::OPAQUE]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("CreateBlendState for OPAQUE failed");
	}

	D3D11_SAMPLER_DESC pointClampSamplerDesc = {};
	pointClampSamplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
	pointClampSamplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
	pointClampSamplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
	pointClampSamplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
	pointClampSamplerDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
	pointClampSamplerDesc.MaxLOD = D3D11_FLOAT32_MAX;

	hr = m_device->CreateSamplerState(&pointClampSamplerDesc, &m_samplerStates[(int)SamplerMode::POINT_CLAMP]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("CreateSamplerState for POINT_CLAMP failed");
	}

	D3D11_SAMPLER_DESC biLinearWrapSamplerDesc = {};
	biLinearWrapSamplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	biLinearWrapSamplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
	biLinearWrapSamplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
	biLinearWrapSamplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
	biLinearWrapSamplerDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
	biLinearWrapSamplerDesc.MaxLOD = D3D11_FLOAT32_MAX;

	hr = m_device->CreateSamplerState(&biLinearWrapSamplerDesc, &m_samplerStates[(int)SamplerMode::BILINEAR_WRAP]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("CreateSamplerState for BILINEAR_WRAP failed");
	}

	D3D11_SAMPLER_DESC biLinearClampSamplerDesc = {};
	biLinearClampSamplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	biLinearClampSamplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
	biLinearClampSamplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
	biLinearClampSamplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
	biLinearClampSamplerDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
	biLinearClampSamplerDesc.MaxLOD = D3D11_FLOAT32_MAX;

	hr = m_device->CreateSamplerState(&biLinearClampSamplerDesc, &m_samplerStates[(int)SamplerMode::BILINEAR_CLAMP]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("CreateSamplerState for BILINEAR_CLAMP failed");
	}

	m_samplerState = m_samplerStates[(int)SamplerMode::POINT_CLAMP];

	m_deviceContext->PSSetSamplers(0, 1, &m_samplerState);

	D3D11_TEXTURE2D_DESC textureDesc = {};
	textureDesc.Width = m_config.m_window->GetClientDimensions().x;
	textureDesc.Height = m_config.m_window->GetClientDimensions().y;
	textureDesc.MipLevels = 1;
	textureDesc.ArraySize = 1;
	textureDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	textureDesc.SampleDesc.Count = 1;
	textureDesc.Usage = D3D11_USAGE_DEFAULT;
	textureDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;

	hr = m_device->CreateTexture2D(&textureDesc, nullptr, &m_depthStencilTexture);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE(Stringf("Could not create texture for depth stencil"));
	}

	hr = m_device->CreateDepthStencilView(m_depthStencilTexture, nullptr, &m_depthStencilView);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE(Stringf("Could not create depth stencil"));
	}

	D3D11_DEPTH_STENCIL_DESC disabledDepthStencilDesc = {};
	disabledDepthStencilDesc.DepthEnable = TRUE;
	disabledDepthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
	disabledDepthStencilDesc.DepthFunc = D3D11_COMPARISON_ALWAYS;

	hr = m_device->CreateDepthStencilState(&disabledDepthStencilDesc, &m_depthStencilStates[(int)DepthMode::DISABLED]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE(Stringf("Could not create depth stencil disabled state"));
	}

	D3D11_DEPTH_STENCIL_DESC enabledDepthStencilDesc = {};
	enabledDepthStencilDesc.DepthEnable = TRUE;
	enabledDepthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
	enabledDepthStencilDesc.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;

	hr = m_device->CreateDepthStencilState(&enabledDepthStencilDesc, &m_depthStencilStates[(int)DepthMode::ENABLED]);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE(Stringf("Could not create depth stencil enabled state"));
	}

	m_depthStencilState = m_depthStencilStates[(int)DepthMode::ENABLED];

	m_deviceContext->OMSetDepthStencilState(m_depthStencilState, 0);

	Image* image = new Image(IntVec2(2, 2), Rgba8::WHITE);
	m_defaultTexture = CreateTextureFromImage(*image);
	BindTexture();

	m_defaultShader = CreateShader("Default", VertexType::PCU);
	BindShader();

	m_diffuseShader = CreateShader("Diffuse", VertexType::PCUTBN);
	BindShader(m_diffuseShader);

	m_blurDownShader = CreateShader("BlurDown", VertexType::PCU);
	m_blurUpShader = CreateShader("BlurUp", VertexType::PCU);
	m_compositeShader = CreateShader("Composite", VertexType::PCU);
#endif
}

void DX11Renderer::BeginFrame()
{
	ID3D11ShaderResourceView* nullSRVs[] = { nullptr };
	m_deviceContext->PSSetShaderResources(0, 1, nullSRVs);

	ID3D11RenderTargetView* rtvs[] = {
		m_renderTargetView,
		m_emissiveRenderTexture->m_renderTargetView
	};

	m_deviceContext->OMSetRenderTargets(2, rtvs, m_depthStencilView);
}

void DX11Renderer::EndFrame()
{
	m_swapChain->Present(0, 0);
}

void DX11Renderer::ShutDown()
{
	DELETE_PTR(m_immediateVBO);
	DELETE_PTR(m_immediateIBO);
	DELETE_PTR(m_cameraCBO);
	DELETE_PTR(m_modelCBO);
	DELETE_PTR(m_directionalLightCBO);
	DELETE_PTR(m_pointLightCBO);
	DELETE_PTR(m_spotLightCBO);
	DELETE_PTR(m_blurCBO);
	DELETE_PTR(m_diffuseShader);
	DELETE_PTR(m_defaultShader);
	DELETE_PTR(m_blurDownShader);
	DELETE_PTR(m_compositeShader);
	DELETE_PTR(m_blurUpShader);
	DELETE_PTR(m_emissiveRenderTexture);
	DELETE_PTR(m_emissiveBlurredRenderTexture);
	DELETE_PTR(m_fullScreenQuadVBO_PCU);

	for (int i = 0; i < (int)m_blurDownTextures.size(); i++)
	{
		DELETE_PTR(m_blurDownTextures[i]);
	}

	for (int i = 0; i < (int)m_blurUpTextures.size(); i++)
	{
		DELETE_PTR(m_blurUpTextures[i]);
	}

	for (int index = 0; index < (int)BlendMode::COUNT; index++)
	{
		DX_SAFE_RELEASE(m_blendStates[index]);
	}

	for (int index = 0; index < (int)SamplerMode::COUNT; index++)
	{
		DX_SAFE_RELEASE(m_samplerStates[index]);
	}

	for (int index = 0; index < (int)RasterizerMode::COUNT; index++)
	{
		DX_SAFE_RELEASE(m_rasterizerStates[index]);
	}

	for (int index = 0; index < (int)DepthMode::COUNT; index++)
	{
		DX_SAFE_RELEASE(m_depthStencilStates[index]);
	}

	DELETE_PTR(m_defaultTexture);

	for (size_t index = 0; index < m_loadedTextures.size(); index++)
	{
		if (m_loadedTextures[index])
		{
			DELETE_PTR(m_loadedTextures[index]);
		}
	}

	/*for (size_t index = 0; index < m_loadedShaders.size(); index++)
	{
		if (m_loadedShaders[index] && m_loadedShaders[index]->m_config.m_name != "")
		{
			DELETE_PTR(m_loadedShaders[index]);
		}
	}*/

	//for (size_t index = 0; index < m_loadedFonts.size(); index++)
	//{
	//	//if (m_loadedFonts[index] != nullptr)
	//	//{
	//	//	DELETE_PTR(m_loadedFonts[index]);
	//	//}
	//}

	DX_SAFE_RELEASE(m_device);
	DX_SAFE_RELEASE(m_deviceContext);
	DX_SAFE_RELEASE(m_swapChain);
	DX_SAFE_RELEASE(m_renderTargetView);
	DX_SAFE_RELEASE(m_depthStencilView);
	DX_SAFE_RELEASE(m_depthStencilTexture);

#if defined(ENGINE_DEBUG_RENDER)
	((IDXGIDebug*)m_dxgiDebug)->ReportLiveObjects(DXGI_DEBUG_ALL, (DXGI_DEBUG_RLO_FLAGS)(DXGI_DEBUG_RLO_DETAIL | DXGI_DEBUG_RLO_IGNORE_INTERNAL));

	((IDXGIDebug*)m_dxgiDebug)->Release();
	m_dxgiDebug = nullptr;

	::FreeLibrary((HMODULE)m_dxgiDebugModule);
	m_dxgiDebugModule = nullptr;
#endif
}

BitmapFont* DX11Renderer::CreateOrGetBitmapFont(const char* bitmapFontFilePathWithNoExtension)
{
	BitmapFont* font = GetFontForFileName(bitmapFontFilePathWithNoExtension);
	if (font)
	{
		return font;
	}

	BitmapFont* newFont = CreateBitmapFont(bitmapFontFilePathWithNoExtension);
	return newFont;
}

BitmapFont* DX11Renderer::GetFontForFileName(char const* imageFilePath)
{
	for (size_t index = 0; index < m_loadedFonts.size(); index++)
	{
		if (!strcmp(m_loadedFonts[index]->m_fontFilePathNameWithNoExtension.c_str(), imageFilePath))
		{
			return m_loadedFonts[index];
		}
	}

	return nullptr;
}

Texture* DX11Renderer::CreateOrGetTextureFromFile(char const* imageFilePath)
{
	Texture* existingTexture = GetTextureForFileName(imageFilePath);
	if (existingTexture)
	{
		return existingTexture;
	}

	Texture* newTexture = CreateTextureFromFile(imageFilePath);
	return newTexture;
}

Texture* DX11Renderer::CreateTextureFromFile(char const* imageFilePath)
{
	Image* image = new Image(imageFilePath);

	Texture* newTexture = CreateTextureFromImage(*image);

	m_loadedTextures.push_back(newTexture);
	return newTexture;
}

Texture* DX11Renderer::CreateTextureFromImage(Image const& image)
{
	Texture* texture = new Texture();
	texture->m_dimensions = image.GetDimensions();
	texture->m_name = image.GetImageFilePath();

	D3D11_TEXTURE2D_DESC textureDesc = {};
	textureDesc.Width = image.GetDimensions().x;
	textureDesc.Height = image.GetDimensions().y;
	textureDesc.MipLevels = 1;
	textureDesc.ArraySize = 1;
	textureDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	textureDesc.SampleDesc.Count = 1;
	textureDesc.Usage = D3D11_USAGE_IMMUTABLE;
	textureDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

	D3D11_SUBRESOURCE_DATA textureData;
	textureData.pSysMem = image.GetRawData();
	textureData.SysMemPitch = 4 * image.GetDimensions().x;

	HRESULT hr = m_device->CreateTexture2D(&textureDesc, &textureData, &texture->m_texture);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE(Stringf("Failed for image file \"%s\".", image.GetImageFilePath().c_str()));
	}

	hr = m_device->CreateShaderResourceView(texture->m_texture, NULL, &texture->m_shaderResourceView);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE(Stringf("Failed for image file \"%s\".", image.GetImageFilePath().c_str()));
	}

	return texture;
}

Texture* DX11Renderer::CreateModifiableTexture(IntVec2 dimensions)
{
	Texture* texture = new Texture();

	D3D11_TEXTURE2D_DESC textureDesc = {};
	textureDesc.Width = dimensions.x;
	textureDesc.Height = dimensions.y;
	textureDesc.MipLevels = 1;
	textureDesc.ArraySize = 1;
	textureDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	textureDesc.SampleDesc.Count = 1;
	textureDesc.Usage = D3D11_USAGE_DEFAULT;
	textureDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;


	HRESULT hr = m_device->CreateTexture2D(&textureDesc, NULL, &texture->m_texture);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Failed for image file");
	}

	hr = m_device->CreateShaderResourceView(texture->m_texture, NULL, &texture->m_shaderResourceView);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Failed for image file");
	}

	D3D11_UNORDERED_ACCESS_VIEW_DESC rtDescView;
	ZeroMemory(&rtDescView, sizeof(rtDescView));
	rtDescView.ViewDimension = D3D11_UAV_DIMENSION_TEXTURE2D;
	rtDescView.Format = DXGI_FORMAT_R8G8B8A8_UNORM;

	if (SUCCEEDED(hr))
	{
		hr = m_device->CreateUnorderedAccessView(texture->m_texture, &rtDescView, &texture->m_unorderedAccessView);
	}

	return texture;
}

Texture* DX11Renderer::CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, uint8_t* texelData)
{
	// Check if the load was successful
	GUARANTEE_OR_DIE(texelData, Stringf("CreateTextureFromData failed for \"%s\" - texelData was null!", name));
	GUARANTEE_OR_DIE(bytesPerTexel >= 3 && bytesPerTexel <= 4, Stringf("CreateTextureFromData failed for \"%s\" - unsupported BPP=%i (must be 3 or 4)", name, bytesPerTexel));
	GUARANTEE_OR_DIE(dimensions.x > 0 && dimensions.y > 0, Stringf("CreateTextureFromData failed for \"%s\" - illegal texture dimensions (%i x %i)", name, dimensions.x, dimensions.y));

	Texture* newTexture = new Texture();
	newTexture->m_name = name; // NOTE: m_name must be a std::string, otherwise it may point to temporary data!
	newTexture->m_dimensions = dimensions;

	m_loadedTextures.push_back(newTexture);
	return newTexture;
}

Texture* DX11Renderer::GetTextureForFileName(char const* imageFilePath)
{
	for (size_t index = 0; index < m_loadedTextures.size(); index++)
	{
		if (!strcmp(m_loadedTextures[index]->m_name.c_str(), imageFilePath))
		{
			return m_loadedTextures[index];
		}
	}

	return nullptr;
}

Texture* DX11Renderer::CreateRenderTexture(IntVec2 const& dimensions, char const* name)
{
	Texture* renderTexture = new Texture();
	renderTexture->m_dimensions = dimensions;
	renderTexture->m_name = name;

	D3D11_TEXTURE2D_DESC renderTextureDesc = {};
	renderTextureDesc.Width = dimensions.x;
	renderTextureDesc.Height = dimensions.y;
	renderTextureDesc.MipLevels = 1;
	renderTextureDesc.ArraySize = 1;
	renderTextureDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	renderTextureDesc.SampleDesc.Count = 1;
	renderTextureDesc.Usage = D3D11_USAGE_DEFAULT;
	renderTextureDesc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;

	HRESULT hr = m_device->CreateTexture2D(&renderTextureDesc, NULL, &renderTexture->m_texture);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE(Stringf("Failed for image file"));
	}

	hr = m_device->CreateShaderResourceView(renderTexture->m_texture, NULL, &renderTexture->m_shaderResourceView);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE(Stringf("Failed for image file"));
	}

	hr = m_device->CreateRenderTargetView(renderTexture->m_texture, NULL, &renderTexture->m_renderTargetView);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create render target view");
	}

	return renderTexture;
}

BitmapFont* DX11Renderer::CreateBitmapFont(const char* bitmapFontFilePathWithNoExtension)
{
	Texture* fontTexture = CreateOrGetTextureFromFile(bitmapFontFilePathWithNoExtension);

	BitmapFont* font = new BitmapFont(bitmapFontFilePathWithNoExtension, *fontTexture);

	m_loadedFonts.push_back(font);

	return font;
}

void DX11Renderer::BindTexture(int textureSlot, const Texture* texture)
{
	SetSamplerMode(SamplerMode::POINT_CLAMP);

	if (texture == nullptr)
	{
		m_deviceContext->PSSetShaderResources(textureSlot, 1, &m_defaultTexture->m_shaderResourceView);
	}
	else if (texture->m_texture == nullptr)
	{
		m_deviceContext->PSSetShaderResources(textureSlot, 1, &m_defaultTexture->m_shaderResourceView);
	}
	else
	{
		m_deviceContext->PSSetShaderResources(textureSlot, 1, &texture->m_shaderResourceView);
	}
}

void DX11Renderer::UnBindTexture(int textureSlot)
{
	ID3D11ShaderResourceView* nullSRV[1] = { nullptr };
	m_deviceContext->PSSetShaderResources(textureSlot, 1, nullSRV);
}

VertexBuffer* DX11Renderer::CreateVertexBuffer(size_t const size)
{
#if DX11_RENDERER
	VertexBuffer* buffer = new VertexBuffer(size);

	D3D11_BUFFER_DESC bufferDesc = { 0 };
	bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
	bufferDesc.ByteWidth = (UINT)size;
	bufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	HRESULT hr = m_device->CreateBuffer(&bufferDesc, nullptr, &buffer->m_buffer);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create vertex buffer");
	}

	return buffer;
#endif
}

void DX11Renderer::CopyCPUToGPU(void const* data, size_t size, VertexBuffer*& vbo)
{
#if DX11_RENDERER
	if (vbo->m_size < size)
	{
		DELETE_PTR(vbo);
		vbo = CreateVertexBuffer(size);
	}

	D3D11_MAPPED_SUBRESOURCE resource;
	m_deviceContext->Map(vbo->m_buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
	memcpy(resource.pData, data, size);
	m_deviceContext->Unmap(vbo->m_buffer, 0);
#endif
}

void DX11Renderer::BindVertexBuffer(VertexBuffer* buffer, int stride, PrimitiveType type)
{
#if DX11_RENDERER
	UINT vertexStride = (UINT)stride;
	UINT startOffset = 0;

	m_deviceContext->IASetVertexBuffers(0, 1, &buffer->m_buffer, &vertexStride, &startOffset);

	if (type == PrimitiveType::TRIANGLE_LIST)
	{
		m_deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	}
	else if (type == PrimitiveType::LINE_LIST)
	{
		m_deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
	}

#endif
}

IndexBuffer* DX11Renderer::CreateIndexBuffer(size_t const size)
{
	IndexBuffer* buffer = new IndexBuffer(size);

	D3D11_BUFFER_DESC bufferDesc = { 0 };
	bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
	bufferDesc.ByteWidth = (UINT)size;
	bufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
	bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	HRESULT hr = m_device->CreateBuffer(&bufferDesc, nullptr, &buffer->m_buffer);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create index buffer");
	}

	return buffer;
}

void DX11Renderer::CopyCPUToGPU(void const* data, size_t size, IndexBuffer*& ibo)
{
	if (ibo->m_size < size)
	{
		DELETE_PTR(ibo);
		ibo = CreateIndexBuffer(size);
	}

	D3D11_MAPPED_SUBRESOURCE resource;
	m_deviceContext->Map(ibo->m_buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
	memcpy(resource.pData, data, size);
	m_deviceContext->Unmap(ibo->m_buffer, 0);
}

void DX11Renderer::BindIndexBuffer(IndexBuffer* buffer)
{
	m_deviceContext->IASetIndexBuffer(buffer->m_buffer, DXGI_FORMAT_R32_UINT, 0);
}

ConstantBuffer* DX11Renderer::CreateConstantBuffer(size_t const size)
{
	ConstantBuffer* buffer = new ConstantBuffer(size);

	D3D11_BUFFER_DESC bufferDesc = { 0 };
	bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
	bufferDesc.ByteWidth = (UINT)size;
	bufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	HRESULT hr = m_device->CreateBuffer(&bufferDesc, nullptr, &buffer->m_buffer);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create constant buffer");
	}

	return buffer;
}

void DX11Renderer::CopyCPUToGPU(void const* data, size_t size, ConstantBuffer* cbo)
{
	D3D11_MAPPED_SUBRESOURCE resource;
	m_deviceContext->Map(cbo->m_buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
	memcpy(resource.pData, data, size);
	m_deviceContext->Unmap(cbo->m_buffer, 0);
}

void DX11Renderer::BindConstantBuffer(int slot, ConstantBuffer* cbo)
{
	m_deviceContext->VSSetConstantBuffers(slot, 1, &cbo->m_buffer);
	m_deviceContext->PSSetConstantBuffers(slot, 1, &cbo->m_buffer);
}

Shader* DX11Renderer::CreateOrGetShader(char const* shaderName, VertexType type)
{
	for (size_t index = 0; index < m_loadedShaders.size(); index++)
	{
		if (!strcmp(m_loadedShaders[index]->m_config.m_name.c_str(), shaderName))
		{
			return m_loadedShaders[index];
		}
	}

	Shader* newShader = CreateShader(shaderName, type);
	return newShader;
}

Shader* DX11Renderer::CreateShader(char const* shaderName, char const* shaderSource, VertexType type)
{
	UNUSED(shaderName);
	UNUSED(shaderSource);
	UNUSED(type);

#if DX11_RENDERER
	ShaderConfig shaderConfig;
	shaderConfig.m_name = shaderName;

	Shader* shader = new Shader(shaderConfig);

	std::vector<unsigned char> vsByteCode;
	std::vector<unsigned char> psByteCode;

	CompileShaderToByteCode(vsByteCode, shader->GetName().c_str(), shaderSource, shader->m_config.m_vertexEntryPoint.c_str(), "vs_5_0");

	HRESULT hr = m_device->CreateVertexShader(vsByteCode.data(), vsByteCode.size(), NULL, &shader->m_vertexShader);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create vertex shader");
	}

	CompileShaderToByteCode(psByteCode, shader->GetName().c_str(), shaderSource, shader->m_config.m_pixelEntryPoint.c_str(), "ps_5_0");

	hr = m_device->CreatePixelShader(psByteCode.data(), psByteCode.size(), NULL, &shader->m_pixelShader);

	if (!SUCCEEDED(hr))
	{
		ERROR_AND_DIE("Could not create pixel shader");
	}

	if (type == VertexType::PCU)
	{
		D3D11_INPUT_ELEMENT_DESC inputElementDesc[] = {
			{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0}
		};

		UINT numElements = ARRAYSIZE(inputElementDesc);
		hr = m_device->CreateInputLayout(inputElementDesc, numElements, vsByteCode.data(), vsByteCode.size(), &shader->m_inputLayoutForVertex_PCU);

		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create vertex layout");
		}
	}
	else if (type == VertexType::PCUTBN)
	{
		D3D11_INPUT_ELEMENT_DESC inputElementDesc[] = {
			{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"BITANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
			{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0}
		};

		UINT numElements = ARRAYSIZE(inputElementDesc);
		hr = m_device->CreateInputLayout(inputElementDesc, numElements, vsByteCode.data(), vsByteCode.size(), &shader->m_inputLayoutForVertex_PCU);

		if (!SUCCEEDED(hr))
		{
			ERROR_AND_DIE("Could not create vertex layout");
		}
	}

	//m_loadedShaders.push_back(shader);

	return shader;
#endif
	return nullptr;
}

Shader* DX11Renderer::CreateShader(char const* shaderName, VertexType type)
{
	std::string fileName = "Data/Shaders/" + std::string(shaderName);
	std::string shaderSource;

	fileName += ".hlsl";

	FileReadToString(shaderSource, fileName);

	Shader* shader = CreateShader(fileName.c_str(), shaderSource.c_str(), type);

	return shader;
}

bool DX11Renderer::CompileShaderToByteCode(std::vector<unsigned char>& outByteCode, char const* name, char const* source, char const* entryPoint, char const* target)
{
	DWORD shaderFlags = D3DCOMPILE_OPTIMIZATION_LEVEL3;

#if defined(ENGINE_DEBUG_RENDER)
	shaderFlags = D3DCOMPILE_DEBUG;
	shaderFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

	ID3DBlob* shaderBlob = NULL;
	ID3DBlob* errorBlob = NULL;

	HRESULT hr = D3DCompile(
		source,
		strlen(source),
		name,
		nullptr,
		nullptr,
		entryPoint,
		target,
		shaderFlags,
		0,
		&shaderBlob,
		&errorBlob
	);

	if (SUCCEEDED(hr))
	{
		outByteCode.resize(shaderBlob->GetBufferSize());
		memcpy(outByteCode.data(), shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize());
	}
	else
	{
		if (errorBlob != NULL)
		{
			DebuggerPrintf((char*)errorBlob->GetBufferPointer());
		}
		ERROR_AND_DIE("Could not compile vertex shader");
	}

	DX_SAFE_RELEASE(shaderBlob);
	DX_SAFE_RELEASE(errorBlob);

	return true;
}

void DX11Renderer::BindShader(Shader* shader)
{
	UNUSED(shader);
#if DX11_RENDERER
	if (shader == nullptr)
	{
		m_deviceContext->IASetInputLayout(m_defaultShader->m_inputLayoutForVertex_PCU);
		m_deviceContext->VSSetShader(m_defaultShader->m_vertexShader, nullptr, 0);
		m_deviceContext->PSSetShader(m_defaultShader->m_pixelShader, nullptr, 0);

		return;
	}

	m_deviceContext->IASetInputLayout(shader->m_inputLayoutForVertex_PCU);
	m_deviceContext->VSSetShader(shader->m_vertexShader, nullptr, 0);
	m_deviceContext->PSSetShader(shader->m_pixelShader, nullptr, 0);
#endif
}

void DX11Renderer::SetModelConstants(Mat44 const& modelMatrix, Rgba8 const& modelColor)
{
	ModelConstants modelConstants;

	float modelColorInFloats[4];
	modelColor.GetAsFloats(modelColorInFloats);

	modelConstants.ModelColor = Vec4(modelColorInFloats[0], modelColorInFloats[1], modelColorInFloats[2], modelColorInFloats[3]);
	modelConstants.ModelMatrix = modelMatrix;

	CopyCPUToGPU(&modelConstants, m_modelCBO->m_size, m_modelCBO);
	BindConstantBuffer(k_modelConstantSlot, m_modelCBO);
}

void DX11Renderer::SetDirectionalLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity, Vec3 worldEyePosition, LightDebug lightDebugFlags, float minFalloff, float maxFalloff, float minFalloffMultiplier, float maxFalloffMultiplier)
{
	DirectionalLightConstants lightConstants;

	lightConstants.SunDirection			= sunDirection.GetNormalized();
	lightConstants.SunIntensity			= sunIntensity;
	lightConstants.AmbientIntensity		= ambientIntensity;
	lightConstants.WorldEyePosition		= worldEyePosition;
	lightConstants.MinFalloff			= minFalloff;
	lightConstants.MaxFalloff			= maxFalloff;
	lightConstants.MinFalloffMultiplier	= minFalloffMultiplier;
	lightConstants.MaxFalloffMultiplier	= maxFalloffMultiplier;
	lightConstants.renderAmbientFlag	= lightDebugFlags.m_renderAmbientFlag;
	lightConstants.renderDiffuseFlag	= lightDebugFlags.m_renderDiffuseFlag;
	lightConstants.renderSpecularFlag	= lightDebugFlags.m_renderSpecularFlag;
	lightConstants.renderEmissiveFlag	= lightDebugFlags.m_renderEmissiveFlag;
	lightConstants.useDiffuseMap		= lightDebugFlags.m_useDiffuseMap;
	lightConstants.useNormalMap			= lightDebugFlags.m_useNormalMap;
	lightConstants.useSpecularMap		= lightDebugFlags.m_useSpecularMap;
	lightConstants.useGlossinessMap		= lightDebugFlags.m_useGlossinessMap;
	lightConstants.useEmissiveMap		= lightDebugFlags.m_useEmissiveMap;

	CopyCPUToGPU(&lightConstants, m_directionalLightCBO->m_size, m_directionalLightCBO);
	BindConstantBuffer(k_directionalLightConstantSlot, m_directionalLightCBO);
}

void DX11Renderer::SetPointLightConstants(std::vector<Vec3> pointPosition, std::vector<Rgba8> pointColor)
{
	PointLightConstants lightConstants[10];

	for (size_t index = 0; index < 10; index++)
	{
		float color[4];
		pointColor[index].GetAsFloats(color);

		lightConstants[index].PointPosition = pointPosition[index];
		lightConstants[index].PointColor = Vec3(color[0], color[1], color[2]);
	}

	CopyCPUToGPU(&lightConstants[0], sizeof(PointLightConstants) * 10, m_pointLightCBO);
	BindConstantBuffer(k_pointLightConstantSlot, m_pointLightCBO);
}

void DX11Renderer::SetSpotLightConstants(std::vector<Vec3> spotLightPosition, std::vector<float> cutOff, std::vector<Vec3> spotLightDirection, std::vector<Rgba8> spotColor)
{
	SpotLightConstants spotLightConstants[2];

	for (int i = 0; i < 2; i++)
	{
		float color[4];
		spotColor[i].GetAsFloats(color);

		spotLightConstants[i].SpotPosition = spotLightPosition[i];
		spotLightConstants[i].SpotDirection = spotLightDirection[i];
		spotLightConstants[i].SpotColor = Vec3(color[0], color[1], color[2]);
		spotLightConstants[i].Cutoff = cutOff[i];
	}

	CopyCPUToGPU(&spotLightConstants[0], sizeof(SpotLightConstants) * 2, m_spotLightCBO);
	BindConstantBuffer(k_spotLightConstantSlot, m_spotLightCBO);
}

void DX11Renderer::SetBlurConstants(BlurConstants constants)
{
	CopyCPUToGPU(&constants, sizeof(BlurConstants), m_blurCBO);
	BindConstantBuffer(k_blurConstantSlot, m_blurCBO);
}

void DX11Renderer::SetBlendMode(BlendMode blendMode)
{
	m_desiredBlendMode = blendMode;
}

void DX11Renderer::SetBlendStatesIfChanged()
{
	float blendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	UINT sampleMask = 0xffffffff;

	if (m_blendStates[(int)m_desiredBlendMode] != m_blendState)
	{
		m_blendState = m_blendStates[(int)m_desiredBlendMode];
		m_deviceContext->OMSetBlendState(m_blendState, blendFactor, sampleMask);
	}
}

void DX11Renderer::SetSamplerMode(SamplerMode sampleMode)
{
	m_desiredSamplerMode = sampleMode;
}

void DX11Renderer::SetSamplerStatesIfChanged()
{
	if (m_samplerStates[(int)m_desiredSamplerMode] != m_samplerState)
	{
		m_samplerState = m_samplerStates[(int)m_desiredSamplerMode];
		m_deviceContext->PSSetSamplers(0, 1, &m_samplerState);
	}
}

void DX11Renderer::SetRasterizerMode(RasterizerMode rasterizerMode)
{
	m_desiredRasterizerMode = rasterizerMode;
}

void DX11Renderer::SetRasterizerStatesIfChanged()
{
	if (m_rasterizerStates[(int)m_desiredRasterizerMode] != m_rasterizerState)
	{
		m_rasterizerState = m_rasterizerStates[(int)m_desiredRasterizerMode];
		m_deviceContext->RSSetState(m_rasterizerState);
	}
}

void DX11Renderer::SetDepthMode(DepthMode depthMode)
{
	m_desiredDepthStencilMode = depthMode;
}

void DX11Renderer::SetDepthStatesIfChanged()
{
	if (m_depthStencilStates[(int)m_desiredDepthStencilMode] != m_depthStencilState)
	{
		m_depthStencilState = m_depthStencilStates[(int)m_desiredDepthStencilMode];
		m_deviceContext->OMSetDepthStencilState(m_depthStencilState, 0);
	}
}

void DX11Renderer::RenderEmissive()
{
	ID3D11ShaderResourceView* nullSRVs[] = { nullptr };
	m_deviceContext->PSSetShaderResources(0, 1, nullSRVs);

	SetDepthMode(DepthMode::DISABLED);
	SetBlendMode(BlendMode::OPAQUE);
	SetSamplerMode(SamplerMode::BILINEAR_CLAMP);
	SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);
	BindShader(m_blurDownShader);

	BlurConstants blurDownConstants;
	blurDownConstants.m_numSamples = 13;
	blurDownConstants.m_lerpT = 1.0f;

	blurDownConstants.m_samples[0].m_weight = 0.0323f;
	blurDownConstants.m_samples[0].m_offset = Vec2(-2, -2);

	blurDownConstants.m_samples[1].m_weight = 0.0645f;
	blurDownConstants.m_samples[1].m_offset = Vec2(-2, 0);

	blurDownConstants.m_samples[2].m_weight = 0.0323f;
	blurDownConstants.m_samples[2].m_offset = Vec2(-2, 2);

	blurDownConstants.m_samples[3].m_weight = 0.1290f;
	blurDownConstants.m_samples[3].m_offset = Vec2(-1, -1);

	blurDownConstants.m_samples[4].m_weight = 0.1290f;
	blurDownConstants.m_samples[4].m_offset = Vec2(-1, 1);

	blurDownConstants.m_samples[5].m_weight = 0.0645f;
	blurDownConstants.m_samples[5].m_offset = Vec2(0, -2);

	blurDownConstants.m_samples[6].m_weight = 0.0968f;
	blurDownConstants.m_samples[6].m_offset = Vec2(0, 0);

	blurDownConstants.m_samples[7].m_weight = 0.0645f;
	blurDownConstants.m_samples[7].m_offset = Vec2(0, 2);

	blurDownConstants.m_samples[8].m_weight = 0.1290f;
	blurDownConstants.m_samples[8].m_offset = Vec2(1, -1);

	blurDownConstants.m_samples[9].m_weight = 0.1290f;
	blurDownConstants.m_samples[9].m_offset = Vec2(1, 1);

	blurDownConstants.m_samples[10].m_weight = 0.0323f;
	blurDownConstants.m_samples[10].m_offset = Vec2(2, -2);

	blurDownConstants.m_samples[11].m_weight = 0.0645f;
	blurDownConstants.m_samples[11].m_offset = Vec2(2, 0);

	blurDownConstants.m_samples[12].m_weight = 0.0323f;
	blurDownConstants.m_samples[12].m_offset = Vec2(2,  2);

	for (int i = 0; i < (int)m_blurDownTextures.size(); i++)
	{
		m_deviceContext->PSSetShaderResources(0, 1, nullSRVs);

		m_deviceContext->OMSetRenderTargets(1, &m_blurDownTextures[i]->m_renderTargetView, nullptr);

		if (i > 0)
		{
			m_deviceContext->PSSetShaderResources(0, 1, nullSRVs);

			m_deviceContext->PSSetShaderResources(0, 1, &m_blurDownTextures[i - 1]->m_shaderResourceView);

			blurDownConstants.m_texelSize.x = float(1.0f / (float)blurTextureViewportSizes[i - 1].x);
			blurDownConstants.m_texelSize.y = float(1.0f / (float)blurTextureViewportSizes[i - 1].y);
		}
		else
		{
			m_deviceContext->PSSetShaderResources(0, 1, nullSRVs);

			m_deviceContext->PSSetShaderResources(0, 1, &m_emissiveRenderTexture->m_shaderResourceView);

			blurDownConstants.m_texelSize.x = float(1.0f / (float)g_theWindow->GetClientDimensions().x);
			blurDownConstants.m_texelSize.y = float(1.0f / (float)g_theWindow->GetClientDimensions().y);
		}

		SetBlurConstants(blurDownConstants);

		D3D11_VIEWPORT viewport = { 0 };
		viewport.TopLeftX = 0.0f;
		viewport.TopLeftY = 0.0f;
		viewport.Width = (FLOAT)blurTextureViewportSizes[i].x;
		viewport.Height = (FLOAT)blurTextureViewportSizes[i].y;
		viewport.MinDepth = 0.0f;
		viewport.MaxDepth = 1.0f;

		m_deviceContext->RSSetViewports(1, &viewport);

		DrawVertexBuffer(m_fullScreenQuadVBO_PCU, 6, sizeof(Vertex_PCU));
	}

	BlurConstants blurUpConstants;
	blurUpConstants.m_numSamples = 9;
	blurUpConstants.m_lerpT = 0.85f;

	blurUpConstants.m_samples[0].m_weight = 0.0625f;
	blurUpConstants.m_samples[0].m_offset = Vec2(-1, -1);

	blurUpConstants.m_samples[1].m_weight = 0.1250f;
	blurUpConstants.m_samples[1].m_offset = Vec2(-1, 0);

	blurUpConstants.m_samples[2].m_weight = 0.0625f;
	blurUpConstants.m_samples[2].m_offset = Vec2(-1, 1);

	blurUpConstants.m_samples[3].m_weight = 0.1250f;
	blurUpConstants.m_samples[3].m_offset = Vec2(0, -1);

	blurUpConstants.m_samples[4].m_weight = 0.2500f;
	blurUpConstants.m_samples[4].m_offset = Vec2(0, 0);

	blurUpConstants.m_samples[5].m_weight = 0.1250f;
	blurUpConstants.m_samples[5].m_offset = Vec2(0, 1);

	blurUpConstants.m_samples[6].m_weight = 0.0625f;
	blurUpConstants.m_samples[6].m_offset = Vec2(1, -1);

	blurUpConstants.m_samples[7].m_weight = 0.1250f;
	blurUpConstants.m_samples[7].m_offset = Vec2(1, 0);

	blurUpConstants.m_samples[8].m_weight = 0.0625f;
	blurUpConstants.m_samples[8].m_offset = Vec2(1, 1);

	for (int i = (int)m_blurUpTextures.size() - 1; i >= 0; i--)
	{
		m_deviceContext->PSSetShaderResources(0, 1, nullSRVs);
		m_deviceContext->PSSetShaderResources(1, 1, nullSRVs);

		m_deviceContext->OMSetRenderTargets(1, &m_blurUpTextures[i]->m_renderTargetView, nullptr);

		m_deviceContext->PSSetShaderResources(0, 1, &m_blurDownTextures[i]->m_shaderResourceView);

		if (i == (int)m_blurUpTextures.size() - 1)
		{
			m_deviceContext->PSSetShaderResources(1, 1, &m_blurDownTextures[i + 1]->m_shaderResourceView);

			blurUpConstants.m_texelSize.x = float(1.0f / (float)blurTextureViewportSizes[i + 1].x);
			blurUpConstants.m_texelSize.y = float(1.0f / (float)blurTextureViewportSizes[i + 1].y);
		}
		else if (i < (int)m_blurUpTextures.size() - 1)
		{
			m_deviceContext->PSSetShaderResources(1, 1, &m_blurUpTextures[i + 1]->m_shaderResourceView);

			blurUpConstants.m_texelSize.x = float(1.0f / (float)blurTextureViewportSizes[i + 1].x);
			blurUpConstants.m_texelSize.y = float(1.0f / (float)blurTextureViewportSizes[i + 1].y);
		}

		SetBlurConstants(blurUpConstants);

		D3D11_VIEWPORT viewport = { 0 };
		viewport.TopLeftX = 0.0f;
		viewport.TopLeftY = 0.0f;
		viewport.Width = (FLOAT)blurTextureViewportSizes[i].x;
		viewport.Height = (FLOAT)blurTextureViewportSizes[i].y;
		viewport.MinDepth = 0.0f;
		viewport.MaxDepth = 1.0f;

		m_deviceContext->RSSetViewports(1, &viewport);

		BindShader(m_blurUpShader);
		DrawVertexBuffer(m_fullScreenQuadVBO_PCU, 6, sizeof(Vertex_PCU));
	}

	m_deviceContext->PSSetShaderResources(0, 1, nullSRVs);
	m_deviceContext->PSSetShaderResources(1, 1, nullSRVs);

	m_deviceContext->OMSetRenderTargets(1, &m_emissiveBlurredRenderTexture->m_renderTargetView, nullptr);

	m_deviceContext->PSSetShaderResources(0, 1, &m_emissiveRenderTexture->m_shaderResourceView);
	m_deviceContext->PSSetShaderResources(1, 1, &m_blurUpTextures[0]->m_shaderResourceView);

	blurUpConstants.m_texelSize.x = float(1.0f / (float)blurTextureViewportSizes[0].x);
	blurUpConstants.m_texelSize.y = float(1.0f / (float)blurTextureViewportSizes[0].y);

	SetBlurConstants(blurUpConstants);

	D3D11_VIEWPORT viewport = { 0 };
	viewport.TopLeftX = 0.0f;
	viewport.TopLeftY = 0.0f;
	viewport.Width  = (float)g_theWindow->GetClientDimensions().x;
	viewport.Height = (float)g_theWindow->GetClientDimensions().y;
	viewport.MinDepth = 0.0f;
	viewport.MaxDepth = 1.0f;

	m_deviceContext->RSSetViewports(1, &viewport);

	BindShader(m_blurUpShader);
	DrawVertexBuffer(m_fullScreenQuadVBO_PCU, 6, sizeof(Vertex_PCU));

	m_deviceContext->PSSetShaderResources(0, 1, nullSRVs);
	m_deviceContext->PSSetShaderResources(1, 1, nullSRVs);

	ID3D11RenderTargetView* rtvs[] = {
		m_renderTargetView,
		m_emissiveRenderTexture->m_renderTargetView
	};

	m_deviceContext->OMSetRenderTargets(2, rtvs, m_depthStencilView);

	m_deviceContext->PSSetShaderResources(0, 1, &m_emissiveBlurredRenderTexture->m_shaderResourceView);
	SetBlendMode(BlendMode::ADDITIVE);
	BindShader(m_compositeShader);
	DrawVertexBuffer(m_fullScreenQuadVBO_PCU, 6, sizeof(Vertex_PCU));
}

void DX11Renderer::ClearScreen(Rgba8 const& clearColor)
{
	float colorAsFloats[4];
	clearColor.GetAsFloats(colorAsFloats);
	m_deviceContext->ClearRenderTargetView(m_renderTargetView, colorAsFloats);
	
	// EMISSIVE RTVs ----------------------------------------------------------------

	float blackAsFloats[4] = {0.0f, 0.0f, 0.0f, 1.0f};
	m_deviceContext->ClearRenderTargetView(m_emissiveRenderTexture->m_renderTargetView, blackAsFloats);
	m_deviceContext->ClearRenderTargetView(m_emissiveBlurredRenderTexture->m_renderTargetView, blackAsFloats);

	for (int i = 0; i < (int)m_blurDownTextures.size(); i++)
	{
		m_deviceContext->ClearRenderTargetView(m_blurDownTextures[i]->m_renderTargetView, blackAsFloats);
	}

	for (int i = 0; i < (int)m_blurUpTextures.size(); i++)
	{
		m_deviceContext->ClearRenderTargetView(m_blurUpTextures[i]->m_renderTargetView, blackAsFloats);
	}

	// --------------------------------------------------------------------------------

	m_deviceContext->ClearDepthStencilView(m_depthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
}

void DX11Renderer::BeginCamera(Camera& camera)
{
	CameraConstants cameraConstant;

	cameraConstant.ViewMatrix = camera.GetViewMatrix();
	cameraConstant.ProjectionMatrix = camera.GetProjectionMatrix();

	CopyCPUToGPU(&cameraConstant, m_cameraCBO->m_size, m_cameraCBO);
	BindConstantBuffer(k_cameraConstantSlot, m_cameraCBO);

	AABB2 dxViewport = camera.GetDXViewport();

	D3D11_VIEWPORT viewport = { 0 };
	viewport.TopLeftX = dxViewport.m_mins.x;
	viewport.TopLeftY = dxViewport.m_mins.y;
	viewport.Width = dxViewport.GetDimensions().x;
	viewport.Height = dxViewport.GetDimensions().y;
	viewport.MinDepth = 0.0f;
	viewport.MaxDepth = 1.0f;

	m_deviceContext->RSSetViewports(1, &viewport);
}

void DX11Renderer::EndCamera(Camera const& camera)
{
	UNUSED(camera);
}

void DX11Renderer::DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, int vertexStride, int vertexOffset, PrimitiveType type)
{
	BindVertexBuffer(vbo, vertexStride, type);
	SetBlendStatesIfChanged();
	SetDepthStatesIfChanged();
	SetSamplerStatesIfChanged();
	SetRasterizerStatesIfChanged();
	m_deviceContext->Draw(vertexCount, vertexOffset);
}

void DX11Renderer::DrawVertexBufferIndexed(VertexBuffer* vbo, IndexBuffer* ibo, int vertexStride, PrimitiveType type)
{
	unsigned int indexCount = (unsigned int)ibo->m_size / sizeof(unsigned int);

	BindVertexBuffer(vbo, vertexStride, type);
	BindIndexBuffer(ibo);
	SetBlendStatesIfChanged();
	SetDepthStatesIfChanged();
	SetSamplerStatesIfChanged();
	SetRasterizerStatesIfChanged();
	m_deviceContext->DrawIndexed(indexCount, 0, 0);
}

void DX11Renderer::DrawVertexArray(int numVertexes, Vertex_PCU const* vertexes)
{
	CopyCPUToGPU(vertexes, numVertexes * 24, m_immediateVBO);
	DrawVertexBuffer(m_immediateVBO, numVertexes, 24);
}

void DX11Renderer::DrawVertexArrayIndexed(int numVertexes, Vertex_PCU const* vertexes, std::vector<unsigned int> const& indices)
{
	CopyCPUToGPU(vertexes, numVertexes * 24, m_immediateVBO);
	CopyCPUToGPU(indices.data(), indices.size() * sizeof(unsigned int), m_immediateIBO);

	DrawVertexBufferIndexed(m_immediateVBO, m_immediateIBO, 24);
}

void DX11Renderer::DrawVertexArray(int numVertexes, Vertex_PCUTBN const* vertexes)
{
	CopyCPUToGPU(vertexes, numVertexes * 60, m_immediateVBO);
	DrawVertexBuffer(m_immediateVBO, numVertexes, 60);
}

void DX11Renderer::DrawVertexArrayIndexed(int numVertexes, Vertex_PCUTBN const* vertexes, std::vector<unsigned int> const& indices)
{
	CopyCPUToGPU(vertexes, numVertexes * 60, m_immediateVBO);
	CopyCPUToGPU(indices.data(), indices.size() * sizeof(unsigned int), m_immediateIBO);

	DrawVertexBufferIndexed(m_immediateVBO, m_immediateIBO, 60);
}
//
//void DX11Renderer::CreateThresholdCSShader(char const* filename)
//{
//	std::string fileName = std::string(filename);
//	std::string shaderSource;
//	std::vector<unsigned char> outByteCode;
//
//	FileReadToString(shaderSource, fileName);
//
//	DWORD shaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
//
//#if defined(ENGINE_DEBUG_RENDER)
//	shaderFlags |= D3DCOMPILE_DEBUG;
//#endif
//
//	ID3DBlob* shaderBlob = NULL;
//	ID3DBlob* errorBlob = NULL;
//
//	HRESULT hr = D3DCompile(
//		shaderSource.c_str(),
//		strlen(shaderSource.c_str()),
//		filename,
//		nullptr,
//		nullptr,
//		"CSMain",
//		"cs_5_0",
//		shaderFlags,
//		0,
//		&shaderBlob,
//		&errorBlob
//	);
//
//	if (SUCCEEDED(hr))
//	{
//		outByteCode.resize(shaderBlob->GetBufferSize());
//		memcpy(outByteCode.data(), shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize());
//	}
//	else
//	{
//		if (errorBlob != NULL)
//		{
//			DebuggerPrintf((char*)errorBlob->GetBufferPointer());
//		}
//		ERROR_AND_DIE("Could not compile compute shader");
//	}
//
//	DX_SAFE_RELEASE(shaderBlob);
//	DX_SAFE_RELEASE(errorBlob);
//
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_thresholdCSShader);
//}
//
//
//void DX11Renderer::CreateDownsampleCSShader(char const* filename)
//{
//	std::string fileName = std::string(filename);
//	std::string shaderSource;
//	std::vector<unsigned char> outByteCode;
//
//	FileReadToString(shaderSource, fileName);
//
//	DWORD shaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
//
//#if defined(ENGINE_DEBUG_RENDER)
//	shaderFlags |= D3DCOMPILE_DEBUG;
//#endif
//
//	ID3DBlob* shaderBlob = NULL;
//	ID3DBlob* errorBlob = NULL;
//
//	HRESULT hr = D3DCompile(
//		shaderSource.c_str(),
//		strlen(shaderSource.c_str()),
//		filename,
//		nullptr,
//		nullptr,
//		"CSMain",
//		"cs_5_0",
//		shaderFlags,
//		0,
//		&shaderBlob,
//		&errorBlob
//	);
//
//	if (SUCCEEDED(hr))
//	{
//		outByteCode.resize(shaderBlob->GetBufferSize());
//		memcpy(outByteCode.data(), shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize());
//	}
//	else
//	{
//		if (errorBlob != NULL)
//		{
//			DebuggerPrintf((char*)errorBlob->GetBufferPointer());
//		}
//		ERROR_AND_DIE("Could not compile compute shader");
//	}
//
//	DX_SAFE_RELEASE(shaderBlob);
//	DX_SAFE_RELEASE(errorBlob);
//
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_downsampleCSShader);
//}
//
//void DX11Renderer::CreateBlurCSShader(char const* filename)
//{
//
//	std::string fileName = std::string(filename);
//	std::string shaderSource;
//	std::vector<unsigned char> outByteCode;
//
//	FileReadToString(shaderSource, fileName);
//
//	DWORD shaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
//
//#if defined(ENGINE_DEBUG_RENDER)
//	shaderFlags |= D3DCOMPILE_DEBUG;
//#endif
//
//	ID3DBlob* shaderBlob = NULL;
//	ID3DBlob* errorBlob = NULL;
//
//	HRESULT hr = D3DCompile(
//		shaderSource.c_str(),
//		strlen(shaderSource.c_str()),
//		filename,
//		nullptr,
//		nullptr,
//		"CSMain",
//		"cs_5_0",
//		shaderFlags,
//		0,
//		&shaderBlob,
//		&errorBlob
//	);
//
//	if (SUCCEEDED(hr))
//	{
//		outByteCode.resize(shaderBlob->GetBufferSize());
//		memcpy(outByteCode.data(), shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize());
//	}
//	else
//	{
//		if (errorBlob != NULL)
//		{
//			DebuggerPrintf((char*)errorBlob->GetBufferPointer());
//		}
//		ERROR_AND_DIE("Could not compile compute shader");
//	}
//
//	DX_SAFE_RELEASE(shaderBlob);
//	DX_SAFE_RELEASE(errorBlob);
//
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_blurCSShader);
//}
//
//void DX11Renderer::CreateCompositeCSShader(char const* filename)
//{
//
//	std::string fileName = std::string(filename);
//	std::string shaderSource;
//	std::vector<unsigned char> outByteCode;
//
//	FileReadToString(shaderSource, fileName);
//
//	DWORD shaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
//
//#if defined(ENGINE_DEBUG_RENDER)
//	shaderFlags |= D3DCOMPILE_DEBUG;
//#endif
//
//	ID3DBlob* shaderBlob = NULL;
//	ID3DBlob* errorBlob = NULL;
//
//	HRESULT hr = D3DCompile(
//		shaderSource.c_str(),
//		strlen(shaderSource.c_str()),
//		filename,
//		nullptr,
//		nullptr,
//		"CSMain",
//		"cs_5_0",
//		shaderFlags,
//		0,
//		&shaderBlob,
//		&errorBlob
//	);
//
//	if (SUCCEEDED(hr))
//	{
//		outByteCode.resize(shaderBlob->GetBufferSize());
//		memcpy(outByteCode.data(), shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize());
//	}
//	else
//	{
//		if (errorBlob != NULL)
//		{
//			DebuggerPrintf((char*)errorBlob->GetBufferPointer());
//		}
//		ERROR_AND_DIE("Could not compile compute shader");
//	}
//
//	DX_SAFE_RELEASE(shaderBlob);
//	DX_SAFE_RELEASE(errorBlob);
//
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_compositeCSShader);
//}
//
//void DX11Renderer::RunThresholdCSShader(Texture& inTexture, Texture& outTexture)
//{
//	ID3D11UnorderedAccessView* ppUAViewNULL[1] = { NULL };
//	ID3D11ShaderResourceView* ppSRVNULL[2] = { NULL, NULL };
//
//	m_deviceContext->CSSetShader(m_thresholdCSShader, NULL, 0);
//	m_deviceContext->CSSetShaderResources(0, 1, &inTexture.m_shaderResourceView);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, &outTexture.m_unorderedAccessView, NULL);
//
//	m_deviceContext->Dispatch(8, 8, 1);
//
//	m_deviceContext->CSSetShader(NULL, NULL, 0);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, ppUAViewNULL, NULL);
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);
//}
//
//void DX11Renderer::RunDownsampleCSShader(Texture& inTexture, Texture& outTexture)
//{
//	ID3D11UnorderedAccessView* ppUAViewNULL[1] = { NULL };
//	ID3D11ShaderResourceView* ppSRVNULL[2] = { NULL, NULL };
//
//	m_deviceContext->CSSetShader(m_downsampleCSShader, NULL, 0);
//	m_deviceContext->CSSetShaderResources(0, 1, &inTexture.m_shaderResourceView);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, &outTexture.m_unorderedAccessView, NULL);
//
//	m_deviceContext->Dispatch(8, 8, 1);
//
//	m_deviceContext->CSSetShader(NULL, NULL, 0);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, ppUAViewNULL, NULL);
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);
//}
//
//void DX11Renderer::CreateUpsampleCSShader(char const* filename)
//{
//	std::string fileName = std::string(filename);
//	std::string shaderSource;
//	std::vector<unsigned char> outByteCode;
//
//	FileReadToString(shaderSource, fileName);
//
//	DWORD shaderFlags = D3DCOMPILE_ENABLE_STRICTNESS;
//
//#if defined(ENGINE_DEBUG_RENDER)
//	shaderFlags |= D3DCOMPILE_DEBUG;
//#endif
//
//	ID3DBlob* shaderBlob = NULL;
//	ID3DBlob* errorBlob = NULL;
//
//	HRESULT hr = D3DCompile(
//		shaderSource.c_str(),
//		strlen(shaderSource.c_str()),
//		filename,
//		nullptr,
//		nullptr,
//		"CSMain",
//		"cs_5_0",
//		shaderFlags,
//		0,
//		&shaderBlob,
//		&errorBlob
//	);
//
//	if (SUCCEEDED(hr))
//	{
//		outByteCode.resize(shaderBlob->GetBufferSize());
//		memcpy(outByteCode.data(), shaderBlob->GetBufferPointer(), shaderBlob->GetBufferSize());
//	}
//	else
//	{
//		if (errorBlob != NULL)
//		{
//			DebuggerPrintf((char*)errorBlob->GetBufferPointer());
//		}
//		ERROR_AND_DIE("Could not compile compute shader");
//	}
//
//	DX_SAFE_RELEASE(shaderBlob);
//	DX_SAFE_RELEASE(errorBlob);
//
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_upsampleCSShader);
//}
//
//void DX11Renderer::RunUpsampleCSShader(Texture& inTexture, Texture& outTexture)
//{
//	ID3D11UnorderedAccessView* ppUAViewNULL[1] = { NULL };
//	ID3D11ShaderResourceView* ppSRVNULL[2] = { NULL, NULL };
//
//	m_deviceContext->CSSetShader(m_upsampleCSShader, NULL, 0);
//	m_deviceContext->CSSetShaderResources(0, 1, &inTexture.m_shaderResourceView);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, &outTexture.m_unorderedAccessView, NULL);
//
//	m_deviceContext->Dispatch(8, 8, 1);
//
//	m_deviceContext->CSSetShader(NULL, NULL, 0);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, ppUAViewNULL, NULL);
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);
//}
//
//void DX11Renderer::RunBlurCSShader(Texture& inTexture, Texture& outTexture)
//{
//	ID3D11UnorderedAccessView* ppUAViewNULL[1] = { NULL };
//	ID3D11ShaderResourceView* ppSRVNULL[2] = { NULL, NULL };
//
//	m_deviceContext->CSSetShader(m_blurCSShader, NULL, 0);
//	m_deviceContext->CSSetShaderResources(0, 1, &inTexture.m_shaderResourceView);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, &outTexture.m_unorderedAccessView, NULL);
//
//	m_deviceContext->Dispatch(8, 8, 1);
//
//	m_deviceContext->CSSetShader(NULL, NULL, 0);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, ppUAViewNULL, NULL);
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);
//}
//
//void DX11Renderer::RunCompositeCSShader(Texture& inThresholdTexture, Texture& inBlurredTexture, Texture& outTexture)
//{
//	ID3D11UnorderedAccessView* ppUAViewNULL[1] = { NULL };
//	ID3D11ShaderResourceView* ppSRVNULL[2] = { NULL, NULL };
//
//	m_deviceContext->CSSetShader(m_compositeCSShader, NULL, 0);
//	m_deviceContext->CSSetShaderResources(0, 1, &inThresholdTexture.m_shaderResourceView);
//	m_deviceContext->CSSetShaderResources(1, 1, &inBlurredTexture.m_shaderResourceView);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, &outTexture.m_unorderedAccessView, NULL);
//
//	m_deviceContext->Dispatch(8, 8, 1);
//
//	m_deviceContext->CSSetShader(NULL, NULL, 0);
//	m_deviceContext->CSSetUnorderedAccessViews(0, 1, ppUAViewNULL, NULL);
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);
//	m_deviceContext->CSSetShaderResources(1, 2, ppSRVNULL);
//}

#endif