#pragma once

#include "Engine/Core/EngineCommon.hpp"

#include "Game/EngineBuildPreferences.hpp"

#if DX11_RENDERER
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Renderer/Camera.hpp"
#include "Engine/Renderer/Renderer.hpp"

#include <vector>
#include <string>

#if defined(_DEBUG)
#define ENGINE_DEBUG_RENDER
#endif

#define DX_SAFE_RELEASE(dxObject)	\
{									\
	if (dxObject)					\
	{								\
		(dxObject)->Release();		\
		(dxObject) = nullptr;		\
	}								\
}									

struct ID3D11Device;
struct ID3D11DeviceContext;
struct IDXGISwapChain;
struct ID3D11RenderTargetView;
struct ID3D11RasterizerState;
struct ID3D11BlendState;
struct ID3D11SamplerState;
struct ID3D11DepthStencilState;
struct ID3D11DepthStencilView;
struct ID3D11Texture2D;
struct ID3D11ComputeShader;
struct ID3D11Buffer;
struct ID3D11ShaderResourceView;
struct ID3D11UnorderedAccessView;

struct BlurConstants;

class Window;
class Image;
class Texture;
class Image;
class Shader;
class VertexBuffer;
class IndexBuffer;
class ConstantBuffer;
class BitmapFont;

class DX11Renderer
{
public:
	ID3D11Device*				m_device				= nullptr;
	ID3D11DeviceContext*		m_deviceContext			= nullptr;
	IDXGISwapChain*				m_swapChain				= nullptr;
	ID3D11RenderTargetView*		m_renderTargetView		= nullptr;

	// BLOOM / EMISSIVE CODE ----------------------------------------------------------------

	Texture const*								m_emissiveRenderTexture				= nullptr;
	Texture const*								m_emissiveBlurredRenderTexture		= nullptr;

	std::vector<Texture*>						m_blurDownTextures;
	std::vector<Texture*>						m_blurUpTextures;
	std::vector<IntVec2>						blurTextureViewportSizes;

	VertexBuffer*								m_fullScreenQuadVBO_PCU				= nullptr;

	Shader*										m_blurDownShader					= nullptr;
	Shader*										m_blurUpShader						= nullptr;
	Shader*										m_compositeShader					= nullptr;

	//---------------------------------------------------------------------------------------

	ID3D11BlendState*			m_blendState			= nullptr;
	BlendMode					m_desiredBlendMode		= BlendMode::ALPHA;
	ID3D11BlendState*			m_blendStates[(int)BlendMode::COUNT] = {};
	ID3D11RasterizerState*		m_rasterizerState		= nullptr;
	RasterizerMode				m_desiredRasterizerMode = RasterizerMode::SOLID_CULL_BACK;
	ID3D11RasterizerState*		m_rasterizerStates[(int)RasterizerMode::COUNT] = {};
	ID3D11SamplerState*			m_samplerState			= nullptr;
	SamplerMode					m_desiredSamplerMode	= SamplerMode::POINT_CLAMP;
	ID3D11SamplerState*			m_samplerStates[(int)SamplerMode::COUNT] = {};
	ID3D11DepthStencilState*	m_depthStencilState			= nullptr;
	DepthMode					m_desiredDepthStencilMode	= DepthMode::DISABLED;
	ID3D11DepthStencilState*	m_depthStencilStates[(int)DepthMode::COUNT] = {};
	ID3D11DepthStencilView*		m_depthStencilView		= nullptr;
	ID3D11Texture2D*			m_depthStencilTexture = nullptr;

	std::vector<Texture*>		m_loadedTextures;
	std::vector<BitmapFont*>	m_loadedFonts;
	std::vector<Shader*>		m_loadedShaders;
	Shader*						m_defaultShader			= nullptr;
	Shader*						m_diffuseShader			= nullptr;
	Texture const*				m_defaultTexture		= nullptr;
	Texture const*				m_currentTexture		= nullptr;
	VertexBuffer*				m_immediateVBO			= nullptr;
	IndexBuffer*				m_immediateIBO			= nullptr;
	ConstantBuffer*				m_cameraCBO				= nullptr;
	ConstantBuffer*				m_modelCBO				= nullptr;
	ConstantBuffer*				m_directionalLightCBO	= nullptr;
	ConstantBuffer*				m_pointLightCBO			= nullptr;
	ConstantBuffer*				m_spotLightCBO			= nullptr;
	ConstantBuffer*				m_blurCBO				= nullptr;
	RenderConfig				m_config;
	void*						m_rc					= nullptr;
	void*						m_dxgiDebugModule		= nullptr;
	void*						m_dxgiDebug				= nullptr;
public:
	DX11Renderer(RenderConfig const& config);
	~DX11Renderer();

	void			StartUp();
	void			BeginFrame();
	void			EndFrame();
	void			ShutDown();

	ID3D11DeviceContext* GetContext() const { return m_deviceContext;}

	BitmapFont*		CreateOrGetBitmapFont( const char* bitmapFontFilePathWithNoExtension );
	BitmapFont*		CreateBitmapFont(const char* bitmapFontFilePathWithNoExtension);
	BitmapFont*		GetFontForFileName(char const* imageFilePath);

	Texture*		CreateOrGetTextureFromFile(char const* imageFilePath);
	Texture*		CreateTextureFromFile(char const* imageFilePath);
	Texture*		CreateTextureFromImage(Image const& image);
	Texture*		CreateModifiableTexture(IntVec2 dimensions);
	Texture*		CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, uint8_t* texelData);
	Texture*		GetTextureForFileName(char const* imageFilePath);
	Texture*		CreateRenderTexture(IntVec2 const& dimensions, char const* name);
	void			BindTexture(int textureSlot = 0, const Texture* texture = nullptr);
	void			UnBindTexture(int textureSlot);

	VertexBuffer*	CreateVertexBuffer(size_t const size);
	void			CopyCPUToGPU(void const* data, size_t size, VertexBuffer*& vbo);
	void			BindVertexBuffer(VertexBuffer* buffer, int stride, PrimitiveType type);

	IndexBuffer*	CreateIndexBuffer(size_t const size);
	void			CopyCPUToGPU(void const* data, size_t size, IndexBuffer*& ibo);
	void			BindIndexBuffer(IndexBuffer* buffer);

	ConstantBuffer* CreateConstantBuffer(size_t const size);
	void			CopyCPUToGPU(void const* data, size_t size, ConstantBuffer* cbo);
	void			BindConstantBuffer(int slot, ConstantBuffer* cbo);

	Shader*			CreateOrGetShader(char const* shaderName, VertexType type);
	Shader*			CreateShader(char const* shaderName, char const* shaderSource, VertexType type);
	Shader*			CreateShader(char const* shaderName, VertexType type);
	bool			CompileShaderToByteCode(std::vector<unsigned char>& outByteCode, char const* name, char const* source, char const* entryPoint, char const* target);
	void			BindShader(Shader* shader = nullptr);

	void			SetModelConstants(Mat44 const& modelMatrix = Mat44(), Rgba8 const& modelColor = Rgba8::WHITE);
	void			SetDirectionalLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity, Vec3 worldEyePosition, LightDebug lightDebugFlags, float minFalloff, float maxFalloff, float minFalloffMultiplier, float maxFalloffMultiplier);
	void			SetPointLightConstants(std::vector<Vec3> pointPosition, std::vector<Rgba8> pointColor);
	void			SetSpotLightConstants(std::vector<Vec3> spotLightPosition, std::vector<float> cutOff, std::vector<Vec3> spotLightDirection, std::vector<Rgba8> spotColor);
	void			SetBlurConstants(BlurConstants constants);

	void			SetBlendMode( BlendMode blendMode );
	void			SetBlendStatesIfChanged();

	void			SetSamplerMode(SamplerMode sampleMode);
	void			SetSamplerStatesIfChanged();

	void			SetRasterizerMode(RasterizerMode rasterizerMode);
	void			SetRasterizerStatesIfChanged();

	void			SetDepthMode(DepthMode depthMode);
	void			SetDepthStatesIfChanged();

	void			RenderEmissive();

	void			ClearScreen(Rgba8 const& clearColor);
	void			BeginCamera(Camera& camera);
	void			EndCamera(Camera const& camera);
	void			DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, int vertexStride = 0, int vertexOffset = 0, PrimitiveType type = PrimitiveType::TRIANGLE_LIST);
	void			DrawVertexBufferIndexed(VertexBuffer* vbo, IndexBuffer* ibo, int vertexStride = 0, PrimitiveType type = PrimitiveType::TRIANGLE_LIST);
	void			DrawVertexArray(int numVertexes, Vertex_PCU const* vertexes);
	void			DrawVertexArrayIndexed(int numVertexes, Vertex_PCU const* vertexes, std::vector<unsigned int> const& indices);
	void			DrawVertexArray(int numVertexes, Vertex_PCUTBN const* vertexes);
	void			DrawVertexArrayIndexed(int numVertexes, Vertex_PCUTBN const* vertexes, std::vector<unsigned int> const& indices);
public:
	ID3D11ComputeShader*			m_thresholdCSShader						= nullptr;
	ID3D11ComputeShader*			m_downsampleCSShader					= nullptr;
	ID3D11ComputeShader*			m_upsampleCSShader						= nullptr;
	ID3D11ComputeShader*			m_blurCSShader							= nullptr;
	ID3D11ComputeShader*			m_compositeCSShader						= nullptr;

	//void CreateThresholdCSShader(char const* filename);
	//void RunThresholdCSShader(Texture& inTexture, Texture& outTexture);
	//void CreateDownsampleCSShader(char const* filename);
	//void RunDownsampleCSShader(Texture& inTexture, Texture& outTexture);
	//void CreateUpsampleCSShader(char const* filename);
	//void RunUpsampleCSShader(Texture& inTexture, Texture& outTexture);
	//void CreateBlurCSShader(char const* filename);
	//void RunBlurCSShader(Texture& inTexture, Texture& outTexture);
	//void CreateCompositeCSShader(char const* filename);
	//void RunCompositeCSShader(Texture& inThresholdTexture, Texture& inBlurredTexture, Texture& outTexture);
};
#endif