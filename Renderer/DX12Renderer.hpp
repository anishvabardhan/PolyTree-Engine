#pragma once

#include "Engine/Core/EngineCommon.hpp"

#include "Game/EngineBuildPreferences.hpp"

#if DX12_RENDERER
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Core/MeshVertex_PCU.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Renderer/Camera.hpp"
#include "Engine/Renderer/Renderer.hpp"

#include <vector>
#include <string>
	
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#include <dxgidebug.h>
#include <d3dcommon.h>

#pragma comment(lib, "dxguid.lib")
#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

#if defined(OPAQUE)
#undef OPAQUE
#endif

#if defined(ENGINE_DEBUG_RENDER)
#include <dxgidebug.h>
#pragma comment(lib, "dxguid.lib")
#endif

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

struct IDXGIFactory2;
struct IDXGIAdapter;
struct ID3D12Debug;
struct IDXGIDebug;
struct IDXGISwapChain1;
struct IDxcCompiler3;
struct IDxcUtils;
struct IDxcIncludeHandler;
struct IDxcBlob;
struct ID3D12Device;
struct ID3D12CommandQueue;
struct ID3D12GraphicsCommandList6;
struct ID3D12CommandAllocator;
struct ID3D12Fence;
struct ID3D12Resource;
struct ID3D12DescriptorHeap;
struct ID3D12PipelineState;
struct ID3D12RootSignature;
struct D3D12_GRAPHICS_PIPELINE_STATE_DESC;
struct D3DX12_MESH_SHADER_PIPELINE_STATE_DESC;

struct MeshletDebugConstants;
struct aiScene;
struct aiNode;
struct aiMesh;

class Model;
class Image;
class Window;
class Shader;
class Texture;
class BitmapFont;
class MeshBuffer;
class IndexBuffer;
class VertexBuffer;
class ConstantBuffer;

enum PipelineState
{
	BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE,
	BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE,
	BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE,
	BLEND_AL_SAMPLE_BILINEAR_RAST_SCB_DEPTH_ENABLE,
	BLEND_AD_SAMPLE_BILINEAR_RAST_SCB_DEPTH_ENABLE,
	BLEND_OP_SAMPLE_BILINEAR_RAST_SCB_DEPTH_ENABLE,

	BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE,
	BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE,
	BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE,
	BLEND_AL_SAMPLE_BILINEAR_RAST_WCB_DEPTH_ENABLE,
	BLEND_AD_SAMPLE_BILINEAR_RAST_WCB_DEPTH_ENABLE,
	BLEND_OP_SAMPLE_BILINEAR_RAST_WCB_DEPTH_ENABLE,

	BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE,
	BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE,
	BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE,
	BLEND_AL_SAMPLE_BILINEAR_RAST_SCN_DEPTH_ENABLE,
	BLEND_AD_SAMPLE_BILINEAR_RAST_SCN_DEPTH_ENABLE,
	BLEND_OP_SAMPLE_BILINEAR_RAST_SCN_DEPTH_ENABLE,

	BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE,
	BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE,
	BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE,
	BLEND_AL_SAMPLE_BILINEAR_RAST_WCN_DEPTH_ENABLE,
	BLEND_AD_SAMPLE_BILINEAR_RAST_WCN_DEPTH_ENABLE,
	BLEND_OP_SAMPLE_BILINEAR_RAST_WCN_DEPTH_ENABLE,

	BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE,
	BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE,
	BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE,
	BLEND_AL_SAMPLE_BILINEAR_RAST_SCB_DEPTH_DISABLE,
	BLEND_AD_SAMPLE_BILINEAR_RAST_SCB_DEPTH_DISABLE,
	BLEND_OP_SAMPLE_BILINEAR_RAST_SCB_DEPTH_DISABLE,

	BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE,
	BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE,
	BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE,
	BLEND_AL_SAMPLE_BILINEAR_RAST_WCB_DEPTH_DISABLE,
	BLEND_AD_SAMPLE_BILINEAR_RAST_WCB_DEPTH_DISABLE,
	BLEND_OP_SAMPLE_BILINEAR_RAST_WCB_DEPTH_DISABLE,

	BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE,
	BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE,
	BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE,
	BLEND_AL_SAMPLE_BILINEAR_RAST_SCN_DEPTH_DISABLE,
	BLEND_AD_SAMPLE_BILINEAR_RAST_SCN_DEPTH_DISABLE,
	BLEND_OP_SAMPLE_BILINEAR_RAST_SCN_DEPTH_DISABLE,

	BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE,
	BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE,
	BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE,
	BLEND_AL_SAMPLE_BILINEAR_RAST_WCN_DEPTH_DISABLE,
	BLEND_AD_SAMPLE_BILINEAR_RAST_WCN_DEPTH_DISABLE,
	BLEND_OP_SAMPLE_BILINEAR_RAST_WCN_DEPTH_DISABLE,

	TOTAL_PIPELINE_STATES
};

const int NUM_FRAME_BUFFERS = 2;

class DX12Renderer
{
public:
	ID3D12Device2*							m_device						= nullptr;
	IDXGIFactory2*							m_dxgiFactory					= nullptr;
	IDXGIAdapter*							m_dxgiAdapter					= nullptr;
	ID3D12Debug*							m_d3dDebugger					= nullptr;
	IDXGIDebug*								m_dxgiDebugger					= nullptr;
	ID3D12CommandQueue*						m_commandQueue					= nullptr;
	ID3D12GraphicsCommandList6*				m_commandList					= nullptr;
	ID3D12CommandAllocator*					m_commandAllocator				= nullptr;
	ID3D12Fence*							m_fence							= nullptr;
	uint64_t								m_fenceValue					= 0;

	IDXGISwapChain1*						m_swapChain						= nullptr;
	ID3D12Resource*							m_renderTargets[NUM_FRAME_BUFFERS];
	ID3D12DescriptorHeap*					m_rtvDescHeap					= nullptr;
	int										m_heapIncrement					= 0;

	// DXC-------------
	IDxcCompiler3*							m_dxcCompiler					= nullptr;
	IDxcUtils*								m_dxcUtils						= nullptr;
	IDxcIncludeHandler*						m_dxcIncludeHandler				= nullptr;
	//-----------------

	// MESH SHADER-----
	D3DX12_MESH_SHADER_PIPELINE_STATE_DESC*  m_meshPipelineStateDescriptor  = nullptr;
	ID3D12PipelineState*					m_meshPSO						= nullptr;
	// ----------------

	D3D12_GRAPHICS_PIPELINE_STATE_DESC*		m_pipelineStateDescriptor		= nullptr;
	ID3D12PipelineState*					m_PSO[TOTAL_PIPELINE_STATES];
	ID3D12RootSignature*					m_rootSig[TOTAL_ROOT_SIGS];
	int										m_frameIndex					= 0;

	BlendMode								m_currentBlendMode				= BlendMode::ALPHA;
	BlendMode								m_desiredBlendMode				= BlendMode::ALPHA;
	RasterizerMode							m_currentRasterizerMode			= RasterizerMode::SOLID_CULL_BACK;
	RasterizerMode							m_desiredRasterizerMode			= RasterizerMode::SOLID_CULL_BACK;
	SamplerMode								m_currentSamplerMode			= SamplerMode::POINT_CLAMP;
	SamplerMode								m_desiredSamplerMode			= SamplerMode::POINT_CLAMP;
	DepthMode								m_currentDepthStencilMode		= DepthMode::DISABLED;
	DepthMode								m_desiredDepthStencilMode		= DepthMode::DISABLED;

	ID3D12DescriptorHeap*					m_dsvDescHeap					= nullptr;
	ID3D12Resource*							m_depthStencilTexture			= nullptr;

	ID3D12DescriptorHeap*					m_imGuiSRVDescHeap				= nullptr;

	std::vector<Texture*>					m_loadedTextures;
	std::vector<BitmapFont*>				m_loadedFonts;
	std::vector<Shader*>					m_loadedShaders;
	Shader*									m_defaultShader					= nullptr;

	// MESH SHADER--------------------------------------------------------------------
	Shader*									m_meshShader					= nullptr;
	//--------------------------------------------------------------------------------

	// AMPLIFICATION SHADER--------------------------------------------------------------------
	Shader*									m_amplificationShader			= nullptr;
	//--------------------------------------------------------------------------------

	Shader*									m_diffuseShader					= nullptr;
	Texture const*							m_defaultTexture				= nullptr;
	Texture const*							m_currentTexture				= nullptr;
	VertexBuffer*							m_immediateVBO					= nullptr;
	IndexBuffer*							m_immediateIBO					= nullptr;
	ConstantBuffer*							m_cameraCBO						= nullptr;
	ConstantBuffer*							m_modelCBO						= nullptr;
	ConstantBuffer*							m_meshletDebugCBO				= nullptr;
	ConstantBuffer*							m_directionalLightCBO			= nullptr;
	ConstantBuffer*							m_pointLightCBO					= nullptr;
	ConstantBuffer*							m_spotLightCBO					= nullptr;
	RenderConfig							m_config;
public:
											DX12Renderer(RenderConfig const& config);
											~DX12Renderer();

	void									StartUp();
	void									BeginFrame();
	void									EndFrame();
	void									ShutDown();

	void									ImGuiStartUp();
	void									ImGuiBeginFrame();
	void									ImGuiEndFrame();
	void									ImGuiShutDown();

	void									ResetCommandList();
	void									ExecuteCommandList();
	void									SignalAndWait();

	Model*									LoadModel(char const* filePath, RootSig pipelineMode);
	void									ProcessNode(aiNode *node, const aiScene *scene, std::vector<Vertex_PCU>& verts, std::vector<unsigned int>& indices);
	void									ProcessNode(aiNode *node, const aiScene *scene, std::vector<MeshVertex_PCU>& verts, std::vector<unsigned int>& indices);
	void									ProcessMesh(aiMesh *mesh, std::vector<Vertex_PCU>& verts, std::vector<unsigned int>& indices);
	void									ProcessMesh(aiMesh *mesh, std::vector<MeshVertex_PCU>& verts, std::vector<unsigned int>& indices);

	BitmapFont*								CreateOrGetBitmapFont( const char* bitmapFontFilePathWithNoExtension );
	BitmapFont*								CreateBitmapFont(const char* bitmapFontFilePathWithNoExtension);
	BitmapFont*								GetFontForFileName(char const* imageFilePath);

	Texture*								CreateOrGetTextureFromFile(char const* imageFilePath);
	Texture*								CreateTextureFromFile(char const* imageFilePath);
	Texture*								CreateTextureFromImage(Image const& image);
	Texture*								CreateModifiableTexture(IntVec2 dimensions);
	Texture*								CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, uint8_t* texelData);
	Texture*								GetTextureForFileName(char const* imageFilePath);
	void									BindTexture(const Texture* texture = nullptr, RootSig pipelineMode = RootSig::DEFAULT_PIPELINE);

	void									CreateShaderResourceView(ID3D12Resource* defaultBuffer, ID3D12DescriptorHeap*& srv, DXGI_FORMAT srvFormat, D3D12_SRV_DIMENSION srvDims, UINT bufferSize, UINT numElements);

	void									CreateDepthStencilBuffer();

	VertexBuffer*							CreateVertexBuffer(size_t const size, std::wstring bufferDebugName = L"Vertex");
	void									CopyCPUToGPU(void const* data, size_t size, VertexBuffer*& vbo);
	void									BindVertexBuffer(VertexBuffer* buffer, int stride);

	IndexBuffer*							CreateIndexBuffer(size_t const size, std::wstring bufferDebugName = L"Index");
	void									CopyCPUToGPU(void const* data, size_t size, IndexBuffer*& ibo);
	void									BindIndexBuffer(IndexBuffer* buffer);

	MeshBuffer*								CreateMeshBuffer(size_t const size, std::wstring bufferDebugName = L"Mesh");
	void									CopyCPUToGPU(void const* data, size_t size, MeshBuffer*& mbo);
	void									BindMeshBuffer(Model* model, Texture* texture = nullptr);

	ConstantBuffer*							CreateConstantBuffer(size_t const size, std::wstring bufferDebugName = L"Constant");
	void									CopyCPUToGPU(void const* data, size_t size, ConstantBuffer*& cbo);
	void									BindConstantBuffer(int slot, ConstantBuffer* cbo, RootSig pipelineMode);
	
	Shader*									CreateShader(char const* shaderName, char const* shaderSource, VertexType type);
	Shader*									CreateShader(char const* shaderName, VertexType type);
	
											// MESH SHADER----------
	Shader*									CreateMeshShader(std::wstring shaderName, VertexType type);
	Shader*									CreateAmplificationShader(std::wstring shaderName, VertexType type);
	void									CreateMeshShadingPipelineState();
											//----------------------

	bool									CompileShader(ID3DBlob** shaderBlob, char const* name, char const* source, char const* entryPoint, char const* target);
	bool									DXCCompileShader(IDxcBlob** shaderBlob, std::wstring name, std::wstring source, std::wstring entryPoint, std::wstring target);
	void									WriteIDXCBlobToFile(IDxcBlob* pBlob, const wchar_t* fileName);

	void									BindShader(Shader* shader = nullptr);

	void									SetModelConstants(RootSig pipelineMode, Mat44 const& modelMatrix = Mat44(), Rgba8 const& modelColor = Rgba8::WHITE, ConstantBuffer* modelCBO = nullptr);
	void									SetModelConstants(std::vector<Mat44> const& modelMatrix, std::vector<Rgba8> const& modelColor); // TODO: FLesh out multi model constants data system
	void									SetDirectionalLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity);
	void									SetPointLightConstants(std::vector<Vec3> pointPosition, std::vector<Rgba8> pointColor);
	void									SetSpotLightConstants(std::vector<Vec3> spotLightPosition, std::vector<float> cutOff, std::vector<Vec3> spotLightDirection, std::vector<Rgba8> spotColor);
	void									SetMeshletDebugConstants(MeshletDebugConstants debugConstants);

	void									SetBlendMode( BlendMode blendMode );
	void									SetBlendStatesIfChanged();

	void									SetSamplerMode(SamplerMode sampleMode);
	void									SetSamplerStatesIfChanged();

	void									SetRasterizerMode(RasterizerMode rasterizerMode);
	void									SetRasterizerStatesIfChanged();

	void									SetDepthMode(DepthMode depthMode);
	void									SetDepthStatesIfChanged();

	void									CreateRootSignature(RootSig rootSig);
	void									CreateAllRootSignatures();

	void									CreateAllPipelineStates();
	void									CreatePipelineState(PipelineState pipelineState);
	void									SetPipelineStateDesc(D3D12_BLEND srcBlend, D3D12_BLEND destBlend, D3D12_FILL_MODE filleMode, D3D12_CULL_MODE cullMode, D3D12_COMPARISON_FUNC depthCompFunc, D3D12_DEPTH_WRITE_MASK depthWriteMask);
	void									SetPipelineState();

	void									ClearScreen(Rgba8 const& clearColor);
	void									BeginCamera(Camera& camera, RootSig pipelineMode);
	void									EndCamera(Camera const& camera);
	void									DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, int vertexStride = 0, int vertexOffset = 0, PrimitiveType type = PrimitiveType::TRIANGLE_LIST);
	void									DrawVertexBufferIndexed(VertexBuffer* vbo, IndexBuffer* ibo, int vertexStride = 0);
	void									DrawVertexArray(int numVertexes, Vertex_PCU const* vertexes);
	void									DrawVertexArray(int numVertexes, Vertex_PCU const* vertexes, VertexBuffer* vbo);
	void									DrawVertexArrayIndexed(int numVertexes, Vertex_PCU const* vertexes, std::vector<unsigned int> const& indices);
	void									DrawVertexArray(int numVertexes, Vertex_PCUTBN const* vertexes);
	void									DrawVertexArrayIndexed(int numVertexes, Vertex_PCUTBN const* vertexes, std::vector<unsigned int> const& indices);
	void									DrawMesh(int numOfMeshlets);
};
#endif