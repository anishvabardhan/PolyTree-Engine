#pragma once

#include "Engine/Core/Rgba8.hpp"
#include "Engine/Core/Vertex_PCU.hpp"
#include "Engine/Core/Vertex_PCUTBN.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Renderer/Camera.hpp"

#include <vector>
#include <string>
#include <memory>

#define DX_SAFE_RELEASE(dxObject)	\
{									\
	if (dxObject)					\
	{								\
		(dxObject)->Release();		\
		(dxObject) = nullptr;		\
	}								\
}									

class Model;
class Image;
class Window;
class Shader;
class Texture;
class BitmapFont;
class MeshBuffer;
class IndexBuffer;
class VertexBuffer;
class DX12Renderer;
class DX11Renderer;
class ConstantBuffer;

enum RootSig
{
	DEFAULT_PIPELINE,
	DIFFUSE_PIPELINE,
	COMPUTE_PIPELINE,
	MESH_SHADER_PIPELINE,

	TOTAL_ROOT_SIGS
};

enum class BlendMode
{
	ALPHA,
	ADDITIVE,
	OPAQUE,
	COUNT
};

enum class SamplerMode
{
	POINT_CLAMP,
	BILINEAR_WRAP,
	BILINEAR_CLAMP,
	COUNT
};

enum class RasterizerMode
{
	SOLID_CULL_NONE,
	SOLID_CULL_BACK,
	WIREFRAME_CULL_NONE,
	WIREFRAME_CULL_BACK,
	COUNT
};

enum class DepthMode
{
	DISABLED,
	ENABLED,
	COUNT
};

enum class ShadingMode
{
	DEFAULT,
	DIFFUSE,
	COUNT
};

enum class VertexType
{
	PC,
	PCU,
	PCUTBN,
	COUNT
};

enum class PrimitiveType
{
	TRIANGLE_LIST,
	LINE_LIST,
	COUNT
};

struct LightDebug
{
	int m_renderAmbientFlag = 1;
	int m_renderDiffuseFlag = 1;
	int m_renderSpecularFlag = 1;
	int m_renderEmissiveFlag = 1;
	int m_useDiffuseMap = 1;
	int m_useNormalMap = 1;
	int m_useSpecularMap = 1;
	int m_useGlossinessMap = 1;
	int m_useEmissiveMap = 1;
};

struct RenderConfig
{
	Window* m_window = nullptr;
};

class Renderer
{
public:
	DX11Renderer*				m_DX11Renderer			= nullptr;
	DX12Renderer*				m_DX12Renderer			= nullptr;

	RenderConfig				m_config;
public:
	Renderer(RenderConfig const& config);
	~Renderer();

	void			StartUp();
	void			BeginFrame();
	void			EndFrame();
	void			ShutDown();

	Model*			LoadModel(char const* filePath, RootSig pipelineMode);

	BitmapFont*		CreateOrGetBitmapFont( const char* bitmapFontFilePathWithNoExtension );
	BitmapFont*		CreateBitmapFont(const char* bitmapFontFilePathWithNoExtension);
	BitmapFont*		GetFontForFileName(char const* imageFilePath);

	Texture*		CreateOrGetTextureFromFile(char const* imageFilePath);
	Texture*		CreateTextureFromFile(char const* imageFilePath);
	Texture*		CreateTextureFromImage(Image const& image);
	Texture*		CreateModifiableTexture(IntVec2 dimensions);
	Texture*		CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, uint8_t* texelData);
	Texture*		GetTextureForFileName(char const* imageFilePath);
	void			BindTexture(int textureSlot = 0, const Texture* texture = nullptr);
	
	VertexBuffer*	CreateVertexBuffer(size_t const size, std::wstring bufferDebugName = L"Vertex");
	void			CopyCPUToGPU(void const* data, size_t size, VertexBuffer*& vbo);
	void			BindVertexBuffer(VertexBuffer* buffer, int stride, PrimitiveType type);

	IndexBuffer*	CreateIndexBuffer(size_t const size);
	void			CopyCPUToGPU(void const* data, size_t size, IndexBuffer*& ibo);
	void			BindIndexBuffer(IndexBuffer* buffer);

	MeshBuffer*		CreateMeshBuffer(size_t const size, std::wstring bufferDebugName = L"Mesh");
	void			CopyCPUToGPU(void const* data, size_t size, MeshBuffer*& mbo);
	void			BindMeshBuffer(Model* mbo);

	ConstantBuffer* CreateConstantBuffer(size_t const size, std::wstring bufferDebugName = L"Constant");
	void			CopyCPUToGPU(void const* data, size_t size, ConstantBuffer* cbo);
	void			BindConstantBuffer(int slot, ConstantBuffer* cbo, RootSig pipelineMode);

	Shader*			CreateShader(char const* shaderName, char const* shaderSource, VertexType type);
	Shader*			CreateShader(char const* shaderName, VertexType type);
	bool			CompileShaderToByteCode(std::vector<unsigned char>& outByteCode, char const* name, char const* source, char const* entryPoint, char const* target);
	void			BindShader(Shader* shader = nullptr);
	
	void			SetModelConstants(RootSig pipelineMode, Mat44 const& modelMatrix = Mat44(), Rgba8 const& modelColor = Rgba8::WHITE, ConstantBuffer* modelCBO = nullptr);
	void			SetModelConstants(std::vector<Mat44> const& modelMatrix, std::vector<Rgba8> const& modelColor); // TODO
	void			SetDirectionalLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity, Vec3 worldEyePosition = Vec3::ZERO, LightDebug lightDebugFlags = {}, float minFalloff = 0.0f, float maxFalloff = 0.0f, float minFalloffMultiplier = 0.0f, float maxFalloffMultiplier = 0.0f);
	void			SetPointLightConstants(std::vector<Vec3> pointPosition, std::vector<Rgba8> pointColor);
	void			SetSpotLightConstants(std::vector<Vec3> spotLightPosition, std::vector<float> cutOff, std::vector<Vec3> spotLightDirection, std::vector<Rgba8> spotColor);

	void			SetBlendMode( BlendMode blendMode );
	void			SetBlendStatesIfChanged();

	void			SetSamplerMode(SamplerMode sampleMode);
	void			SetSamplerStatesIfChanged();

	void			SetRasterizerMode(RasterizerMode rasterizerMode);
	void			SetRasterizerStatesIfChanged();

	void			SetDepthMode(DepthMode depthMode);
	void			SetDepthStatesIfChanged();

	void			ClearScreen(Rgba8 const& clearColor);
	void			BeginCamera(Camera& camera, RootSig pipelineMode);
	void			EndCamera(Camera const& camera);
	void			DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, int vertexStride = 0, int vertexOffset = 0, PrimitiveType type = PrimitiveType::TRIANGLE_LIST);
	void			DrawVertexBufferIndexed(VertexBuffer* vbo, IndexBuffer* ibo, int vertexStride = 0, PrimitiveType type = PrimitiveType::TRIANGLE_LIST);
	void			DrawVertexArray(int numVertexes, Vertex_PCU const* vertexes);
	void			DrawVertexArrayIndexed(int numVertexes, Vertex_PCU const* vertexes, std::vector<unsigned int> const& indices);
	void			DrawVertexArray(int numVertexes, Vertex_PCUTBN const* vertexes);
	void			DrawVertexArrayIndexed(int numVertexes, Vertex_PCUTBN const* vertexes, std::vector<unsigned int> const& indices);
	void			DrawMesh(int numOfMeshlets);
public:/*
	ID3D11ComputeShader*			m_thresholdCSShader						= nullptr;
	ID3D11ComputeShader*			m_downsampleCSShader					= nullptr;
	ID3D11ComputeShader*			m_upsampleCSShader						= nullptr;
	ID3D11ComputeShader*			m_blurCSShader							= nullptr;
	ID3D11ComputeShader*			m_compositeCSShader						= nullptr;*/

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