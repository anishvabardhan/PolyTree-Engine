#include "Engine/Renderer/Renderer.hpp"

#include "Engine/Window/Window.hpp"
#include "Engine/Core/Image.hpp"
#include "Engine/Core/FileUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Renderer/Model.hpp"
#include "Engine/Renderer/Shader.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Renderer/BitmapFont.hpp"
#include "Engine/Renderer/IndexBuffer.hpp"
#include "Engine/Renderer/VertexBuffer.hpp"
#include "Engine/Renderer/ConstantBuffer.hpp"

#include "Game/EngineBuildPreferences.hpp"

#if DX11_RENDERER
#include "Engine/Renderer/DX11Renderer.hpp"
#elif DX12_RENDERER
#include "Engine/Renderer/DX12Renderer.hpp"
#endif

#include "ThirdParty/stb_image/stb_image.h"	

#define UNUSED(x) (void)x
#define DELETE_PTR(x) if(x) { delete x; x = nullptr; }

Renderer::Renderer(RenderConfig const& config)
{
	m_config = config;
#if DX11_RENDERER
	m_DX11Renderer = new DX11Renderer(m_config);
#elif DX12_RENDERER
	m_DX12Renderer = new DX12Renderer(m_config);
#endif
}

Renderer::~Renderer()
{
#if DX11_RENDERER
	DELETE_PTR(m_DX11Renderer);
#elif DX12_RENDERER
	DELETE_PTR(m_DX12Renderer);
#endif
}

void Renderer::StartUp()
{
#if DX11_RENDERER
	m_DX11Renderer->StartUp();
#elif DX12_RENDERER
	m_DX12Renderer->StartUp();
#endif
}

void Renderer::BeginFrame()
{
#if DX11_RENDERER
	m_DX11Renderer->BeginFrame();
#elif DX12_RENDERER
	m_DX12Renderer->BeginFrame();
#endif
}

void Renderer::EndFrame()
{
#if DX11_RENDERER
	m_DX11Renderer->EndFrame();
#elif DX12_RENDERER
	m_DX12Renderer->EndFrame();
#endif
}

void Renderer::ShutDown()
{
#if DX11_RENDERER
	m_DX11Renderer->ShutDown();
#elif DX12_RENDERER
	m_DX12Renderer->ShutDown();
#endif
}

Model* Renderer::LoadModel(char const* filePath, RootSig pipelineMode)
{
#if DX11_RENDERER
	UNUSED(filePath);
	UNUSED(pipelineMode);
	return nullptr;
#elif DX12_RENDERER
	return m_DX12Renderer->LoadModel(filePath, pipelineMode);
#endif
}

BitmapFont* Renderer::CreateOrGetBitmapFont(const char* bitmapFontFilePathWithNoExtension)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateOrGetBitmapFont(bitmapFontFilePathWithNoExtension);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateOrGetBitmapFont(bitmapFontFilePathWithNoExtension);
#endif
}

BitmapFont* Renderer::GetFontForFileName(char const* imageFilePath)
{
#if DX11_RENDERER
	return m_DX11Renderer->GetFontForFileName(imageFilePath);
#elif DX12_RENDERER
	return m_DX12Renderer->GetFontForFileName(imageFilePath);
#endif
}

Texture* Renderer::CreateOrGetTextureFromFile(char const* imageFilePath)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateOrGetTextureFromFile(imageFilePath);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateOrGetTextureFromFile(imageFilePath);
#endif
}

Texture* Renderer::CreateTextureFromFile(char const* imageFilePath)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateTextureFromFile(imageFilePath);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateTextureFromFile(imageFilePath);
#endif
}

Texture* Renderer::CreateTextureFromImage(Image const& image)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateTextureFromImage(image);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateTextureFromImage(image);
#endif
}

Texture* Renderer::CreateModifiableTexture(IntVec2 dimensions)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateModifiableTexture(dimensions);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateModifiableTexture(dimensions);
#endif
}

Texture* Renderer::CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, uint8_t* texelData)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateTextureFromData(name, dimensions, bytesPerTexel, texelData);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateTextureFromData(name, dimensions, bytesPerTexel, texelData);
#endif
}

Texture* Renderer::GetTextureForFileName(char const* imageFilePath)
{
#if DX11_RENDERER
	return m_DX11Renderer->GetTextureForFileName(imageFilePath);
#elif DX12_RENDERER
	return m_DX12Renderer->GetTextureForFileName(imageFilePath);
#endif
}

BitmapFont* Renderer::CreateBitmapFont(const char* bitmapFontFilePathWithNoExtension)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateBitmapFont(bitmapFontFilePathWithNoExtension);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateBitmapFont(bitmapFontFilePathWithNoExtension);
#endif
}

void Renderer::BindTexture(int textureSlot, const Texture* texture)
{
#if DX11_RENDERER
	m_DX11Renderer->BindTexture(textureSlot, texture);
#elif DX12_RENDERER
	UNUSED(textureSlot);
	m_DX12Renderer->BindTexture(texture);
#endif
}

VertexBuffer* Renderer::CreateVertexBuffer(size_t const size, std::wstring bufferDebugName)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateVertexBuffer(size);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateVertexBuffer(size, bufferDebugName);
#endif
}

void Renderer::CopyCPUToGPU(void const* data, size_t size, VertexBuffer*& vbo)
{
#if DX11_RENDERER
	m_DX11Renderer->CopyCPUToGPU(data, size, vbo);
#elif DX12_RENDERER
	return m_DX12Renderer->CopyCPUToGPU(data, size, vbo);
#endif
}

void Renderer::BindVertexBuffer(VertexBuffer* buffer, int stride, PrimitiveType type)
{
#if DX11_RENDERER
	m_DX11Renderer->BindVertexBuffer(buffer, stride, type);
#elif DX12_RENDERER
	UNUSED(type);
	return m_DX12Renderer->BindVertexBuffer(buffer, stride);
#endif
}

IndexBuffer* Renderer::CreateIndexBuffer(size_t const size)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateIndexBuffer(size);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateIndexBuffer(size);
#endif
}

void Renderer::CopyCPUToGPU(void const* data, size_t size, IndexBuffer*& ibo)
{
#if DX11_RENDERER
	m_DX11Renderer->CopyCPUToGPU(data, size, ibo);
#elif DX12_RENDERER
	return m_DX12Renderer->CopyCPUToGPU(data, size, ibo);
#endif
}

void Renderer::BindIndexBuffer(IndexBuffer* buffer)
{
#if DX11_RENDERER
	m_DX11Renderer->BindIndexBuffer(buffer);
#elif DX12_RENDERER
	return m_DX12Renderer->BindIndexBuffer(buffer);
#endif
}

MeshBuffer* Renderer::CreateMeshBuffer(size_t const size, std::wstring bufferDebugName)
{
	UNUSED(size);
	UNUSED(bufferDebugName);
#if DX12_RENDERER
	return m_DX12Renderer->CreateMeshBuffer(size, bufferDebugName);
#endif
	return nullptr;
}

void Renderer::CopyCPUToGPU(void const* data, size_t size, MeshBuffer*& mbo)
{
	UNUSED(data);
	UNUSED(size);
	UNUSED(mbo);
#if DX12_RENDERER
	return m_DX12Renderer->CopyCPUToGPU(data, size, mbo);
#endif
}

void Renderer::BindMeshBuffer(Model* mbo)
{
	UNUSED(mbo);
#if DX12_RENDERER
	return m_DX12Renderer->BindMeshBuffer(mbo);
#endif
}

ConstantBuffer* Renderer::CreateConstantBuffer(size_t const size, std::wstring bufferDebugName)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateConstantBuffer(size);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateConstantBuffer(size, bufferDebugName);
#endif
}

void Renderer::CopyCPUToGPU(void const* data, size_t size, ConstantBuffer* cbo)
{
#if DX11_RENDERER
	m_DX11Renderer->CopyCPUToGPU(data, size, cbo);
#elif DX12_RENDERER
	return m_DX12Renderer->CopyCPUToGPU(data, size, cbo);
#endif
}

void Renderer::BindConstantBuffer(int slot, ConstantBuffer* cbo, RootSig pipelineMode)
{
#if DX11_RENDERER
	UNUSED(pipelineMode);
	m_DX11Renderer->BindConstantBuffer(slot, cbo);
#elif DX12_RENDERER
	return m_DX12Renderer->BindConstantBuffer(slot, cbo, pipelineMode);
#endif
}

Shader* Renderer::CreateShader(char const* shaderName, char const* shaderSource, VertexType type)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateShader(shaderName, shaderSource, type);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateShader(shaderName, shaderSource, type);
#endif
}

Shader* Renderer::CreateShader(char const* shaderName, VertexType type)
{
#if DX11_RENDERER
	return m_DX11Renderer->CreateShader(shaderName, type);
#elif DX12_RENDERER
	return m_DX12Renderer->CreateShader(shaderName, type);
#endif
}

bool Renderer::CompileShaderToByteCode(std::vector<unsigned char>& outByteCode, char const* name, char const* source, char const* entryPoint, char const* target)
{
	UNUSED(outByteCode);
	UNUSED(name);
	UNUSED(source);
	UNUSED(entryPoint);
	UNUSED(target);
#if DX11_RENDERER
	return m_DX11Renderer->CompileShaderToByteCode(outByteCode, name, source, entryPoint, target);
#else
	return false;
#endif
}

void Renderer::BindShader(Shader* shader)
{
#if DX11_RENDERER
	m_DX11Renderer->BindShader(shader);
#elif DX12_RENDERER
	return m_DX12Renderer->BindShader(shader);
#endif
}

void Renderer::SetModelConstants(RootSig pipelineMode, Mat44 const& modelMatrix, Rgba8 const& modelColor, ConstantBuffer* modelCBO)
{
#if DX11_RENDERER
	UNUSED(pipelineMode);
	UNUSED(modelCBO);
	m_DX11Renderer->SetModelConstants(modelMatrix, modelColor);
#elif DX12_RENDERER
	m_DX12Renderer->SetModelConstants(pipelineMode, 0, modelMatrix, modelColor, modelCBO);
#endif
}

void Renderer::SetModelConstants(std::vector<Mat44> const& modelMatrix, std::vector<Rgba8> const& modelColor)
{
	UNUSED(modelMatrix);
	UNUSED(modelColor);

//#if DX11_RENDERER
//	//m_DX11Renderer->SetModelConstants(modelMatrix, modelColor);
//#elif DX12_RENDERER
//	m_DX12Renderer->SetModelConstants(modelMatrix, modelColor);
//#endif
}

void Renderer::SetDirectionalLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity, Vec3 worldEyePosition, LightDebug lightDebugFlags, float minFalloff, float maxFalloff, float minFalloffMultiplier, float maxFalloffMultiplier)
{
#if DX11_RENDERER
	m_DX11Renderer->SetDirectionalLightConstants(sunDirection, sunIntensity, ambientIntensity, worldEyePosition, lightDebugFlags, minFalloff, maxFalloff, minFalloffMultiplier, maxFalloffMultiplier);
#elif DX12_RENDERER
	UNUSED(worldEyePosition);
	UNUSED(lightDebugFlags);
	UNUSED(minFalloff);
	UNUSED(maxFalloff);
	UNUSED(minFalloffMultiplier);
	UNUSED(maxFalloffMultiplier);
	m_DX12Renderer->SetDirectionalLightConstants(sunDirection, sunIntensity, ambientIntensity);
#endif
}

void Renderer::SetPointLightConstants(std::vector<Vec3> pointPosition, std::vector<Rgba8> pointColor)
{
#if DX11_RENDERER
	m_DX11Renderer->SetPointLightConstants(pointPosition, pointColor);
#endif
}

void Renderer::SetSpotLightConstants(std::vector<Vec3> spotLightPosition, std::vector<float> cutOff, std::vector<Vec3> spotLightDirection, std::vector<Rgba8> spotColor)
{
#if DX11_RENDERER
	m_DX11Renderer->SetSpotLightConstants(spotLightPosition, cutOff, spotLightDirection, spotColor);
#endif
}

void Renderer::SetBlendMode(BlendMode blendMode)
{
#if DX11_RENDERER
	m_DX11Renderer->SetBlendMode(blendMode);
#elif DX12_RENDERER
	return m_DX12Renderer->SetBlendMode(blendMode);
#endif
}

void Renderer::SetBlendStatesIfChanged()
{
#if DX11_RENDERER
	m_DX11Renderer->SetBlendStatesIfChanged();
#elif DX12_RENDERER
	return m_DX12Renderer->SetBlendStatesIfChanged();
#endif
}

void Renderer::SetSamplerMode(SamplerMode sampleMode)
{
#if DX11_RENDERER
	m_DX11Renderer->SetSamplerMode(sampleMode);
#elif DX12_RENDERER
	return m_DX12Renderer->SetSamplerMode(sampleMode);
#endif
}

void Renderer::SetSamplerStatesIfChanged()
{
#if DX11_RENDERER
	m_DX11Renderer->SetSamplerStatesIfChanged();
#elif DX12_RENDERER
	return m_DX12Renderer->SetSamplerStatesIfChanged();
#endif
}

void Renderer::SetRasterizerMode(RasterizerMode rasterizerMode)
{
#if DX11_RENDERER
	m_DX11Renderer->SetRasterizerMode(rasterizerMode);
#elif DX12_RENDERER
	return m_DX12Renderer->SetRasterizerMode(rasterizerMode);
#endif
}

void Renderer::SetRasterizerStatesIfChanged()
{
#if DX11_RENDERER
	m_DX11Renderer->SetRasterizerStatesIfChanged();
#elif DX12_RENDERER
	return m_DX12Renderer->SetRasterizerStatesIfChanged();
#endif
}

void Renderer::SetDepthMode(DepthMode depthMode)
{
#if DX11_RENDERER
	m_DX11Renderer->SetDepthMode(depthMode);
#elif DX12_RENDERER
	return m_DX12Renderer->SetDepthMode(depthMode);
#endif
}

void Renderer::SetDepthStatesIfChanged()
{
#if DX11_RENDERER
	m_DX11Renderer->SetDepthStatesIfChanged(); 
#elif DX12_RENDERER
	return m_DX12Renderer->SetDepthStatesIfChanged();
#endif
}

void Renderer::ClearScreen(Rgba8 const& clearColor)
{
#if DX11_RENDERER
	m_DX11Renderer->ClearScreen(clearColor);
#elif DX12_RENDERER
	return m_DX12Renderer->ClearScreen(clearColor);
#endif
}

void Renderer::BeginCamera(Camera& camera, RootSig pipelineMode)
{
#if DX11_RENDERER
	UNUSED(pipelineMode);
	m_DX11Renderer->BeginCamera(camera);
#elif DX12_RENDERER
	return m_DX12Renderer->BeginCamera(camera, pipelineMode);
#endif
}

void Renderer::EndCamera(Camera const& camera)
{
#if DX11_RENDERER
	m_DX11Renderer->EndCamera(camera);
#elif DX12_RENDERER
	return m_DX12Renderer->EndCamera(camera);
#endif
}

void Renderer::DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, int vertexStride, int vertexOffset, PrimitiveType type)
{
#if DX11_RENDERER
	m_DX11Renderer->DrawVertexBuffer(vbo, vertexCount, vertexStride, vertexOffset, type);
#elif DX12_RENDERER
	UNUSED(type);
	return m_DX12Renderer->DrawVertexBuffer(vbo, vertexCount, vertexStride, vertexOffset, type);
#endif
}

void Renderer::DrawVertexBufferIndexed(VertexBuffer* vbo, IndexBuffer* ibo, int vertexStride, PrimitiveType type)
{
#if DX11_RENDERER
	m_DX11Renderer->DrawVertexBufferIndexed(vbo, ibo, vertexStride, type);
#elif DX12_RENDERER
	UNUSED(type);
	return m_DX12Renderer->DrawVertexBufferIndexed(vbo, ibo, vertexStride);
#endif
}

void Renderer::DrawVertexArray(int numVertexes, Vertex_PCU const* vertexes)
{
#if DX11_RENDERER
	m_DX11Renderer->DrawVertexArray(numVertexes, vertexes);
#elif DX12_RENDERER
	return m_DX12Renderer->DrawVertexArray(numVertexes, vertexes);
#endif
}

void Renderer::DrawVertexArrayIndexed(int numVertexes, Vertex_PCU const* vertexes, std::vector<unsigned int> const& indices)
{
#if DX11_RENDERER
	m_DX11Renderer->DrawVertexArrayIndexed(numVertexes, vertexes, indices);
#elif DX12_RENDERER
	return m_DX12Renderer->DrawVertexArrayIndexed(numVertexes, vertexes, indices);
#endif
}

void Renderer::DrawVertexArray(int numVertexes, Vertex_PCUTBN const* vertexes)
{
#if DX11_RENDERER
	m_DX11Renderer->DrawVertexArray(numVertexes, vertexes);
#elif DX12_RENDERER
	return m_DX12Renderer->DrawVertexArray(numVertexes, vertexes);
#endif
}

void Renderer::DrawVertexArrayIndexed(int numVertexes, Vertex_PCUTBN const* vertexes, std::vector<unsigned int> const& indices)
{
#if DX11_RENDERER
	m_DX11Renderer->DrawVertexArrayIndexed(numVertexes, vertexes, indices);
#elif DX12_RENDERER
	return m_DX12Renderer->DrawVertexArrayIndexed(numVertexes, vertexes, indices);
#endif
}
void Renderer::DrawMesh(int numOfMeshlets)
{
	UNUSED(numOfMeshlets);
#if DX11_RENDERER
#elif DX12_RENDERER
	return m_DX12Renderer->DrawMesh(numOfMeshlets);
#endif
}
//
//void Renderer::CreateThresholdCSShader(char const* filename)
//{/*
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
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_thresholdCSShader);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->CreateThresholdCSShader(filename);
//#endif
//}
//
//
//void Renderer::CreateDownsampleCSShader(char const* filename)
//{/*
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
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_downsampleCSShader);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->CreateDownsampleCSShader(filename);
//#endif
//}
//
//void Renderer::CreateBlurCSShader(char const* filename)
//{
//	/*
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
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_blurCSShader);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->CreateBlurCSShader(filename);
//#endif
//}
//
//void Renderer::CreateCompositeCSShader(char const* filename)
//{
//	/*
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
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_compositeCSShader);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->CreateCompositeCSShader(filename);
//#endif
//}
//
//void Renderer::RunThresholdCSShader(Texture& inTexture, Texture& outTexture)
//{/*
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
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->RunThresholdCSShader(inTexture, outTexture);
//#endif
//}
//
//void Renderer::RunDownsampleCSShader(Texture& inTexture, Texture& outTexture)
//{/*
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
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->RunDownsampleCSShader(inTexture, outTexture);
//#endif
//}
//
//void Renderer::CreateUpsampleCSShader(char const* filename)
//{/*
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
//	hr = m_device->CreateComputeShader(outByteCode.data(), outByteCode.size(), NULL, &m_upsampleCSShader);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->CreateUpsampleCSShader(filename);
//#endif
//}
//
//void Renderer::RunUpsampleCSShader(Texture& inTexture, Texture& outTexture)
//{/*
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
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->RunUpsampleCSShader(inTexture, outTexture);
//#endif
//}
//
//void Renderer::RunBlurCSShader(Texture& inTexture, Texture& outTexture)
//{/*
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
//	m_deviceContext->CSSetShaderResources(0, 2, ppSRVNULL);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->RunBlurCSShader(inTexture, outTexture);
//#endif
//}
//
//void Renderer::RunCompositeCSShader(Texture& inThresholdTexture, Texture& inBlurredTexture, Texture& outTexture)
//{/*
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
//	m_deviceContext->CSSetShaderResources(1, 2, ppSRVNULL);*/
//
//#ifdef DX11_RENDERER
//	m_DX11Renderer->RunCompositeCSShader(inThresholdTexture, inBlurredTexture, outTexture);
//#endif
//}
