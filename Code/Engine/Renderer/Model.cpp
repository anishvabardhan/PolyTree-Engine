#include "Engine/Renderer/Model.hpp"

#if DX12_RENDERER

#include "Engine/Math/Frustum.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Math/RandomNumberGenerator.hpp"
#include "Engine/Core/Image.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Renderer/MeshBuffer.hpp"
#include "Engine/Renderer/IndexBuffer.hpp"
#include "Engine/Renderer/VertexBuffer.hpp"
#include "Engine/Renderer/ConstantBuffer.hpp"

#include "ThirdParty/Meshoptimizer/src/meshoptimizer.h"

#include <memory>
#include <cassert>
#include <algorithm>
#include <unordered_set>

struct ModelRealtimeData
{
	uint32_t DrawnMeshletCount;
	uint32_t CulledMeshletCount;
	uint32_t DrawnVertexCount;
	uint32_t DrawnTriangleCount;
};

struct MeshInfo
{
	uint32_t MeshletCount;
};

struct MeshInstance
{
	int InstanceCount;
};

struct FrustumConstants
{
	Plane3D Planes[6];
	Vec3 CullViewPosition;
};

struct ModelConstants
{
	Vec4 ModelColor;
	Mat44 ModelMatrix;
};

Model::Model(std::vector<Vertex_PCU> modelVertices, std::vector<unsigned int> modelIndices)
{
	m_mesh = new Mesh();

	m_mesh->m_indices = modelIndices;
}

Model::Model(std::vector<MeshVertex_PCUTBN> modelVertices, std::vector<unsigned int> modelIndices, std::vector<Vec3> instancePositions, std::vector<float> instanceScales, bool isRock, int numOfInstances, std::string* textureFilePath, std::string* normalTextureFilePath, Vec3 worldPos)
{
	UNUSED(worldPos);

	m_mesh = new Mesh();

	m_mesh->m_meshVertices = modelVertices;
	m_mesh->m_indices = modelIndices;

	m_mesh->m_numOfInstances = numOfInstances;

	/*for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			for (int k = 0; k < 10; k++)
			{
				Mat44 transform;
				transform.SetTranslation3D(Vec3(i + worldPos.x, j + worldPos.y, k + worldPos.z));
				transform.AppendScaleUniform3D(0.001f);
				transform.AppendXRotation(90.0f);
				transform.AppendYRotation(90.0f);

				MeshletInstance instanceData;
				instanceData.InstanceTransform = transform;

				m_mesh->m_instanceData.push_back(instanceData);
			}
		}
	}*/

	m_mesh->ComputeInstanceData(instancePositions, instanceScales, isRock);

	if (textureFilePath == nullptr)
	{
		m_image = new Image(IntVec2(2, 2), Rgba8::WHITE);
	}
	else
	{
		m_image = new Image(textureFilePath->c_str());
	}

	if (normalTextureFilePath == nullptr)
	{
		m_normalImage = new Image(IntVec2(2, 2), Rgba8::WHITE);
	}
	else
	{
		m_normalImage = new Image(normalTextureFilePath->c_str());
	}
}

Model::Model(std::vector<MeshVertex_PCUTBN> modelVertices, std::vector<unsigned int> modelIndices, std::string* textureFilePath, std::string* normalTextureFilePath, Vec3 worldPos)
{
	UNUSED(worldPos);

	m_mesh = new Mesh();

	m_mesh->m_meshVertices = modelVertices;
	m_mesh->m_indices = modelIndices;

	m_mesh->m_numOfInstances = 1;

	Mat44 transform;

	MeshletInstance instanceData;
	instanceData.InstanceTransform = transform;
	instanceData.InstanceScale = 1.0f;

	m_mesh->m_instanceData.push_back(instanceData);

	if (textureFilePath == nullptr)
	{
		m_image = new Image(IntVec2(2, 2), Rgba8::WHITE);
	}
	else
	{
		m_image = new Image(textureFilePath->c_str());
	}

	if (normalTextureFilePath == nullptr)
	{
		m_normalImage = new Image(IntVec2(2, 2), Rgba8::WHITE);
	}
	else
	{
		m_normalImage = new Image(normalTextureFilePath->c_str());
	}
}

Model::~Model()
{
	DELETE_PTR(m_ibo);
	DELETE_PTR(m_vbo);
	DELETE_PTR(m_mbo);
	DELETE_PTR(m_mesh);
	DELETE_PTR(m_image);
	DELETE_PTR(m_texture);
	DELETE_PTR(m_modelCBO);
	DELETE_PTR(m_FrustumCBO);
	DELETE_PTR(m_normalImage);
	DELETE_PTR(m_meshInfoCBO);
	DELETE_PTR(m_meshCullData);
	DELETE_PTR(m_vertexIndices);
	DELETE_PTR(m_normalTexture);
	DELETE_PTR(m_meshInstanceCBO);
	DELETE_PTR(m_primitiveIndices);
	DELETE_PTR(m_meshletInstanceData);

	DX_SAFE_RELEASE(m_modelDescHeap);
}

Vec2 Mesh::WorldToScreenSpace(const Vec3& worldPoint, const Mat44& mvp)
{
	Vec4 clipSpacePos = mvp.TransformHomogeneous3D(Vec4(worldPoint.x, worldPoint.y, worldPoint.z, 1.0f));

	// Convert from NDC to screen space
	Vec2 screenPos;
	screenPos.x = (clipSpacePos.x * 0.5f + 0.5f) * g_theWindow->GetClientDimensions().x;
	screenPos.y = (1.0f - (clipSpacePos.y * 0.5f + 0.5f)) * g_theWindow->GetClientDimensions().y;

	return screenPos;
}

void Model::RenderBoundingBox() const
{
	g_theRenderer->SetBlendMode(BlendMode::ALPHA);
	g_theRenderer->SetDepthMode(DepthMode::ENABLED);
	g_theRenderer->SetSamplerMode(SamplerMode::POINT_CLAMP);
	g_theRenderer->SetRasterizerMode(RasterizerMode::WIREFRAME_CULL_NONE);
	g_theRenderer->BindTexture();
	g_theRenderer->BindShader();

	g_theRenderer->DrawVertexBuffer(m_screenBoundingBoxVBO, 6, sizeof(Vertex_PCU));
}

void Model::CreateBuffers()
{
	// CREATE MESH VBO
	m_vbo = g_theRenderer->CreateVertexBuffer((int)m_mesh->m_meshVertices.size() * sizeof(MeshVertex_PCUTBN), std::wstring(L"Mesh Vertex"));
	g_theRenderer->CopyCPUToGPU(m_mesh->m_meshVertices.data(), (int)m_mesh->m_meshVertices.size() * sizeof(MeshVertex_PCUTBN), m_vbo);

	// INITIALIZE MODEL CBOs
	m_modelCBO = g_theRenderer->CreateConstantBuffer(sizeof(ModelConstants), std::wstring(L"Mesh Constant"));
	//m_worldBoundingBoxCBO = g_theRenderer->CreateConstantBuffer(sizeof(ModelConstants), std::wstring(L"Mesh Bounding Box"));
	//m_screenBoundingBoxCBO = g_theRenderer->CreateConstantBuffer(sizeof(ModelConstants), std::wstring(L"Mesh Bounding Box"));
	m_meshInfoCBO = g_theRenderer->CreateConstantBuffer(sizeof(MeshInfo), std::wstring(L"Mesh Info Constant"));
	m_meshInstanceCBO = g_theRenderer->CreateConstantBuffer(sizeof(MeshInstance), std::wstring(L"Mesh Instance Constant"));
	m_FrustumCBO = g_theRenderer->CreateConstantBuffer(sizeof(FrustumConstants), std::wstring(L"Frustum Constant"));

	// CREATE MESH BUFFER
	m_mbo = g_theRenderer->CreateMeshBuffer(m_mesh->m_meshlets.size() * sizeof(Meshlet), std::wstring(L"Mesh Meshlet"));
	g_theRenderer->CopyCPUToGPU(m_mesh->m_meshlets.data(), m_mesh->m_meshlets.size() * sizeof(Meshlet), m_mbo);

	// CREATE VERTEX INDICES BUFFER
	m_vertexIndices = g_theRenderer->CreateMeshBuffer(m_mesh->m_uniqueVertexIndices.size() * sizeof(uint32_t), std::wstring(L"Mesh Unique Verts"));
	g_theRenderer->CopyCPUToGPU(m_mesh->m_uniqueVertexIndices.data(), m_mesh->m_uniqueVertexIndices.size() * sizeof(uint32_t), m_vertexIndices);

	// CREATE PRIMITIVE INDICES BUFFER
	m_primitiveIndices = g_theRenderer->CreateMeshBuffer(m_mesh->m_primitiveIndices.size() * sizeof(PackedPrimitive), std::wstring(L"Mesh Unique Primitives"));
	g_theRenderer->CopyCPUToGPU(m_mesh->m_primitiveIndices.data(), m_mesh->m_primitiveIndices.size() * sizeof(PackedPrimitive), m_primitiveIndices);

	// CREATE MESH CULL DATA BUFFER
	m_meshCullData = g_theRenderer->CreateMeshBuffer(m_mesh->m_cullData.size() * sizeof(CullData), std::wstring(L"Mesh Cull Data"));
	g_theRenderer->CopyCPUToGPU(m_mesh->m_cullData.data(), m_mesh->m_cullData.size() * sizeof(CullData), m_meshCullData);

	// CREATE MESHLET INSTANCE DATA BUFFER
	m_meshletInstanceData = g_theRenderer->CreateMeshBuffer(m_mesh->m_instanceData.size() * sizeof(MeshletInstance), std::wstring(L"Meshlet Instance Data"));
	g_theRenderer->CopyCPUToGPU(m_mesh->m_instanceData.data(), m_mesh->m_instanceData.size() * sizeof(MeshletInstance), m_meshletInstanceData);

	// CREATE MESHLET LAST FRAME VISIBILITY DATA BUFFER
	for (int i = 0; i < m_mesh->m_meshlets.size() * m_mesh->m_numOfInstances; i++)
	{
		m_mesh->m_meshletsVisibility.push_back(0);
	}

	m_meshletsVisibilityBuffer = g_theRenderer->m_DX12Renderer->CreateRWMeshBuffer(m_mesh->m_meshletsVisibility.size() * sizeof(uint32_t), std::wstring(L"Meshlet Visibility Data"));
	g_theRenderer->m_DX12Renderer->CopyRWMeshBufferCPUToGPU(m_mesh->m_meshletsVisibility.data(), m_mesh->m_meshletsVisibility.size() * sizeof(uint32_t), m_meshletsVisibilityBuffer);
	//----------------------------------------------
}

void Model::CreateAndSetDescriptorHeap()
{
	// Create the Descriptor heap for the model
	D3D12_DESCRIPTOR_HEAP_DESC srvDescHeap = {};
	srvDescHeap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	srvDescHeap.NumDescriptors = 17;
	srvDescHeap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	srvDescHeap.NodeMask = 0;

	HRESULT hr = g_theRenderer->m_DX12Renderer->m_device->CreateDescriptorHeap(&srvDescHeap, IID_PPV_ARGS(&m_modelDescHeap));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 srv desc heap!"); 
	}

	UINT srvDescriptorSize = g_theRenderer->m_DX12Renderer->m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

	m_modelCPUDescHandle = m_modelDescHeap->GetCPUDescriptorHandleForHeapStart();

	// Create the diffuse texture
	m_texture = CreateModelTexture(m_image);
	m_modelCPUDescHandle.ptr += srvDescriptorSize;

	// Create the SRV for the meshlet data
	g_theRenderer->m_DX12Renderer->CreateShaderResourceView(m_mbo->m_defaultBuffer, &m_modelCPUDescHandle, DXGI_FORMAT_UNKNOWN, D3D12_SRV_DIMENSION_BUFFER, sizeof(Meshlet), (UINT)m_mesh->m_meshlets.size());
	m_modelCPUDescHandle.ptr += srvDescriptorSize;

	// Create the SRV for the mesh vertices data
	g_theRenderer->m_DX12Renderer->CreateShaderResourceView(m_vbo->m_defaultBuffer, &m_modelCPUDescHandle, DXGI_FORMAT_UNKNOWN, D3D12_SRV_DIMENSION_BUFFER, sizeof(MeshVertex_PCUTBN), (UINT)m_mesh->m_meshVertices.size());
	m_modelCPUDescHandle.ptr += srvDescriptorSize;

	// Create the SRV for the unique vertex indices data
	g_theRenderer->m_DX12Renderer->CreateShaderResourceView(m_vertexIndices->m_defaultBuffer, &m_modelCPUDescHandle, DXGI_FORMAT_UNKNOWN, D3D12_SRV_DIMENSION_BUFFER, sizeof(uint32_t), (UINT)m_mesh->m_uniqueVertexIndices.size());
	m_modelCPUDescHandle.ptr += srvDescriptorSize;

	// Create the SRV for the unique primitive indices data
	g_theRenderer->m_DX12Renderer->CreateShaderResourceView(m_primitiveIndices->m_defaultBuffer, &m_modelCPUDescHandle, DXGI_FORMAT_UNKNOWN, D3D12_SRV_DIMENSION_BUFFER, sizeof(PackedPrimitive), (UINT)m_mesh->m_primitiveIndices.size());
	m_modelCPUDescHandle.ptr += srvDescriptorSize;

	// Create the SRV for the meshlet cull data
	g_theRenderer->m_DX12Renderer->CreateShaderResourceView(m_meshCullData->m_defaultBuffer, &m_modelCPUDescHandle, DXGI_FORMAT_UNKNOWN, D3D12_SRV_DIMENSION_BUFFER, sizeof(CullData), (UINT)m_mesh->m_cullData.size());
	m_modelCPUDescHandle.ptr += srvDescriptorSize;

	// Create the SRV for the meshlet instance data
	g_theRenderer->m_DX12Renderer->CreateShaderResourceView(m_meshletInstanceData->m_defaultBuffer, &m_modelCPUDescHandle, DXGI_FORMAT_UNKNOWN, D3D12_SRV_DIMENSION_BUFFER, sizeof(MeshletInstance), (UINT)m_mesh->m_instanceData.size());
	m_modelCPUDescHandle.ptr += srvDescriptorSize;

	// Create the normal texture
	if (m_normalImage)
	{
		m_normalTexture = CreateModelTexture(m_normalImage);
	}

	// Create UAV Hi-ZBuffers
	for (int i = 0; i < 8; i++)
	{
		m_modelCPUDescHandle.ptr += srvDescriptorSize;
		g_theRenderer->m_DX12Renderer->CreateUnorderedAccessView(g_theRenderer->m_DX12Renderer->m_hiZBuffers[i]->m_defaultBuffer, &m_modelCPUDescHandle, DXGI_FORMAT_R32_FLOAT, D3D12_UAV_DIMENSION_TEXTURE2D);
	}

	// Create UAV Visibility buffer 
	m_modelCPUDescHandle.ptr += srvDescriptorSize;
	g_theRenderer->m_DX12Renderer->CreateUnorderedAccessView(m_meshletsVisibilityBuffer->m_defaultBuffer, &m_modelCPUDescHandle, DXGI_FORMAT_UNKNOWN, D3D12_UAV_DIMENSION_BUFFER, sizeof(uint32_t), m_mesh->m_meshlets.size() * m_mesh->m_numOfInstances);
}

void Model::CreateUAVBuffer()
{
	D3D12_HEAP_PROPERTIES hpUpload{};
	hpUpload.Type = D3D12_HEAP_TYPE_UPLOAD;
	hpUpload.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpUpload.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpUpload.CreationNodeMask = 0;
	hpUpload.VisibleNodeMask = 0;

	D3D12_HEAP_PROPERTIES hpDefault{};
	hpDefault.Type = D3D12_HEAP_TYPE_DEFAULT;
	hpDefault.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpDefault.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpDefault.CreationNodeMask = 0;
	hpDefault.VisibleNodeMask = 0;

	D3D12_RESOURCE_DESC bufferDesc = {};
	bufferDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	bufferDesc.Width = sizeof(ModelRealtimeData) * m_mesh->m_numOfInstances; // Total size in bytes
	bufferDesc.Height = 1;
	bufferDesc.DepthOrArraySize = 1;
	bufferDesc.MipLevels = 1;
	bufferDesc.Format = DXGI_FORMAT_UNKNOWN;
	bufferDesc.SampleDesc.Count = 1;
	bufferDesc.SampleDesc.Quality = 0;
	bufferDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	bufferDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

	// Create the resource
	HRESULT hr = g_theRenderer->m_DX12Renderer->m_device->CreateCommittedResource(
		&hpDefault,                      // Heap properties (usually D3D12_HEAP_TYPE_DEFAULT)
		D3D12_HEAP_FLAG_NONE,            // Heap flags
		&bufferDesc,                     // Resource description
		D3D12_RESOURCE_STATE_COMMON,  // Initial resource state
		nullptr,                         // Optimized clear value (not used here)
		IID_PPV_ARGS(&m_structuredBuffer)
	);

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 UAV buffer!"); 
	}

	D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
	uavDesc.ViewDimension = D3D12_UAV_DIMENSION_BUFFER;
	uavDesc.Format = DXGI_FORMAT_UNKNOWN; // Structured buffers use DXGI_FORMAT_UNKNOWN
	uavDesc.Buffer.FirstElement = 0;
	uavDesc.Buffer.NumElements = m_mesh->m_numOfInstances; // Total number of elements
	uavDesc.Buffer.StructureByteStride = sizeof(ModelRealtimeData); // Size of each element in bytes
	uavDesc.Buffer.CounterOffsetInBytes = 0; // If using a counter (optional)
	uavDesc.Buffer.Flags = D3D12_BUFFER_UAV_FLAG_NONE; 

	g_theRenderer->m_DX12Renderer->m_device->CreateUnorderedAccessView(
		m_structuredBuffer,       // Resource
		nullptr,                // Counter resource (optional, typically null)
		&uavDesc,               // UAV description
		m_modelCPUDescHandle // CPU handle in descriptor heap
	);
}

Texture* Model::CreateModelTexture(Image* image)
{
	Texture* texture = new Texture();
	texture->m_dimensions = image->GetDimensions();
	texture->m_name = image->GetImageFilePath();

	D3D12_HEAP_PROPERTIES hpUpload{};
	hpUpload.Type = D3D12_HEAP_TYPE_UPLOAD;
	hpUpload.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpUpload.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpUpload.CreationNodeMask = 0;
	hpUpload.VisibleNodeMask = 0;

	D3D12_HEAP_PROPERTIES hpDefault{};
	hpDefault.Type = D3D12_HEAP_TYPE_DEFAULT;
	hpDefault.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpDefault.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpDefault.CreationNodeMask = 0;
	hpDefault.VisibleNodeMask = 0;

	D3D12_RESOURCE_DESC uploadResourceDesc = {};
	uploadResourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	uploadResourceDesc.Alignment = 0;
	uploadResourceDesc.Width = image->GetDimensions().x * image->GetDimensions().y * sizeof(unsigned char) * 4;
	uploadResourceDesc.Height = 1;
	uploadResourceDesc.DepthOrArraySize = 1;
	uploadResourceDesc.MipLevels = 1;
	uploadResourceDesc.Format = DXGI_FORMAT_UNKNOWN;
	uploadResourceDesc.SampleDesc = {1, 0};
	uploadResourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	uploadResourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	D3D12_RESOURCE_DESC defaultResourceDesc = {};
	defaultResourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
	defaultResourceDesc.Alignment = 0;
	defaultResourceDesc.Width = image->GetDimensions().x;
	defaultResourceDesc.Height = image->GetDimensions().y;
	defaultResourceDesc.DepthOrArraySize = 1;
	defaultResourceDesc.MipLevels = static_cast<UINT16>(log2(std::max(image->GetDimensions().x, image->GetDimensions().y)) + 1);
	defaultResourceDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	defaultResourceDesc.SampleDesc = {1, 0};
	defaultResourceDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
	defaultResourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	HRESULT hr = g_theRenderer->m_DX12Renderer->m_device->CreateCommittedResource(&hpUpload, D3D12_HEAP_FLAG_NONE, &uploadResourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&texture->m_uploadBuffer));
	hr = g_theRenderer->m_DX12Renderer->m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &defaultResourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&texture->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 texture!"); 
	}

	texture->m_uploadBuffer->SetName(L"Mesh Texture Upload Buffer");
	texture->m_defaultBuffer->SetName(L"Mesh Texture Default Buffer");

	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
	srvDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
	srvDesc.Texture2D.MipLevels = 1;
	srvDesc.Texture2D.MostDetailedMip = 0;
	srvDesc.Texture2D.PlaneSlice = 0;
	srvDesc.Texture2D.ResourceMinLODClamp = 0.0f;

	g_theRenderer->m_DX12Renderer->m_device->CreateShaderResourceView(texture->m_defaultBuffer, &srvDesc, m_modelCPUDescHandle);

	void* dest = nullptr;

	D3D12_RANGE uploadRange;
	uploadRange.Begin = 0;
	uploadRange.End = image->GetDimensions().x * image->GetDimensions().y * sizeof(unsigned char);
	texture->m_uploadBuffer->Map(0, &uploadRange, &dest);
	memcpy(dest, image->GetRawData(), image->GetDimensions().x * image->GetDimensions().y * sizeof(unsigned char) * 4);
	texture->m_uploadBuffer->Unmap(0, &uploadRange);

	D3D12_BOX textureSizeAsBox;
	textureSizeAsBox.left = textureSizeAsBox.top = textureSizeAsBox.front = 0;
	textureSizeAsBox.right = image->GetDimensions().x;
	textureSizeAsBox.bottom = image->GetDimensions().y;
	textureSizeAsBox.back = 1;
	D3D12_TEXTURE_COPY_LOCATION txtcSrc, txtcDst;
	txtcSrc.pResource = texture->m_uploadBuffer;
	txtcSrc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
	txtcSrc.PlacedFootprint.Offset = 0;
	txtcSrc.PlacedFootprint.Footprint.Width = image->GetDimensions().x;
	txtcSrc.PlacedFootprint.Footprint.Height = image->GetDimensions().y;
	txtcSrc.PlacedFootprint.Footprint.Depth = 1;
	txtcSrc.PlacedFootprint.Footprint.RowPitch = sizeof(unsigned char) * 4 * image->GetDimensions().x;
	txtcSrc.PlacedFootprint.Footprint.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	txtcDst.pResource = texture->m_defaultBuffer;
	txtcDst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
	txtcDst.SubresourceIndex = 0;
	g_theRenderer->m_DX12Renderer->m_commandList->CopyTextureRegion(&txtcDst, 0, 0, 0, &txtcSrc, &textureSizeAsBox);

	g_theRenderer->m_DX12Renderer->ExecuteCommandList();
	g_theRenderer->m_DX12Renderer->SignalAndWait();
	g_theRenderer->m_DX12Renderer->ResetCommandList();

	return texture;
}

void Model::InitializeGPUData()
{ 
	// COMPUTE MESHLET DATA
	m_mesh->ComputeMeshlets();

	// COMPUTE MESHLET CULL DATA
	m_mesh->m_cullData = m_mesh->ComputeMeshletCullData();

	// INITIALIZE GPU RESOURCES
	CreateBuffers();
	CreateAndSetDescriptorHeap();
}

void Model::SetMeshInfoConstants(uint32_t meshletCount)
{
	MeshInfo meshInfo;

	meshInfo.MeshletCount = meshletCount;

	g_theRenderer->m_DX12Renderer->CopyCPUToGPU(&meshInfo, m_meshInfoCBO->m_size, m_meshInfoCBO);
	g_theRenderer->m_DX12Renderer->BindConstantBuffer(3, m_meshInfoCBO, RootSig::MESH_SHADER_PIPELINE);
}

void Model::SetMeshInstanceConstants(int instanceCount)
{
	MeshInstance meshInstance;

	meshInstance.InstanceCount = instanceCount;

	g_theRenderer->m_DX12Renderer->CopyCPUToGPU(&meshInstance, m_meshInstanceCBO->m_size, m_meshInstanceCBO);
	g_theRenderer->m_DX12Renderer->BindConstantBuffer(4, m_meshInstanceCBO, RootSig::MESH_SHADER_PIPELINE);
}

void Model::SetFrustumConstants(Frustum* frustum, Vec3 cullCamPosition)
{
	FrustumConstants worldCamFrustumConstants;

	worldCamFrustumConstants.Planes[0] = frustum->m_nearPlane;
	worldCamFrustumConstants.Planes[1] = frustum->m_farPlane;

	worldCamFrustumConstants.Planes[2] = frustum->m_rightPlane;
	worldCamFrustumConstants.Planes[3] = frustum->m_leftPlane;
	
	worldCamFrustumConstants.Planes[4] = frustum->m_topPlane;
	worldCamFrustumConstants.Planes[5] = frustum->m_bottomPlane;

	worldCamFrustumConstants.CullViewPosition = cullCamPosition;

	g_theRenderer->m_DX12Renderer->CopyCPUToGPU(&worldCamFrustumConstants, m_FrustumCBO->m_size, m_FrustumCBO);
	g_theRenderer->m_DX12Renderer->BindConstantBuffer(5, m_FrustumCBO, RootSig::MESH_SHADER_PIPELINE);
}

int Mesh::ComputeReuseScore(const InlineMeshlet& meshlet, uint32_t (&triIndices)[3])
{
	int count = 0;

	for (int i = 0; i < meshlet.m_uniqueVertexIndices.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (meshlet.m_uniqueVertexIndices[i] == triIndices[j])
			{
				count++;
			}
		}
	}

	return count;
}

Vec3 Mesh::ComputeNormals(Vec3* triVerts)
{
	Vec3 p0 = triVerts[0];
	Vec3 p1 = triVerts[1];
	Vec3 p2 = triVerts[2];

	Vec3 e1 = p1 - p0;
	Vec3 e2 = p2 - p0;

	Vec3 normal = CrossProduct3D(e1, e2).GetNormalized();

	return normal;
}

BoundingSphere Mesh::ComputeMinimumBoundingSphere(std::vector<Vec3> verts, size_t count)
{
	uint32_t minAxis[3] = {0, 0, 0};
	uint32_t maxAxis[3] = {0, 0, 0};

	float min = FLT_MAX;
	float max = 0.0f;

	int minIndex = INT_MAX;
	int maxIndex = 0;

	// X-AXIS MIN MAX VERTEX

	for (int i = 0; i < count; i++)
	{
		if (min > verts[i].x)
		{
			min = verts[i].x;
			minIndex = i;
		}

		if (max < verts[i].x)
		{				  
			max = verts[i].x;
			maxIndex = i;
		}
	}

	minAxis[0] = minIndex;
	maxAxis[0] = maxIndex;

	// Y-AXIS MIN MAX VERTEX

	min = FLT_MAX;
	max = 0.0f;

	minIndex = INT_MAX;
	maxIndex = 0;

	for (int i = 0; i < count; i++)
	{
		if (min > verts[i].y)
		{
			min = verts[i].y;
			minIndex = i;
		}

		if (max < verts[i].y)
		{
			max = verts[i].y;
			maxIndex = i;
		}
	}

	minAxis[1] = minIndex;
	maxAxis[1] = maxIndex;

	// Z-AXIS MIN MAX VERTEX

	min = FLT_MAX;
	max = 0.0f;

	minIndex = INT_MAX;
	maxIndex = 0;

	for (int i = 0; i < count; i++)
	{
		if (min > verts[i].z)
		{
			min = verts[i].z;
			minIndex = i;
		}

		if (max < verts[i].z)
		{
			max = verts[i].z;
			maxIndex = i;
		}
	}

	minAxis[2] = minIndex;
	maxAxis[2] = maxIndex;

	int maxDistAxis = 0;
	float maxDistSquared = 0.0f;

	for (int i = 0; i < 3; i++)
	{
		minIndex = minAxis[i];
		maxIndex = maxAxis[i];

		float distSquared = GetDistanceSquared3D(verts[minIndex], verts[maxIndex]);

		if (distSquared > maxDistSquared)
		{
			maxDistSquared = distSquared;
			maxDistAxis = i;
		}
	}

	Vec3 p1 = verts[minAxis[maxDistAxis]];
	Vec3 p2 = verts[maxAxis[maxDistAxis]];

	Vec3 currentCenter = (p1 + p2) * 0.5f;

	float currentRadius = GetDistance3D(p2, p1) * 0.5f;
	float radiusSq = currentRadius * currentRadius;

	for (int i = 0; i < count; i++)
	{
		Vec3 point = verts[i];

		float distSq = GetDistanceSquared3D(point, currentCenter);

		if (distSq > radiusSq)
		{
			float dist = sqrtf(distSq);
			float k = (currentRadius / dist) * 0.5f + 0.5f;

			currentCenter = currentCenter * k + point * (1 - k);
			currentRadius = (currentRadius + dist) * 0.5f;
		}
	}

	return BoundingSphere(currentCenter, currentRadius);
}

bool Mesh::AddToMeshlet(InlineMeshlet& meshlet, uint32_t(&tri)[3])
{
	if (meshlet.m_uniqueVertexIndices.size() == MAX_VERTICES_PER_MESHLET)
		return false;

	if (meshlet.m_primitiveIndices.size() == MAX_TRIANGLES_PER_MESHLET)
		return false;

	if (meshlet.m_uniqueVertexIndices.size() == 0)
	{
		meshlet.m_uniqueVertexIndices.push_back(tri[0]);
		meshlet.m_uniqueVertexIndices.push_back(tri[1]);
		meshlet.m_uniqueVertexIndices.push_back(tri[2]);

		PackedPrimitive newTriangle;

		newTriangle.m_i0 = tri[0];
		newTriangle.m_i1 = tri[1];
		newTriangle.m_i2 = tri[2];

		meshlet.m_primitiveIndices.push_back(newTriangle);
	}
	else
	{
		int newCount = 3;
		std::vector<uint32_t> newVertsToAdd(3, (uint32_t)-1);

		for (int j = 0; j < 3; j++)
		{
			if (std::find(meshlet.m_uniqueVertexIndices.begin(), meshlet.m_uniqueVertexIndices.end(), tri[j]) != meshlet.m_uniqueVertexIndices.end())
			{
				newCount--;
			}
			else
			{
				newVertsToAdd[j] = tri[j];
			}
		}

		if(meshlet.m_uniqueVertexIndices.size() + newCount > MAX_VERTICES_PER_MESHLET)
			return false;

		for (int i = 0; i < 3; i++)
		{
			if (newVertsToAdd[i] != (uint32_t)-1)
			{
				meshlet.m_uniqueVertexIndices.push_back(newVertsToAdd[i]);
			}
		}

		if(meshlet.m_primitiveIndices.size() + 1 > MAX_TRIANGLES_PER_MESHLET)
			return false;

		PackedPrimitive newTriangle;

		newTriangle.m_i0 = tri[0];
		newTriangle.m_i1 = tri[1];
		newTriangle.m_i2 = tri[2];

		meshlet.m_primitiveIndices.push_back(newTriangle);
	}

	return true;
}

void Mesh::BuildAdjacencyList( const uint32_t* indices, uint32_t indexCount, std::vector<uint32_t>& adjacency )
{
	std::unordered_map<Edge, std::vector<uint32_t>, EdgeHash, EdgeEqual> edgeToTriangles;

	int triangleCount = indexCount / 3;

	for (int i = 0; i < triangleCount; i++)
	{
		uint32_t i0 = indices[i * 3 + 0];
		uint32_t i1 = indices[i * 3 + 1];
		uint32_t i2 = indices[i * 3 + 2];

		Edge e0 = Edge(i0, i1);
		Edge e1 = Edge(i1, i2);
		Edge e2 = Edge(i0, i2);

		edgeToTriangles[e0].push_back(i);
		edgeToTriangles[e1].push_back(i);
		edgeToTriangles[e2].push_back(i);
	}

	for (auto edge : edgeToTriangles)
	{
		size_t numOfSharedTrianglesOnEdge = edge.second.size();

		if (numOfSharedTrianglesOnEdge == 2)
		{
			adjacency.push_back(edge.second[0]);
			adjacency.push_back(edge.second[1]);
		}
	}
}

bool Mesh::IsMeshletFull(InlineMeshlet& meshlet)
{
	return (meshlet.m_uniqueVertexIndices.size() == MAX_VERTICES_PER_MESHLET || meshlet.m_primitiveIndices.size() == MAX_TRIANGLES_PER_MESHLET);
}

bool CompareTestScores(const std::pair<uint32_t, float>& a, const std::pair<uint32_t, float>& b)
{
	return a.second > b.second;
}

Vec4 QuantizeSNorm(Vec4 value)
{
	Vec4 quantized;

	quantized.x = (GetClamped(value.x, -1.0f, 1.0f) * 0.5f + 0.5f) * 255.0f;
	quantized.y = (GetClamped(value.y, -1.0f, 1.0f) * 0.5f + 0.5f) * 255.0f;
	quantized.z = (GetClamped(value.z, -1.0f, 1.0f) * 0.5f + 0.5f) * 255.0f;
	quantized.w = (GetClamped(value.w, -1.0f, 1.0f) * 0.5f + 0.5f) * 255.0f;

	return quantized;
}

Vec4 QuantizeUNorm(Vec4 value)
{
	Vec4 quantized;

	quantized.x = (GetClamped(value.x, 0.0f, 1.0f)) * 255.0f;
	quantized.y = (GetClamped(value.y, 0.0f, 1.0f)) * 255.0f;
	quantized.z = (GetClamped(value.z, 0.0f, 1.0f)) * 255.0f;
	quantized.w = (GetClamped(value.w, 0.0f, 1.0f)) * 255.0f;

	return quantized;
}

float Mesh::ComputeScore(const InlineMeshlet& meshlet, BoundingSphere sphere, BoundingSphere normal, uint32_t(&triIndices)[3], Vec3* triVerts)
{
	const float reuseWeight = 0.33f;
	const float locWeight = 0.33f;
	const float oriWeight = 0.33f;

	// Vertex reuse
	uint32_t reuse = ComputeReuseScore(meshlet, triIndices);
	float reuseScore = 1 - ((float(reuse)) / 3.0f);

	// Distance from center point
	float maxSq = 0.0f;
	for (uint32_t i = 0; i < 3u; ++i)
	{
		Vec3 v = sphere.m_center - triVerts[i];
		maxSq = std::max(maxSq, DotProduct3D(v, v));
	}
	float r = sphere.m_radius;
	float r2 = r * r;
	float locScore = std::log2(maxSq / r2 + 1);

	Vec3 triNormal = ComputeNormals(triVerts);
	float dotValue = DotProduct3D(triNormal, normal.m_center);
	float oriScore = (1.0f - dotValue) * 0.5f;

	float b = (reuseWeight * reuseScore) + (locWeight * locScore) + (oriWeight * oriScore);

	return b;
}

void Mesh::Meshletize(std::vector<InlineMeshlet>& output, const uint32_t* indices, uint32_t indexCount, const std::vector<MeshVertex_PCUTBN> positions, uint32_t vertexCount)
{
	UNUSED(vertexCount);

	const uint32_t triCount = indexCount / 3;

	// Build a primitive adjacency list
	std::vector<uint32_t> adjacency;
	adjacency.resize(indexCount);

	BuildAdjacencyList(indices, indexCount, adjacency);

	// Rest our outputs
	output.clear();
	output.emplace_back();
	auto* curr = &output.back();

	// Bitmask of all triangles in mesh to determine whether a specific one has been added.
	std::vector<bool> checklist;
	checklist.resize(triCount);

	std::vector<Vec3> m_positions;
	std::vector<Vec3> m_normals;
	std::vector<std::pair<uint32_t, float>> candidates;
	std::unordered_set<uint32_t> candidateCheck;

	BoundingSphere psphere;
	BoundingSphere normal;

	// Arbitrarily start at triangle zero.
	uint32_t triIndex = 0;
	candidates.push_back(std::make_pair(triIndex, 0.0f));
	candidateCheck.insert(triIndex);

	// Continue adding triangles until 
	while (!candidates.empty())
	{
		uint32_t index = candidates.back().first;
		candidates.pop_back();

		uint32_t tri[3] =
		{
			indices[index * 3],
			indices[index * 3 + 1],
			indices[index * 3 + 2],
		};

		assert(tri[0] < vertexCount);
		assert(tri[1] < vertexCount);
		assert(tri[2] < vertexCount);

		// Try to add triangle to meshlet
		if (AddToMeshlet(*curr, tri))
		{
			// Success! Mark as added.
			checklist[index] = true;

			// Add m_positions & normal to list
			Vec3 points[3] =
			{
				positions[tri[0]].m_position,
				positions[tri[1]].m_position,
				positions[tri[2]].m_position,
			};

			m_positions.push_back(points[0]);
			m_positions.push_back(points[1]);
			m_positions.push_back(points[2]);

			Vec3 triNormal;
			triNormal = ComputeNormals(points);
			m_normals.push_back(triNormal);

			// Compute new bounding sphere & normal axis
			psphere = ComputeMinimumBoundingSphere(m_positions, static_cast<uint32_t>(m_positions.size()));

			BoundingSphere nsphere = ComputeMinimumBoundingSphere(m_normals, static_cast<UINT32>(m_normals.size()));

			normal.m_center = nsphere.m_center.GetNormalized();
			normal.m_radius = nsphere.m_radius;

			// Find and add all applicable adjacent triangles to candidate list
			const uint32_t adjIndex = index * 3;

			uint32_t adj[3] =
			{
				adjacency[adjIndex],
				adjacency[adjIndex + 1],
				adjacency[adjIndex + 2],
			};

			for (uint32_t i = 0; i < 3u; ++i)
			{
				// Invalid triangle in adjacency slot
				if (adj[i] == -1)
					continue;

				// Already processed triangle
				if (checklist[adj[i]])
					continue;

				// Triangle already in the candidate list
				if (candidateCheck.count(adj[i]))
					continue;

				candidates.push_back(std::make_pair(adj[i], FLT_MAX));
				candidateCheck.insert(adj[i]);
			}

			// Re-score remaining candidate triangles
			for (uint32_t i = 0; i < static_cast<uint32_t>(candidates.size()); ++i)
			{
				uint32_t candidate = candidates[i].first;

				uint32_t triIndices[3] =
				{
					indices[candidate * 3],
					indices[candidate * 3 + 1],
					indices[candidate * 3 + 2],
				};

				assert(triIndices[0] < vertexCount);
				assert(triIndices[1] < vertexCount);
				assert(triIndices[2] < vertexCount);

				Vec3 triVerts[3] =
				{
					positions[triIndices[0]].m_position,
					positions[triIndices[1]].m_position,
					positions[triIndices[2]].m_position,
				};

				candidates[i].second = ComputeScore(*curr, psphere, normal, triIndices, triVerts);
			}

			// Determine whether we need to move to the next meshlet.
			if (IsMeshletFull(*curr))
			{
				m_positions.clear();
				m_normals.clear();
				candidateCheck.clear();

				// Use one of our existing candidates as the next meshlet seed.
				if (!candidates.empty())
				{
					candidates[0] = candidates.back();
					candidates.resize(1);
					candidateCheck.insert(candidates[0].first);
				}

				output.emplace_back();
				curr = &output.back();
			}
			else
			{
				std::sort(candidates.begin(), candidates.end(), &CompareTestScores);
			}
		}
		else
		{
			if (candidates.empty())
			{
				m_positions.clear();
				m_normals.clear();
				candidateCheck.clear();

				output.emplace_back();
				curr = &output.back();
			}
		}

		// Ran out of candidates; add a new seed candidate to start the next meshlet.
		if (candidates.empty())
		{
			while (triIndex < triCount && checklist[triIndex])
				++triIndex;

			if (triIndex == triCount)
				break;

			candidates.push_back(std::make_pair(triIndex, 0.0f));
			candidateCheck.insert(triIndex);
		}
	}

	// The last meshlet may have never had any primitives added to it - in which case we want to remove it.
	if (output.back().m_primitiveIndices.empty())
	{
		output.pop_back();
	}
}

void Mesh::ComputeMeshlets()
{
	std::vector<uint32_t> optimizedIndices(m_indices.size(), UINT32_MAX);

	meshopt_optimizeVertexCache(optimizedIndices.data(), m_indices.data(), m_indices.size(), m_meshVertices.size());

	Meshletize(m_inlineMeshlets, optimizedIndices.data(), (uint32_t)optimizedIndices.size(), m_meshVertices, (uint32_t)m_meshVertices.size());

	Rgba8 colors[] = 
	{
		Rgba8::WHITE,
		Rgba8::RED,
		Rgba8::GREEN,
		Rgba8::BLUE,
		Rgba8::CYAN,
		Rgba8::MAGENTA,
		Rgba8::YELLOW
	};

	for (int i = 0; i < m_inlineMeshlets.size(); i++)
	{
		m_inlineMeshlets[i].m_color = colors[i % 7];

		for (int k = 0; k < m_inlineMeshlets[i].m_uniqueVertexIndices.size(); k++)
		{   
			for (int j = 0; j < m_inlineMeshlets[i].m_primitiveIndices.size(); j++)
			{
				if (m_inlineMeshlets[i].m_primitiveIndices[j].m_i0 == m_inlineMeshlets[i].m_uniqueVertexIndices[k])
				{
					m_inlineMeshlets[i].m_primitiveIndices[j].m_i0 = k;
				}

				if (m_inlineMeshlets[i].m_primitiveIndices[j].m_i1 == m_inlineMeshlets[i].m_uniqueVertexIndices[k])
				{
					m_inlineMeshlets[i].m_primitiveIndices[j].m_i1 = k;
				}

				if (m_inlineMeshlets[i].m_primitiveIndices[j].m_i2 == m_inlineMeshlets[i].m_uniqueVertexIndices[k])
				{
					m_inlineMeshlets[i].m_primitiveIndices[j].m_i2 = k;
				}
			}
		}
	}

	m_meshlets.resize(m_inlineMeshlets.size());

	int vertexOffset = 0;
	int primitiveOffset = 0;

	for (int j = 0; j < m_inlineMeshlets.size(); j++)
	{
		Meshlet meshlet;

		for (int i = 0; i < m_inlineMeshlets[j].m_uniqueVertexIndices.size(); i++)
		{
			m_uniqueVertexIndices.push_back(m_inlineMeshlets[j].m_uniqueVertexIndices[i]);
		}

		for (int i = 0; i < m_inlineMeshlets[j].m_primitiveIndices.size(); i++)
		{
			m_primitiveIndices.push_back(m_inlineMeshlets[j].m_primitiveIndices[i]);
		}

		meshlet.m_vertexOffset = vertexOffset;
		meshlet.m_vertexCount = (uint32_t)m_inlineMeshlets[j].m_uniqueVertexIndices.size();

		meshlet.m_primitiveOffset = primitiveOffset;
		meshlet.m_primitiveCount = (uint32_t)m_inlineMeshlets[j].m_primitiveIndices.size();

		float colorAsFloats[4];
		m_inlineMeshlets[j].m_color.GetAsFloats(colorAsFloats);

		meshlet.m_color[0] = colorAsFloats[0];
		meshlet.m_color[1] = colorAsFloats[1];
		meshlet.m_color[2] = colorAsFloats[2];
		meshlet.m_color[3] = colorAsFloats[3];

		m_meshlets[j] = meshlet;

		vertexOffset += (uint32_t)m_inlineMeshlets[j].m_uniqueVertexIndices.size();
		primitiveOffset += (uint32_t)m_inlineMeshlets[j].m_primitiveIndices.size();
	}
}

std::vector<CullData> Mesh::ComputeMeshletCullData()
{
	std::vector<CullData> cullData;
	cullData.resize(m_inlineMeshlets.size());

	std::vector<BoundingSphere> boundSpheres = ComputeBoundSphereData();
	ConeData normalConeData = ComputeNormalConeData();

	for (int i = 0; i < cullData.size(); i++)
	{
		cullData[i].m_boundingSphere = boundSpheres[i];
		cullData[i].m_normalCone[0] = normalConeData[i].first[0];
		cullData[i].m_normalCone[1] = normalConeData[i].first[1];
		cullData[i].m_normalCone[2] = normalConeData[i].first[2];
		cullData[i].m_normalCone[3] = normalConeData[i].first[3];
		cullData[i].m_apexOffset = normalConeData[i].second;
	}

	return cullData;
}

std::vector<BoundingSphere> Mesh::ComputeBoundSphereData()
{
	std::vector<BoundingSphere> bSp;

	for (int i = 0; i < m_inlineMeshlets.size(); i++)
	{
		std::vector<Vec3> positions;

		for (int j = 0; j < m_inlineMeshlets[i].m_uniqueVertexIndices.size(); j++)
		{
			positions.push_back(m_meshVertices[m_inlineMeshlets[i].m_uniqueVertexIndices[j]].m_position);
		}

		Vec3 centroid(0, 0, 0);
		int numVertices = (int)positions.size();

		// Step 1: Compute the centroid (geometric center)
		for (const Vec3& v : positions) {
			centroid.x += v.x;
			centroid.y += v.y;
			centroid.z += v.z;
		}

		centroid.x /= numVertices;
		centroid.y /= numVertices;
		centroid.z /= numVertices;

		// Step 2: Compute the radius (max distance from the centroid to any vertex)
		float maxDistSquared = 0.0f;
		for (const Vec3& v : positions) {
			Vec3 diff = v - centroid;
			float distSquared = diff.GetLengthSquared();  // Avoid using sqrt here for efficiency
			if (distSquared > maxDistSquared) {
				maxDistSquared = distSquared;
			}
		}

		// Step 3: The radius is the square root of the max distance squared
		float radius = sqrtf(maxDistSquared);

		// Return the bounding sphere with the calculated center and radius
		BoundingSphere sph = BoundingSphere(centroid, radius);

		bSp.push_back(sph);
	}

	return bSp;
}
void Mesh::GenerateBoundingBox()
{
	float minX = 0.0f;
	float maxX = 0.0f;
	float minY = 0.0f;
	float maxY = 0.0f;
	float minZ = 0.0f;
	float maxZ = 0.0f;

	for (int i = 0; i < m_meshVertices.size(); i++)
	{
		if (maxX < m_meshVertices[i].m_position.x)
		{
			maxX = m_meshVertices[i].m_position.x;
		}

		if (minX > m_meshVertices[i].m_position.x)
		{
			minX = m_meshVertices[i].m_position.x;
		}

		if (maxY < m_meshVertices[i].m_position.y)
		{
			maxY = m_meshVertices[i].m_position.y;
		}

		if (minY > m_meshVertices[i].m_position.y)
		{
			minY = m_meshVertices[i].m_position.y;
		}

		if (maxZ < m_meshVertices[i].m_position.z)
		{
			maxZ = m_meshVertices[i].m_position.z;
		}

		if (minZ > m_meshVertices[i].m_position.z)
		{
			minZ = m_meshVertices[i].m_position.z;
		}
	}

	m_worldBoundingBox = AABB3(minX, minY, minZ, maxX, maxY, maxZ);
}

std::pair<Vec2, Vec2> Mesh::computeScreenSpaceBoundingBox(const Vec3& minWorld, const Vec3& maxWorld, const Mat44& mvp)
{
	Vec3 corners[8] = {
		Vec3(minWorld.x, minWorld.y, minWorld.z),
		Vec3(maxWorld.x, minWorld.y, minWorld.z),
		Vec3(minWorld.x, maxWorld.y, minWorld.z),
		Vec3(maxWorld.x, maxWorld.y, minWorld.z),
		Vec3(minWorld.x, minWorld.y, maxWorld.z),
		Vec3(maxWorld.x, minWorld.y, maxWorld.z),
		Vec3(minWorld.x, maxWorld.y, maxWorld.z),
		Vec3(maxWorld.x, maxWorld.y, maxWorld.z)
	};

	// Initialize min and max points in screen space
	Vec2 minScreen = WorldToScreenSpace(corners[0], mvp);
	Vec2 maxScreen = minScreen;

	// Transform each corner to screen space and update the min/max bounds
	for (int i = 1; i < 8; ++i) {
		Vec2 screenPos = WorldToScreenSpace(corners[i], mvp);
		
		if (screenPos.x < minScreen.x) {
			minScreen.x = screenPos.x;
		}
		if (screenPos.x > maxScreen.x) {
			maxScreen.x = screenPos.x;
		}

		// Update min/max for y component
		if (screenPos.y < minScreen.y) {
			minScreen.y = screenPos.y;
		}
		if (screenPos.y > maxScreen.y) {
			maxScreen.y = screenPos.y;
		}
	}

	return {minScreen, maxScreen};
}

ConeData Mesh::ComputeNormalConeData()
{
	std::vector<std::pair<uint8_t[4], float>> coneData;
	coneData.resize(m_meshlets.size());
	
	std::vector<Vec3> vertices;
	std::vector<Vec3> normals;
	
	vertices.resize(256);
	normals.resize(256);
	
	for (uint32_t mi = 0; mi < m_meshlets.size(); ++mi)
	{
		auto& m = m_meshlets[mi];
		auto& c = coneData[mi];
	
		// Cache vertices
		for (uint32_t i = 0; i < m.m_vertexCount; ++i)
		{
			uint32_t vIndex = m_uniqueVertexIndices[m.m_vertexOffset + i];
	
			assert(vIndex < m_meshVertices.size());
			vertices[i] = m_meshVertices[vIndex].m_position;
		}
	
		// Generate primitive normals & cache
		for (uint32_t i = 0; i < m.m_primitiveCount; ++i)
		{
			auto primitive = m_primitiveIndices[m.m_primitiveOffset + i];
	
			Vec3 triangle[3]
			{
				vertices[primitive.m_i0],
				vertices[primitive.m_i1],
				vertices[primitive.m_i2],
			};
	
			Vec3 p10 = triangle[1] - triangle[0];
			Vec3 p20 = triangle[2] - triangle[0];
			Vec3 n = (CrossProduct3D(p10, p20)).GetNormalized();
	
			normals[i] = n;
		}
	
		// Calculate spatial bounds
		BoundingSphere positionBounds = ComputeMinimumBoundingSphere(vertices, m.m_vertexCount);
	
		// Calculate the normal cone
		// 1. Normalized center point of minimum bounding sphere of unit normals == conic axis
		BoundingSphere normalBounds = ComputeMinimumBoundingSphere(normals, m.m_primitiveCount);
	
		// 2. Calculate dot product of all normals to conic axis, selecting minimum
	
		Vec4 axis;
		axis.x = normalBounds.m_center.GetNormalized().x;
		axis.y = normalBounds.m_center.GetNormalized().y;
		axis.z = normalBounds.m_center.GetNormalized().z;
		axis.w = 0;
	
		float minDot = 1.0f;
		for (uint32_t i = 0; i < m.m_primitiveCount; ++i)
		{
			float dot = DotProduct3D(Vec3(axis.x, axis.y, axis.z), normals[i]);
			minDot = std::min(minDot, dot);
		}
	
		if (minDot < 0.1f)
		{
			// Degenerate cone
			c.first[0] = 127;
			c.first[1] = 127;
			c.first[2] = 127;
			c.first[3] = 255;
			continue;
		}
	
		// Find the point on center-t*axis ray that lies in negative half-space of all triangles
		float maxt = 0;
	
		for (uint32_t i = 0; i < m.m_primitiveCount; ++i)
		{
			auto primitive = m_primitiveIndices[m.m_primitiveOffset + i];
	
			uint32_t indices[3]
			{
				primitive.m_i0,
				primitive.m_i1,
				primitive.m_i2,
			};
	
			Vec3 triangle[3]
			{
				vertices[indices[0]],
				vertices[indices[1]],
				vertices[indices[2]],
			};
	
			Vec3 cj = positionBounds.m_center - triangle[0];
	
			Vec3 n = (normals[i]);
			float dc = DotProduct3D(cj, n);
			float dn = DotProduct3D(Vec3(axis.x, axis.y, axis.z), n);
	
			// dn should be larger than mindp cutoff above
			assert(dn > 0.0f);
			float t = dc / dn;
	
			maxt = (t > maxt) ? t : maxt;
		}
	
		// cone apex should be in the negative half-space of all cluster triangles by construction
		c.second = maxt;
	
		// cos(a) for normal cone is minDot; we need to add 90 degrees on both sides and invert the cone
		// which gives us -cos(a+90) = -(-sin(a)) = sin(a) = sqrt(1 - cos^2(a))
		float coneCutoffValue = sqrtf(1.0f - minDot * minDot);
		Vec4 coneCutoff = Vec4(coneCutoffValue, coneCutoffValue, coneCutoffValue, coneCutoffValue);
	
		Vec4 quantized = QuantizeSNorm(axis);
		c.first[0] = (uint8_t)quantized.x;
		c.first[1] = (uint8_t)quantized.y;
		c.first[2] = (uint8_t)quantized.z;
	
		Vec4 error = (((quantized / 255.0f) * 2.0f) - Vec4::ONE) - axis;
	
		error.x = fabsf(error.x);
		error.y = fabsf(error.y); 
		error.z = fabsf(error.z);
		error.w = fabsf(error.w);
	
		float sum = error.x + error.y + error.z + error.w;
		error = Vec4(sum, sum, sum, sum);
	
		quantized = QuantizeUNorm(coneCutoff + error);
		quantized.x = std::min(quantized.x + 1.0f, 255.0f);
		quantized.y = std::min(quantized.y + 1.0f, 255.0f);
		quantized.z = std::min(quantized.z + 1.0f, 255.0f);
		quantized.w = std::min(quantized.w + 1.0f, 255.0f);
		c.first[3] = (uint8_t)quantized.x;
	}

	return coneData;
}
void Mesh::ComputeInstanceData(std::vector<Vec3> instancePositions, std::vector<float> instanceScales, bool isRock)
{
	if (!isRock)
	{
		for (int i = 0; i < m_numOfInstances; i++)
		{
			Mat44 transform;
			transform.SetTranslation3D(instancePositions[i]);
			transform.AppendScaleUniform3D(instanceScales[i]);
			transform.AppendXRotation(90.0f);
			transform.AppendYRotation(90.0f);

			MeshletInstance instanceData;
			instanceData.InstanceTransform = transform;
			instanceData.InstanceScale = instanceScales[i];

			m_instanceData.push_back(instanceData);
		}
	}
	else
	{
		RandomNumberGenerator rng = RandomNumberGenerator();

		for (int i = 0; i < m_numOfInstances; i++)
		{

			//float yaw = rng.RollRandomFloatInRange(-180.0f, 180.0f);
			//float pitch = rng.RollRandomFloatInRange(-180.0f, 180.0f);

			Mat44 transform;
			transform.SetTranslation3D(instancePositions[i]);
			transform.AppendScaleUniform3D(instanceScales[i]);
			transform.AppendXRotation(90.0f);
			transform.AppendYRotation(90.0f);

			MeshletInstance instanceData;
			instanceData.InstanceTransform = transform;
			instanceData.InstanceScale = instanceScales[i];

			m_instanceData.push_back(instanceData);
		}
	}
}

//std::vector<CullData> Mesh::ComputeTestMeshletCullData()
//{ 
//	std::vector<CullData> cullData;
//	cullData.resize(m_meshlets.size());
//
//    std::vector<Vec3> vertices;
//    std::vector<Vec3> normals;
//
//	vertices.resize(256);
//	normals.resize(256);
//
//    for (uint32_t mi = 0; mi < m_meshlets.size(); ++mi)
//    {
//        auto& m = m_meshlets[mi];
//        auto& c = cullData[mi];
//
//        // Cache vertices
//        for (uint32_t i = 0; i < m.m_vertexCount; ++i)
//        {
//            uint32_t vIndex = m_uniqueVertexIndices[m.m_vertexOffset + i];
//
//            assert(vIndex < m_meshVertices.size());
//            vertices[i] = m_meshVertices[vIndex].m_position;
//        }
//
//        // Generate primitive normals & cache
//        for (uint32_t i = 0; i < m.m_primitiveCount; ++i)
//        {
//            auto primitive = m_primitiveIndices[m.m_primitiveOffset + i];
//
//            Vec3 triangle[3]
//            {
//                vertices[primitive.m_i0],
//                vertices[primitive.m_i1],
//                vertices[primitive.m_i2],
//            };
//
//            Vec3 p10 = triangle[1] - triangle[0];
//            Vec3 p20 = triangle[2] - triangle[0];
//            Vec3 n = (CrossProduct3D(p10, p20)).GetNormalized();
//
//            normals[i] = n;
//        }
//
//        // Calculate spatial bounds
//        BoundingSphere positionBounds = ComputeMinimumBoundingSphere(vertices, m.m_vertexCount);
//        c.m_boundingSphere = positionBounds;
//
//        // Calculate the normal cone
//        // 1. Normalized center point of minimum bounding sphere of unit normals == conic axis
//        BoundingSphere normalBounds = ComputeMinimumBoundingSphere(normals, m.m_primitiveCount);
//
//        // 2. Calculate dot product of all normals to conic axis, selecting minimum
//
//		Vec4 axis;
//		axis.x = normalBounds.m_center.GetNormalized().x;
//		axis.y = normalBounds.m_center.GetNormalized().y;
//		axis.z = normalBounds.m_center.GetNormalized().z;
//		axis.w = 0;
//
//        float minDot = 1.0f;
//        for (uint32_t i = 0; i < m.m_primitiveCount; ++i)
//        {
//            float dot = DotProduct3D(Vec3(axis.x, axis.y, axis.z), normals[i]);
//            minDot = std::min(minDot, dot);
//        }
//
//        if (minDot < 0.1f)
//        {
//            // Degenerate cone
//            c.m_normalCone[0] = 127;
//            c.m_normalCone[1] = 127;
//            c.m_normalCone[2] = 127;
//            c.m_normalCone[3] = 255;
//            continue;
//        }
//
//        // Find the point on center-t*axis ray that lies in negative half-space of all triangles
//        float maxt = 0;
//
//        for (uint32_t i = 0; i < m.m_primitiveCount; ++i)
//        {
//            auto primitive = m_primitiveIndices[m.m_primitiveOffset + i];
//
//            uint32_t indices[3]
//            {
//                primitive.m_i0,
//                primitive.m_i1,
//                primitive.m_i2,
//            };
//
//            Vec3 triangle[3]
//            {
//                vertices[indices[0]],
//                vertices[indices[1]],
//                vertices[indices[2]],
//            };
//
//            Vec3 cj = positionBounds.m_center - triangle[0];
//
//            Vec3 n = (normals[i]);
//            float dc = DotProduct3D(cj, n);
//            float dn = DotProduct3D(Vec3(axis.x, axis.y, axis.z), n);
//
//            // dn should be larger than mindp cutoff above
//            assert(dn > 0.0f);
//            float t = dc / dn;
//
//            maxt = (t > maxt) ? t : maxt;
//        }
//
//        // cone apex should be in the negative half-space of all cluster triangles by construction
//        c.m_apexOffset = maxt;
//
//        // cos(a) for normal cone is minDot; we need to add 90 degrees on both sides and invert the cone
//        // which gives us -cos(a+90) = -(-sin(a)) = sin(a) = sqrt(1 - cos^2(a))
//		float coneCutoffValue = sqrtf(1.0f - minDot * minDot);
//		Vec4 coneCutoff = Vec4(coneCutoffValue, coneCutoffValue, coneCutoffValue, coneCutoffValue);
//
//		Vec4 quantized = QuantizeSNorm(axis);
//		c.m_normalCone[0] = (uint8_t)quantized.x;
//		c.m_normalCone[1] = (uint8_t)quantized.y;
//		c.m_normalCone[2] = (uint8_t)quantized.z;
//
//		Vec4 error = (((quantized / 255.0f) * 2.0f) - Vec4::ONE) - axis;
//
//		error.x = fabsf(error.x);
//		error.y = fabsf(error.y); 
//		error.z = fabsf(error.z);
//		error.w = fabsf(error.w);
//
//		float sum = error.x + error.y + error.z + error.w;
//		error = Vec4(sum, sum, sum, sum);
//
//		quantized = QuantizeUNorm(coneCutoff + error);
//		quantized.x = std::min(quantized.x + 1.0f, 255.0f);
//		quantized.y = std::min(quantized.y + 1.0f, 255.0f);
//		quantized.z = std::min(quantized.z + 1.0f, 255.0f);
//		quantized.w = std::min(quantized.w + 1.0f, 255.0f);
//		c.m_normalCone[3] = (uint8_t)quantized.x;
//    }
//
//	return cullData;
//}

#endif