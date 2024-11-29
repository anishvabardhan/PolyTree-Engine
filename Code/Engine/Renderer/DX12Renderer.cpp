#include "Engine/Renderer/DX12Renderer.hpp"
#if DX12_RENDERER

#include "Engine/Window/Window.hpp"
#include "Engine/Core/Image.hpp"
#include "Engine/Core/FileUtils.hpp"
#include "Engine/Core/StringUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/Vec4.hpp"
#include "Engine/Math/MathUtils.hpp"
#include "Engine/Renderer/Model.hpp"
#include "Engine/Renderer/Shader.hpp"
#include "Engine/Renderer/Texture.hpp"
#include "Engine/Renderer/BitmapFont.hpp"
#include "Engine/Renderer/MeshBuffer.hpp"
#include "Engine/Renderer/IndexBuffer.hpp"
#include "Engine/Renderer/VertexBuffer.hpp"
#include "Engine/Renderer/ConstantBuffer.hpp"

#include "Game/EngineBuildPreferences.hpp"

#include "ThirdParty/ImGui/imgui.h"
#include "ThirdParty/ImGui/implot.h"
#include "ThirdParty/ImGui/imgui_impl_dx12.h"
#include "ThirdParty/ImGui/imgui_impl_win32.h"

#include "ThirdParty/stb_image/stb_image.h"	

#include "ThirdParty/D3D12Agility/d3dx12/d3dx12_pipeline_state_stream.h"

#include "ThirdParty/Assimp/assimp/scene.h"
#include "ThirdParty/Assimp/assimp/Importer.hpp"
#include "ThirdParty/Assimp/assimp/postprocess.h"

#include "ThirdParty/DXC/dxcapi.h"

#include <fstream>

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

struct Constants
{
	Mat44 CullingViewMatrix;
	Mat44 CullingProjectionMatrix;
};

struct DirectionalLightConstants
{
	Vec3 SunDirection;
	float SunIntensity;
	float AmbientIntensity;
	float pad0 = 0.0f;
	float pad1 = 0.0f;
	float pad2 = 0.0f;
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

struct ProfilerConstants
{
	int m_meshletView = 0;
	int m_frustumCulling = 1;
	int m_backfaceCulling = 1;
	int m_occlusionCulling = 1;
};

static int const k_meshletDebugConstantSlot = 2;
static int const k_directionalLightConstantSlot = 6;
static int const k_cameraConstantSlot = 0;
static int const k_modelConstantSlot = 1;

DX12Renderer::DX12Renderer(RenderConfig const& config)
{
	m_config = config;

	for (int i = 0; i < NUM_FRAME_BUFFERS; i++)
	{
		m_renderTargets[i] = nullptr;
	}
}

DX12Renderer::~DX12Renderer()
{
}

void DX12Renderer::StartUp()
{
	HRESULT hr = D3D12GetDebugInterface(IID_PPV_ARGS(&m_d3dDebugger));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 Debugger!"); 
	}

	m_d3dDebugger->EnableDebugLayer();

	hr = DXGIGetDebugInterface1(0, IID_PPV_ARGS(&m_dxgiDebugger));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create DXGI Debugger!"); 
	}

	hr = CreateDXGIFactory2(DXGI_CREATE_FACTORY_DEBUG, IID_PPV_ARGS(&m_dxgiFactory));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("DXGIFactory failed!!");
	}

	IDXGIFactory6* tempFactory6 = nullptr;

	hr = m_dxgiFactory->QueryInterface(IID_PPV_ARGS(&tempFactory6));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("BRUV");
	}

	hr = tempFactory6->EnumAdapterByGpuPreference(0, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&m_dxgiAdapter));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("BRUV0");
	}

	DX_SAFE_RELEASE(tempFactory6);

	DXGI_ADAPTER_DESC adapterDesc = {};
	m_dxgiAdapter->GetDesc(&adapterDesc);

	hr = D3D12CreateDevice(m_dxgiAdapter, D3D_FEATURE_LEVEL_12_2, IID_PPV_ARGS(&m_device));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 Device!"); 
	}

	m_device->SetName(L"Main virtual device");

	D3D12_COMMAND_QUEUE_DESC cmdQueDesc = {};
	cmdQueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
	cmdQueDesc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_HIGH;
	cmdQueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_DISABLE_GPU_TIMEOUT;
	cmdQueDesc.NodeMask = 0;

	hr = m_device->CreateCommandQueue(&cmdQueDesc, IID_PPV_ARGS(&m_commandQueue));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 cmd queue!"); 
	}

	hr = m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 fence!"); 
	}

	hr = m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&m_commandAllocator));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 cmd allocator!"); 
	}

	hr = m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_commandAllocator, nullptr, IID_PPV_ARGS(&m_commandList));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 cmd list!"); 
	}

	D3D12_DESCRIPTOR_HEAP_DESC rtvDescHeap = {};
	rtvDescHeap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
	rtvDescHeap.NumDescriptors = NUM_FRAME_BUFFERS;
	rtvDescHeap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
	rtvDescHeap.NodeMask = 0;

	hr = m_device->CreateDescriptorHeap(&rtvDescHeap, IID_PPV_ARGS(&m_rtvDescHeap));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 rtv desc heap!"); 
	}

	m_heapIncrement = m_device->GetDescriptorHandleIncrementSize(rtvDescHeap.Type);

	DXGI_SWAP_CHAIN_DESC1 swapChainDesc = {};
	swapChainDesc.Width = g_theWindow->GetClientDimensions().x;
	swapChainDesc.Height = g_theWindow->GetClientDimensions().y;
	swapChainDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	swapChainDesc.Stereo = false; 
	swapChainDesc.SampleDesc = {1, 0};
	swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	swapChainDesc.BufferCount = NUM_FRAME_BUFFERS;
	swapChainDesc.Scaling = DXGI_SCALING_NONE;
	swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
	swapChainDesc.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
	swapChainDesc.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING | DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

	hr = m_dxgiFactory->CreateSwapChainForHwnd(m_commandQueue, (HWND)g_theWindow->GetHwnd(), &swapChainDesc, nullptr, nullptr, &m_swapChain);

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 swap chain!"); 
	}

	for (int i = 0; i < NUM_FRAME_BUFFERS; i++)
	{
		DX_SAFE_RELEASE(m_renderTargets[i]);

		hr = m_swapChain->GetBuffer(i, IID_PPV_ARGS(&m_renderTargets[i]));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Frame buffer doesn't exist!");
		}

		D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle = m_rtvDescHeap->GetCPUDescriptorHandleForHeapStart();
		cpuHandle.ptr += (size_t)m_heapIncrement * i;

		m_device->CreateRenderTargetView(m_renderTargets[i], 0, cpuHandle);
	}

	// MESH SHADER SUPPORT CHECK-----------------------------

	D3D12_FEATURE_DATA_SHADER_MODEL shaderModel = { D3D_SHADER_MODEL_6_5 };
	if (FAILED(m_device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &shaderModel, sizeof(shaderModel)))
		|| (shaderModel.HighestShaderModel < D3D_SHADER_MODEL_6_5))
	{
		ERROR_AND_DIE("ERROR: Shader Model 6.5 is not supported\n");
	}

	D3D12_FEATURE_DATA_D3D12_OPTIONS7 features = {};
	if (FAILED(m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &features, sizeof(features)))
		|| (features.MeshShaderTier == D3D12_MESH_SHADER_TIER_NOT_SUPPORTED))
	{
		ERROR_AND_DIE("Mesh Shaders aren't supported!");
	}

	//-------------------------------------------------------

	// ALL ROOT SIGNATURES INITIALIZE------------------------------------

	CreateAllRootSignatures();

	//-------------------------------------------------------------------

	// DXC INITIALIZE--------------------------------------------------------------

	hr = DxcCreateInstance(CLSID_DxcCompiler, IID_PPV_ARGS(&m_dxcCompiler));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create DXC compiler"); 
	}

	hr = DxcCreateInstance(CLSID_DxcUtils, IID_PPV_ARGS(&m_dxcUtils));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create DXC compiler"); 
	}

	hr = m_dxcUtils->CreateDefaultIncludeHandler(&m_dxcIncludeHandler);

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create DXC compiler"); 
	}

	//-----------------------------------------------------------------------------
	 
	// DEFAULT PIPELINE SET UP -----------------------------------------------------------

	m_pipelineStateDescriptor = new D3D12_GRAPHICS_PIPELINE_STATE_DESC();
	
	m_defaultShader = CreateShader("Default", VertexType::PCUTBN);
	m_diffuseShader = CreateShader("Diffuse", VertexType::PCUTBN);
	
	CreateAllPipelineStates();

	//------------------------------------------------------------------------------------
	
	// Z-PREPASS PIPELINE SET UP -----------------------------------------------------------

	CreateZPrepassPipelineState();

	//------------------------------------------------------------------------------------

	// COMPUTE PIPELINE SET UP -----------------------------------------------------------

	m_testComputeShader = CreateComputeShader(L"TestCS");

	CreateComputeShadingPipelineState();

	//------------------------------------------------------------------------------------


	// MESH SHADER PIPELINE SET UP -----------------------------------------------------------

	m_meshShader = CreateMeshShader(L"MeshletMS", VertexType::PCU);
	m_amplificationShader = CreateAmplificationShader(L"MeshletAS", VertexType::PCU);

	CreateMeshShadingPipelineState();

	//----------------------------------------------------------------------------------------

	Image* image = new Image(IntVec2(2, 2), Rgba8::WHITE);
	m_defaultTexture = CreateTextureFromImage(*image);

	CreateDescriptorHeap(m_ZBuffersDescHeap, 17);

	CreateDepthStencilBuffer();

	D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle = m_ZBuffersDescHeap->GetCPUDescriptorHandleForHeapStart();
	UINT descriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
	cpuHandle.ptr += descriptorSize;

	InitializeHiZBuffers();
	
	m_immediateVBO = CreateVertexBuffer(sizeof(Vertex_PCU));

	m_cameraCBO = CreateConstantBuffer(sizeof(CameraConstants), std::wstring(L"Camera"));
	m_constantsCBO = CreateConstantBuffer(sizeof(Constants), std::wstring(L"Culling"));
	m_modelCBO = CreateConstantBuffer(sizeof(ModelConstants), std::wstring(L"Model"));
	m_meshletDebugCBO = CreateConstantBuffer(sizeof(ProfilerConstants), std::wstring(L"Meshlet Debug"));
	m_directionalLightCBO = CreateConstantBuffer(sizeof(DirectionalLightConstants), std::wstring(L"Directional Light"));

	ImGuiStartUp();
}

void DX12Renderer::BeginFrame()
{
	ImGuiBeginFrame();
}

void DX12Renderer::EndFrame()
{
	ImGuiEndFrame();

	DispatchComputeShader();

	D3D12_RESOURCE_BARRIER rbDesc = {};
	rbDesc.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
	rbDesc.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
	rbDesc.Transition.pResource = m_renderTargets[m_frameIndex];
	rbDesc.Transition.Subresource = 0;
	rbDesc.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
	rbDesc.Transition.StateAfter = D3D12_RESOURCE_STATE_PRESENT;

	m_commandList->ResourceBarrier(1, &rbDesc);
	
	ExecuteCommandList();
	SignalAndWait();
	ResetCommandList();

	m_frameIndex = (m_frameIndex + 1) % NUM_FRAME_BUFFERS;

	m_swapChain->Present(0, 0);
}

void DX12Renderer::ShutDown()
{
	ImGuiShutDown();

	DX_SAFE_RELEASE(m_imGuiSRVDescHeap);

	DELETE_PTR(m_pipelineStateDescriptor);
	
	delete m_immediateVBO;
	m_immediateVBO = nullptr;

	DELETE_PTR(m_cameraCBO);
	DELETE_PTR(m_constantsCBO);
	DELETE_PTR(m_modelCBO);
	DELETE_PTR(m_directionalLightCBO);
	DELETE_PTR(m_meshletDebugCBO)
	DELETE_PTR(m_defaultTexture);

	for (int i = 0; i < m_hiZBuffers.size(); i++)
	{
		DELETE_PTR(m_hiZBuffers[i]);
	}

	DX_SAFE_RELEASE(m_computePSO);
	DX_SAFE_RELEASE(m_ZBuffersDescHeap);

	for (int i = 0; i < (int)m_loadedFonts.size(); i++)
	{
		if (m_loadedFonts[i])
		{
			DELETE_PTR(m_loadedFonts[i]);
		}
	}

	for (int i = 0; i < (int)m_loadedTextures.size(); i++)
	{
		if (m_loadedTextures[i] != nullptr && m_loadedTextures[i]->GetDimensions().x <= INT_MAX && m_loadedTextures[i]->GetDimensions().y <= INT_MAX && m_loadedTextures[i]->GetDimensions().x >= 0 && m_loadedTextures[i]->GetDimensions().y >= 0)
		{
			DELETE_PTR(m_loadedTextures[i]);
		}
	}

	m_loadedTextures.clear();

	for (int i = 0; i < (int)m_loadedShaders.size(); i++)
	{
		if (m_loadedShaders[i])
		{
			DELETE_PTR(m_loadedShaders[i]);
		}
	}

	DX_SAFE_RELEASE(m_dsvDescHeap);
	DX_SAFE_RELEASE(m_depthStencilTexture);

	for (int i = 0; i < TOTAL_ROOT_SIGS; i++)
	{
		DX_SAFE_RELEASE(m_rootSig[i]);
	}

	for (int i = 0; i < TOTAL_PIPELINE_STATES; i++)
	{
		DX_SAFE_RELEASE(m_PSO[i]);
	}

	DX_SAFE_RELEASE(m_zPrepassPSO);
	DX_SAFE_RELEASE(m_meshPSO);

	for (int i = 0; i < NUM_FRAME_BUFFERS; i++)
	{
		m_commandQueue->Signal(m_fence, m_fenceValue + i);
	}

	DX_SAFE_RELEASE(m_fence);
	DX_SAFE_RELEASE(m_commandQueue);
	DX_SAFE_RELEASE(m_commandList);
	DX_SAFE_RELEASE(m_commandAllocator);

	for (int i = 0; i < NUM_FRAME_BUFFERS; i++)
	{
		DX_SAFE_RELEASE(m_renderTargets[i]);
	};

	DX_SAFE_RELEASE(m_rtvDescHeap);
	DX_SAFE_RELEASE(m_swapChain);
	DX_SAFE_RELEASE(m_device);
	DX_SAFE_RELEASE(m_dxgiAdapter);
	DX_SAFE_RELEASE(m_dxgiFactory);
	
	m_dxgiDebugger->ReportLiveObjects(DXGI_DEBUG_ALL, (DXGI_DEBUG_RLO_FLAGS)(DXGI_DEBUG_RLO_DETAIL | DXGI_DEBUG_RLO_IGNORE_INTERNAL));
	DX_SAFE_RELEASE(m_dxgiDebugger);
	DX_SAFE_RELEASE(m_d3dDebugger);
}

void DX12Renderer::ImGuiStartUp()
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      

	D3D12_DESCRIPTOR_HEAP_DESC imGuiSRVDescHeap = {};
	imGuiSRVDescHeap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	imGuiSRVDescHeap.NumDescriptors = NUM_FRAME_BUFFERS;
	imGuiSRVDescHeap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	imGuiSRVDescHeap.NodeMask = 0;

	HRESULT hr = m_device->CreateDescriptorHeap(&imGuiSRVDescHeap, IID_PPV_ARGS(&m_imGuiSRVDescHeap));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 ImGui srv desc heap!"); 
	}

	ImGui_ImplWin32_Init(g_theWindow->GetHwnd());
	ImGui_ImplDX12_Init(m_device, NUM_FRAME_BUFFERS, DXGI_FORMAT_R8G8B8A8_UNORM, m_imGuiSRVDescHeap, m_imGuiSRVDescHeap->GetCPUDescriptorHandleForHeapStart(), m_imGuiSRVDescHeap->GetGPUDescriptorHandleForHeapStart());
}

void DX12Renderer::ImGuiBeginFrame()
{
	ImGui_ImplDX12_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();
}

void DX12Renderer::ImGuiEndFrame()
{
	m_commandList->SetDescriptorHeaps(1, &m_imGuiSRVDescHeap);
	ImGui::Render();
	ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), m_commandList);
}

void DX12Renderer::ImGuiShutDown()
{
	ImGui_ImplDX12_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImPlot::DestroyContext();
	ImGui::DestroyContext();
}

void DX12Renderer::ResetCommandList()
{
	m_commandAllocator->Reset();
	m_commandList->Reset(m_commandAllocator, nullptr);
}

void DX12Renderer::ExecuteCommandList()
{
	if (SUCCEEDED(m_commandList->Close()))
	{
		ID3D12CommandList* list[] = {m_commandList};
		m_commandQueue->ExecuteCommandLists(1, list);
	}
}

void DX12Renderer::SignalAndWait()
{
	m_commandQueue->Signal(m_fence, ++m_fenceValue);
	
	while (m_fence->GetCompletedValue() < m_fenceValue)
	{
		_mm_pause();
	}
}

Model* DX12Renderer::LoadModel(char const* filePath, RootSig pipelineMode)
{
	Assimp::Importer importer;

	aiScene const* scene = importer.ReadFile(filePath,
		 aiProcess_JoinIdenticalVertices
		| aiProcess_Triangulate | aiProcess_ConvertToLeftHanded);

	if (pipelineMode == DEFAULT_PIPELINE)
	{
		std::vector<Vertex_PCU> verts;
		std::vector<unsigned int> indices;

		ProcessNode(scene->mRootNode, scene, verts, indices);

		Model* model = new Model(verts, indices);

		return model;
	}
	else if (pipelineMode == MESH_SHADER_PIPELINE)
	{
		std::vector<MeshVertex_PCU> verts;
		std::vector<unsigned int> indices;

		ProcessNode(scene->mRootNode, scene, verts, indices);

		Model* model = /*new Model(verts, indices)*/nullptr;

		return model;
	}

	return nullptr;
}

void DX12Renderer::ProcessNode(aiNode* node, const aiScene* scene, std::vector<Vertex_PCU>& verts, std::vector<unsigned int>& indices)
{
	for(unsigned int i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		ProcessMesh(mesh, verts, indices);
	}

	for(unsigned int i = 0; i < node->mNumChildren; i++)
	{
		ProcessNode(node->mChildren[i], scene, verts, indices);
	}
}

void DX12Renderer::ProcessNode(aiNode* node, const aiScene* scene, std::vector<MeshVertex_PCU>& verts, std::vector<unsigned int>& indices)
{
	for(unsigned int i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		ProcessMesh(mesh, verts, indices);
	}

	for(unsigned int i = 0; i < node->mNumChildren; i++)
	{
		ProcessNode(node->mChildren[i], scene, verts, indices);
	}
}

void DX12Renderer::ProcessMesh(aiMesh* mesh, std::vector<Vertex_PCU>& verts, std::vector<unsigned int>& indices)
{
	Rgba8 colors[7] = 
	{
		Rgba8::RED,
		Rgba8::GREEN,
		Rgba8::BLUE,
		Rgba8::CYAN,
		Rgba8::MAGENTA,
		Rgba8::YELLOW,
		Rgba8::WHITE
	};

	for(unsigned int i = 0; i < mesh->mNumVertices; i++)
	{
		Vertex_PCU vertex;
		Vec3 vector;
	
		vector.x = mesh->mVertices[i].x;
		vector.y = mesh->mVertices[i].y;
		vector.z = mesh->mVertices[i].z;
		
		vertex.m_position = vector;
		vertex.m_color = colors[i % 7];
		vertex.m_uvTexCoords = Vec2::ZERO;
	
		verts.push_back(vertex);
	}

	for(unsigned int i = 0; i < mesh->mNumFaces; i++)
	{
		aiFace face = mesh->mFaces[i];

		for(unsigned int j = 0; j < face.mNumIndices; j++)
			indices.push_back(face.mIndices[j]);        
	}
}

void DX12Renderer::ProcessMesh(aiMesh* mesh, std::vector<MeshVertex_PCU>& verts, std::vector<unsigned int>& indices)
{
	Vec4 colors[7] = 
	{
		Vec4(0.0f, 0.0f, 1.0f, 1.0f),
		Vec4(0.0f, 1.0f, 0.0f, 1.0f),
		Vec4(0.0f, 1.0f, 1.0f, 1.0f),
		Vec4(1.0f, 0.0f, 0.0f, 1.0f),
		Vec4(1.0f, 0.0f, 1.0f, 1.0f),
		Vec4(1.0f, 1.0f, 0.0f, 1.0f),
		Vec4(1.0f, 1.0f, 1.0f, 1.0f)
	};

	for(unsigned int i = 0; i < mesh->mNumVertices; i++)
	{
		MeshVertex_PCU vertex;
		Vec3 vector;

		vector.x = mesh->mVertices[i].x;
		vector.y = mesh->mVertices[i].y;
		vector.z = mesh->mVertices[i].z;

		vertex.m_position = vector;
		vertex.m_color = colors[i % 7];
		vertex.m_uv = Vec2::ZERO;

		verts.push_back(vertex);
	}

	for(unsigned int i = 0; i < mesh->mNumFaces; i++)
	{
		aiFace face = mesh->mFaces[i];

		for(unsigned int j = 0; j < face.mNumIndices; j++)
			indices.push_back(face.mIndices[j]);        
	}
}

BitmapFont* DX12Renderer::CreateOrGetBitmapFont(const char* bitmapFontFilePathWithNoExtension)
{
	BitmapFont* font = GetFontForFileName(bitmapFontFilePathWithNoExtension);
	if (font)
	{
		return font;
	}

	BitmapFont* newFont = CreateBitmapFont(bitmapFontFilePathWithNoExtension);
	return newFont;
}

BitmapFont* DX12Renderer::CreateBitmapFont(const char* bitmapFontFilePathWithNoExtension)
{
	Texture* fontTexture = CreateOrGetTextureFromFile(bitmapFontFilePathWithNoExtension);

	BitmapFont* font = new BitmapFont(bitmapFontFilePathWithNoExtension, *fontTexture);

	m_loadedFonts.push_back(font);

	return font;
}

BitmapFont* DX12Renderer::GetFontForFileName(char const* imageFilePath)
{
	UNUSED(imageFilePath);

	return nullptr;
}

Texture* DX12Renderer::CreateOrGetTextureFromFile(char const* imageFilePath)
{
	Texture* existingTexture = GetTextureForFileName(imageFilePath);
	if (existingTexture)
	{
		return existingTexture;
	}

	Texture* newTexture = CreateTextureFromFile(imageFilePath);
	return newTexture;
}

Texture* DX12Renderer::CreateTextureFromFile(char const* imageFilePath)
{
	Image* image = new Image(imageFilePath);

	Texture* newTexture = CreateTextureFromImage(*image);

	m_loadedTextures.push_back(newTexture);
	return newTexture;
}

Texture* DX12Renderer::CreateTextureFromImage(Image const& image)
{
	Texture* texture = new Texture();
	texture->m_dimensions = image.GetDimensions();
	texture->m_name = image.GetImageFilePath();

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
	uploadResourceDesc.Width = image.GetDimensions().x * image.GetDimensions().y * sizeof(unsigned char) * 4;
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
	defaultResourceDesc.Width = image.GetDimensions().x;
	defaultResourceDesc.Height = image.GetDimensions().y;
	defaultResourceDesc.DepthOrArraySize = 1;
	defaultResourceDesc.MipLevels = 1;
	defaultResourceDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	defaultResourceDesc.SampleDesc = {1, 0};
	defaultResourceDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
	defaultResourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	HRESULT hr = m_device->CreateCommittedResource(&hpUpload, D3D12_HEAP_FLAG_NONE, &uploadResourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&texture->m_uploadBuffer));
	hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &defaultResourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&texture->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 texture!"); 
	}

	texture->m_uploadBuffer->SetName(L"Texture Upload Buffer");
	texture->m_defaultBuffer->SetName(L"Texture Default Buffer");

	D3D12_DESCRIPTOR_HEAP_DESC srvDescHeap = {};
	srvDescHeap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	srvDescHeap.NumDescriptors = 1;
	srvDescHeap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	srvDescHeap.NodeMask = 0;

	hr = m_device->CreateDescriptorHeap(&srvDescHeap, IID_PPV_ARGS(&texture->m_shaderResourceView));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 srv desc heap!"); 
	}

	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
	srvDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
	srvDesc.Texture2D.MipLevels = 1;
	srvDesc.Texture2D.MostDetailedMip = 0;
	srvDesc.Texture2D.PlaneSlice = 0;
	srvDesc.Texture2D.ResourceMinLODClamp = 0.0f;

	m_device->CreateShaderResourceView(texture->m_defaultBuffer, &srvDesc, texture->m_shaderResourceView->GetCPUDescriptorHandleForHeapStart());

	void* dest = nullptr;

	D3D12_RANGE uploadRange;
	uploadRange.Begin = 0;
	uploadRange.End = image.GetDimensions().x * image.GetDimensions().y * sizeof(unsigned char);
	texture->m_uploadBuffer->Map(0, &uploadRange, &dest);
	memcpy(dest, image.GetRawData(), image.GetDimensions().x * image.GetDimensions().y * sizeof(unsigned char) * 4);
	texture->m_uploadBuffer->Unmap(0, &uploadRange);

	D3D12_BOX textureSizeAsBox;
	textureSizeAsBox.left = textureSizeAsBox.top = textureSizeAsBox.front = 0;
	textureSizeAsBox.right = image.GetDimensions().x;
	textureSizeAsBox.bottom = image.GetDimensions().y;
	textureSizeAsBox.back = 1;
	D3D12_TEXTURE_COPY_LOCATION txtcSrc, txtcDst;
	txtcSrc.pResource = texture->m_uploadBuffer;
	txtcSrc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
	txtcSrc.PlacedFootprint.Offset = 0;
	txtcSrc.PlacedFootprint.Footprint.Width = image.GetDimensions().x;
	txtcSrc.PlacedFootprint.Footprint.Height = image.GetDimensions().y;
	txtcSrc.PlacedFootprint.Footprint.Depth = 1;
	txtcSrc.PlacedFootprint.Footprint.RowPitch = sizeof(unsigned char) * 4 * image.GetDimensions().x;
	txtcSrc.PlacedFootprint.Footprint.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	txtcDst.pResource = texture->m_defaultBuffer;
	txtcDst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
	txtcDst.SubresourceIndex = 0;
	m_commandList->CopyTextureRegion(&txtcDst, 0, 0, 0, &txtcSrc, &textureSizeAsBox);

	ExecuteCommandList();
	SignalAndWait();
	ResetCommandList();

	return texture;
}

Texture* DX12Renderer::CreateModifiableTexture(IntVec2 dimensions)
{
	UNUSED(dimensions);

	return nullptr;
}

Texture* DX12Renderer::CreateTextureFromData(char const* name, IntVec2 dimensions, int bytesPerTexel, uint8_t* texelData)
{
	UNUSED(name);
	UNUSED(dimensions);
	UNUSED(bytesPerTexel);
	UNUSED(texelData);

	return nullptr;
}

Texture* DX12Renderer::GetTextureForFileName(char const* imageFilePath)
{
	UNUSED(imageFilePath);

	return nullptr;
}

void DX12Renderer::BindTexture(const Texture* texture, RootSig pipelineMode)
{
	if (pipelineMode == RootSig::DEFAULT_PIPELINE)
	{

		m_commandList->SetGraphicsRootSignature(m_rootSig[RootSig::DEFAULT_PIPELINE]);

		if (texture == nullptr)
		{
			m_commandList->SetDescriptorHeaps(1, &m_defaultTexture->m_shaderResourceView);
			m_commandList->SetGraphicsRootDescriptorTable(2, m_defaultTexture->m_shaderResourceView->GetGPUDescriptorHandleForHeapStart());
		}
		else
		{
			m_commandList->SetDescriptorHeaps(1, &texture->m_shaderResourceView);
			m_commandList->SetGraphicsRootDescriptorTable(2, texture->m_shaderResourceView->GetGPUDescriptorHandleForHeapStart());
		}
	}
	else if (pipelineMode == RootSig::DIFFUSE_PIPELINE)
	{

		m_commandList->SetGraphicsRootSignature(m_rootSig[RootSig::DIFFUSE_PIPELINE]);

		if (texture == nullptr)
		{
			m_commandList->SetDescriptorHeaps(1, &m_defaultTexture->m_shaderResourceView);
			m_commandList->SetGraphicsRootDescriptorTable(3, m_defaultTexture->m_shaderResourceView->GetGPUDescriptorHandleForHeapStart());
		}
		else
		{
			m_commandList->SetDescriptorHeaps(1, &texture->m_shaderResourceView);
			m_commandList->SetGraphicsRootDescriptorTable(3, texture->m_shaderResourceView->GetGPUDescriptorHandleForHeapStart());
		}
	}
	else if (pipelineMode == RootSig::MESH_SHADER_PIPELINE)
	{
		m_commandList->SetGraphicsRootSignature(m_rootSig[RootSig::MESH_SHADER_PIPELINE]);

		if (texture == nullptr)
		{
			m_commandList->SetDescriptorHeaps(1, &m_defaultTexture->m_shaderResourceView);
			m_commandList->SetGraphicsRootDescriptorTable(4, m_defaultTexture->m_shaderResourceView->GetGPUDescriptorHandleForHeapStart());
		}
		else
		{
			m_commandList->SetDescriptorHeaps(1, &texture->m_shaderResourceView);
			m_commandList->SetGraphicsRootDescriptorTable(4, texture->m_shaderResourceView->GetGPUDescriptorHandleForHeapStart());
		}
	}
}

void DX12Renderer::InitializeHiZBuffers()
{
	D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle = m_ZBuffersDescHeap->GetCPUDescriptorHandleForHeapStart();
	UINT descriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
	cpuHandle.ptr += descriptorSize;

	IntVec2 fullResDims = g_theWindow->GetClientDimensions();

	int currentDimsHeight = fullResDims.y / 2;

	while (currentDimsHeight > 1)
	{
		int currentDimsWidth = currentDimsHeight * g_theWindow->GetAspect();

		Image* currentImage = new Image(IntVec2(currentDimsWidth, currentDimsHeight), Rgba8::WHITE);
		Texture* currentZBuffer = CreateZBufferTexture(*currentImage);

		CreateUnorderedAccessView(currentZBuffer->m_defaultBuffer, &cpuHandle, DXGI_FORMAT_R32_FLOAT, D3D12_UAV_DIMENSION_TEXTURE2D);
		cpuHandle.ptr += descriptorSize;

		CreateShaderResourceView(currentZBuffer->m_defaultBuffer, &cpuHandle, DXGI_FORMAT_R32_FLOAT, D3D12_SRV_DIMENSION_TEXTURE2D, sizeof(float), currentDimsWidth * currentDimsHeight);
		cpuHandle.ptr += descriptorSize;

		m_hiZBuffers.push_back(currentZBuffer);

		currentDimsHeight /= 2;
	}
}

void DX12Renderer::CreateDescriptorHeap(ID3D12DescriptorHeap*& outDescHeap, int numOfDescriptors)
{
	D3D12_DESCRIPTOR_HEAP_DESC srvDescHeap = {};
	srvDescHeap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	srvDescHeap.NumDescriptors = numOfDescriptors;
	srvDescHeap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	srvDescHeap.NodeMask = 0;

	HRESULT hr = m_device->CreateDescriptorHeap(&srvDescHeap, IID_PPV_ARGS(&outDescHeap));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 desc heap!"); 
	}
}

void DX12Renderer::CreateShaderResourceView(ID3D12Resource* defaultBuffer, ID3D12DescriptorHeap*& outDescHeap, DXGI_FORMAT srvFormat, D3D12_SRV_DIMENSION srvDims, UINT byteStride, UINT numElements)
{
	D3D12_DESCRIPTOR_HEAP_DESC srvDescHeap = {};
	srvDescHeap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	srvDescHeap.NumDescriptors = 1;
	srvDescHeap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	srvDescHeap.NodeMask = 0;

	HRESULT hr = m_device->CreateDescriptorHeap(&srvDescHeap, IID_PPV_ARGS(&outDescHeap));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 srv desc heap!"); 
	}

	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
	srvDesc.Format = srvFormat;
	srvDesc.ViewDimension = srvDims;
	srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
	srvDesc.Buffer.StructureByteStride = byteStride;
	srvDesc.Buffer.NumElements = numElements;
	srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;

	m_device->CreateShaderResourceView(defaultBuffer, &srvDesc, outDescHeap->GetCPUDescriptorHandleForHeapStart());
}

void DX12Renderer::CreateShaderResourceView(ID3D12Resource* defaultBuffer, D3D12_CPU_DESCRIPTOR_HANDLE* existingCPUDescHandle, DXGI_FORMAT srvFormat, D3D12_SRV_DIMENSION srvDims, UINT byteStride, UINT numElements)
{
	if(existingCPUDescHandle)
	{
		D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
		srvDesc.Format = srvFormat;
		srvDesc.ViewDimension = srvDims;
		srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
		
		if (srvDims == D3D12_SRV_DIMENSION_BUFFER)
		{
			srvDesc.Buffer.StructureByteStride = byteStride;
			srvDesc.Buffer.NumElements = numElements;
			srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;
		}
		else if (srvDims == D3D12_SRV_DIMENSION_TEXTURE2D)
		{
			srvDesc.Texture2D.MipLevels = 1;
			srvDesc.Texture2D.PlaneSlice = 0;
		}


		m_device->CreateShaderResourceView(defaultBuffer, &srvDesc, *existingCPUDescHandle);
	}
}

void DX12Renderer::CreateUnorderedAccessView(ID3D12Resource* defaultBuffer, ID3D12DescriptorHeap*& outDescHeap, DXGI_FORMAT uavFormat, D3D12_UAV_DIMENSION uavDims)
{
	D3D12_DESCRIPTOR_HEAP_DESC uavDescHeap = {};
	uavDescHeap.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	uavDescHeap.NumDescriptors = 1;
	uavDescHeap.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	uavDescHeap.NodeMask = 0;

	HRESULT hr = m_device->CreateDescriptorHeap(&uavDescHeap, IID_PPV_ARGS(&outDescHeap));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 uav desc heap!"); 
	}

	D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
	uavDesc.ViewDimension = uavDims;
	uavDesc.Format = uavFormat; 
	uavDesc.Texture2D.MipSlice = 0;
	uavDesc.Texture2D.PlaneSlice = 0;

	outDescHeap->SetName(L"UAV DescHeap");

	m_device->CreateUnorderedAccessView(defaultBuffer, nullptr, &uavDesc, outDescHeap->GetCPUDescriptorHandleForHeapStart());
}

void DX12Renderer::CreateUnorderedAccessView(ID3D12Resource* defaultBuffer, D3D12_CPU_DESCRIPTOR_HANDLE* existingCPUDescHandle, DXGI_FORMAT uavFormat, D3D12_UAV_DIMENSION uavDims, UINT byteStride, UINT numElements)
{
	if (existingCPUDescHandle)
	{
		D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
		uavDesc.ViewDimension = uavDims;
		uavDesc.Format = uavFormat; 

		if (uavDims == D3D12_UAV_DIMENSION_BUFFER)
		{
			uavDesc.Buffer.StructureByteStride = byteStride;
			uavDesc.Buffer.NumElements = numElements;
			uavDesc.Buffer.Flags = D3D12_BUFFER_UAV_FLAG_NONE;
		}
		else if (uavDims == D3D12_UAV_DIMENSION_TEXTURE2D)
		{
			uavDesc.Texture2D.MipSlice = 0;
			uavDesc.Texture2D.PlaneSlice = 0;
		}

		m_device->CreateUnorderedAccessView(defaultBuffer, nullptr, &uavDesc, *existingCPUDescHandle);
	}
}

void DX12Renderer::CreateDepthStencilBuffer()
{
	D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
	dsvHeapDesc.NumDescriptors = 1;
	dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
	dsvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;

	HRESULT hr = m_device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&m_dsvDescHeap));
	
	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 dsv desc heap!"); 
	}

	D3D12_DEPTH_STENCIL_VIEW_DESC depthStencilDesc = {};
	depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	depthStencilDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
	depthStencilDesc.Flags = D3D12_DSV_FLAG_NONE;

	D3D12_CLEAR_VALUE depthOptimizedClearValue = {};
	depthOptimizedClearValue.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	depthOptimizedClearValue.DepthStencil.Depth = 1.0f;
	depthOptimizedClearValue.DepthStencil.Stencil = 0;

	D3D12_HEAP_PROPERTIES hpDefault{};
	hpDefault.Type = D3D12_HEAP_TYPE_DEFAULT;
	hpDefault.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpDefault.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpDefault.CreationNodeMask = 0;
	hpDefault.VisibleNodeMask = 0;

	D3D12_RESOURCE_DESC defaultResourceDesc = {};
	defaultResourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
	defaultResourceDesc.Alignment = 0;
	defaultResourceDesc.Width = g_theWindow->GetClientDimensions().x;
	defaultResourceDesc.Height = g_theWindow->GetClientDimensions().y;
	defaultResourceDesc.DepthOrArraySize = 1;
	defaultResourceDesc.MipLevels = 1;
	defaultResourceDesc.Format = DXGI_FORMAT_R24G8_TYPELESS;
	defaultResourceDesc.SampleDesc = {1, 0};
	defaultResourceDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
	defaultResourceDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

	hr = m_device->CreateCommittedResource( &hpDefault, D3D12_HEAP_FLAG_NONE, &defaultResourceDesc, D3D12_RESOURCE_STATE_DEPTH_WRITE, &depthOptimizedClearValue, IID_PPV_ARGS(&m_depthStencilTexture) );

	m_dsvDescHeap->SetName(L"Depth/Stencil Resource Heap");

	m_device->CreateDepthStencilView(m_depthStencilTexture, &depthStencilDesc, m_dsvDescHeap->GetCPUDescriptorHandleForHeapStart());
	
	D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle = m_ZBuffersDescHeap->GetCPUDescriptorHandleForHeapStart();

	CreateShaderResourceView(m_depthStencilTexture, &cpuHandle, DXGI_FORMAT_R24_UNORM_X8_TYPELESS, D3D12_SRV_DIMENSION_TEXTURE2D, sizeof(float), g_theWindow->GetClientDimensions().x * g_theWindow->GetClientDimensions().y);
}

VertexBuffer* DX12Renderer::CreateVertexBuffer(size_t const size, std::wstring bufferDebugName)
{
#if DX12_RENDERER

	VertexBuffer* buffer = new VertexBuffer(size);

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

	D3D12_RESOURCE_DESC resourceDesc = {};
	resourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	resourceDesc.Alignment = 0;
	resourceDesc.Width = size;
	resourceDesc.Height = 1;
	resourceDesc.DepthOrArraySize = 1;
	resourceDesc.MipLevels = 1;
	resourceDesc.Format = DXGI_FORMAT_UNKNOWN;
	resourceDesc.SampleDesc = {1, 0};
	resourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	resourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	HRESULT hr = m_device->CreateCommittedResource(&hpUpload, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_uploadBuffer));
	hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 vbo!"); 
	}

	std::wstring uploadBufferName = bufferDebugName + std::wstring(L" Upload Buffer");
	std::wstring defaultBufferName = bufferDebugName + std::wstring(L" Default Buffer");

	buffer->m_uploadBuffer->SetName(uploadBufferName.c_str());
	buffer->m_defaultBuffer->SetName(defaultBufferName.c_str());

	return buffer;
#endif
}

void DX12Renderer::CopyCPUToGPU(void const* data, size_t size, VertexBuffer*& vbo)
{
#if DX12_RENDERER

	if (vbo->m_size < size)
	{
		DELETE_PTR(vbo);
		vbo = CreateVertexBuffer(size, std::wstring(L"Test"));
	}

	void* dest = nullptr;

	D3D12_RANGE uploadRange;
	uploadRange.Begin = 0;
	uploadRange.End = size;
	vbo->m_uploadBuffer->Map(0, &uploadRange, &dest);
	memcpy(dest, data, size);
	vbo->m_uploadBuffer->Unmap(0, &uploadRange);

	m_commandList->CopyBufferRegion(vbo->m_defaultBuffer, 0, vbo->m_uploadBuffer, 0, size);

	ExecuteCommandList();
	SignalAndWait();
	ResetCommandList();
#endif
}

void DX12Renderer::BindVertexBuffer(VertexBuffer* buffer, int stride)
{
#if DX12_RENDERER

	D3D12_VERTEX_BUFFER_VIEW vertexBufferView;
	vertexBufferView.BufferLocation = buffer->m_defaultBuffer->GetGPUVirtualAddress();
	vertexBufferView.StrideInBytes = stride;
	vertexBufferView.SizeInBytes = (UINT)buffer->m_size;

	m_commandList->IASetVertexBuffers(0, 1, &vertexBufferView);

#endif
}

IndexBuffer* DX12Renderer::CreateIndexBuffer(size_t const size, std::wstring bufferDebugName)
{
#if DX12_RENDERER

	IndexBuffer* buffer = new IndexBuffer(size);

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

	D3D12_RESOURCE_DESC resourceDesc = {};
	resourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	resourceDesc.Alignment = 0;
	resourceDesc.Width = size;
	resourceDesc.Height = 1;
	resourceDesc.DepthOrArraySize = 1;
	resourceDesc.MipLevels = 1;
	resourceDesc.Format = DXGI_FORMAT_UNKNOWN;
	resourceDesc.SampleDesc = {1, 0};
	resourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	resourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	HRESULT hr = m_device->CreateCommittedResource(&hpUpload, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_uploadBuffer));
	hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 vbo!"); 
	}

	std::wstring uploadBufferName = bufferDebugName + std::wstring(L" Upload Buffer");
	std::wstring defaultBufferName = bufferDebugName + std::wstring(L" Default Buffer");

	buffer->m_uploadBuffer->SetName(uploadBufferName.c_str());
	buffer->m_defaultBuffer->SetName(defaultBufferName.c_str());

	return buffer;
#endif
}

void DX12Renderer::CopyCPUToGPU(void const* data, size_t size, IndexBuffer*& ibo)
{
#if DX12_RENDERER

	if (ibo->m_size < size)
	{
		DELETE_PTR(ibo);
		ibo = CreateIndexBuffer(size, std::wstring(L"Test"));
	}

	void* dest = nullptr;

	D3D12_RANGE uploadRange;
	uploadRange.Begin = 0;
	uploadRange.End = size;
	ibo->m_uploadBuffer->Map(0, &uploadRange, &dest);
	memcpy(dest, data, size);
	ibo->m_uploadBuffer->Unmap(0, &uploadRange);

	m_commandList->CopyBufferRegion(ibo->m_defaultBuffer, 0, ibo->m_uploadBuffer, 0, size);

	ExecuteCommandList();
	SignalAndWait();
	ResetCommandList();
#endif
}

void DX12Renderer::BindIndexBuffer(IndexBuffer* buffer)
{
#if DX12_RENDERER

	D3D12_INDEX_BUFFER_VIEW indexBufferView;
	indexBufferView.BufferLocation = buffer->m_defaultBuffer->GetGPUVirtualAddress();
	indexBufferView.Format = DXGI_FORMAT_R32_UINT;
	indexBufferView.SizeInBytes = (UINT)buffer->m_size;

	m_commandList->IASetIndexBuffer(&indexBufferView);
#endif
}

MeshBuffer* DX12Renderer::CreateMeshBuffer(size_t const size, std::wstring bufferDebugName)
{
#if DX12_RENDERER

	MeshBuffer* buffer = new MeshBuffer(size);

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

	D3D12_RESOURCE_DESC resourceDesc = {};
	resourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	resourceDesc.Alignment = 0;
	resourceDesc.Width = size;
	resourceDesc.Height = 1;
	resourceDesc.DepthOrArraySize = 1;
	resourceDesc.MipLevels = 1;
	resourceDesc.Format = DXGI_FORMAT_UNKNOWN;
	resourceDesc.SampleDesc = {1, 0};
	resourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	resourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	HRESULT hr = m_device->CreateCommittedResource(&hpUpload, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_uploadBuffer));
	hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 mbo!"); 
	}

	std::wstring uploadBufferName = bufferDebugName + std::wstring(L" Upload Buffer");
	std::wstring defaultBufferName = bufferDebugName + std::wstring(L" Default Buffer");

	buffer->m_uploadBuffer->SetName(uploadBufferName.c_str());
	buffer->m_defaultBuffer->SetName(defaultBufferName.c_str());

	return buffer;
#endif
	return nullptr;
}

MeshBuffer* DX12Renderer::CreateRWMeshBuffer(size_t const size, std::wstring bufferDebugName)
{
	MeshBuffer* buffer = new MeshBuffer(size);

	D3D12_HEAP_PROPERTIES hpUpload{};
	hpUpload.Type = D3D12_HEAP_TYPE_UPLOAD;
	hpUpload.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpUpload.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpUpload.CreationNodeMask = 1;
	hpUpload.VisibleNodeMask = 1;

	D3D12_RESOURCE_DESC uploadBufferDesc = {};
	uploadBufferDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	uploadBufferDesc.Alignment = 0;
	uploadBufferDesc.Width = size;
	uploadBufferDesc.Height = 1;
	uploadBufferDesc.DepthOrArraySize = 1;
	uploadBufferDesc.MipLevels = 1;
	uploadBufferDesc.Format = DXGI_FORMAT_UNKNOWN;
	uploadBufferDesc.SampleDesc.Count = 1;
	uploadBufferDesc.SampleDesc.Quality = 0;
	uploadBufferDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	// No flags for upload heap
	uploadBufferDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	D3D12_HEAP_PROPERTIES hpDefault{};
	hpDefault.Type = D3D12_HEAP_TYPE_DEFAULT;
	hpDefault.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpDefault.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpDefault.CreationNodeMask = 0;
	hpDefault.VisibleNodeMask = 0;

	D3D12_RESOURCE_DESC resourceDesc = {};
	resourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	resourceDesc.Alignment = 0;
	resourceDesc.Width = size;
	resourceDesc.Height = 1;
	resourceDesc.DepthOrArraySize = 1;
	resourceDesc.MipLevels = 1;
	resourceDesc.Format = DXGI_FORMAT_UNKNOWN;
	resourceDesc.SampleDesc = {1, 0};
	resourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	resourceDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

	HRESULT hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_defaultBuffer));
	hr = m_device->CreateCommittedResource(&hpUpload, D3D12_HEAP_FLAG_NONE, &uploadBufferDesc, D3D12_RESOURCE_STATE_GENERIC_READ, 0, IID_PPV_ARGS(&buffer->m_uploadBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 mbo!"); 
	}

	std::wstring uploadBufferName = bufferDebugName + std::wstring(L" Upload Buffer");
	std::wstring defaultBufferName = bufferDebugName + std::wstring(L" Default Buffer");

	buffer->m_uploadBuffer->SetName(uploadBufferName.c_str());
	buffer->m_defaultBuffer->SetName(defaultBufferName.c_str());

	return buffer;
}

void DX12Renderer::CopyCPUToGPU(void const* data, size_t size, MeshBuffer*& mbo)
{
#if DX12_RENDERER

	void* dest = nullptr;

	D3D12_RANGE uploadRange;
	uploadRange.Begin = 0;
	uploadRange.End = size;
	mbo->m_uploadBuffer->Map(0, &uploadRange, &dest);
	memcpy(dest, data, size);
	mbo->m_uploadBuffer->Unmap(0, &uploadRange);

	m_commandList->CopyBufferRegion(mbo->m_defaultBuffer, 0, mbo->m_uploadBuffer, 0, size);

	ExecuteCommandList();
	SignalAndWait();
	ResetCommandList();
#endif
}

void DX12Renderer::CopyRWMeshBufferCPUToGPU(void const* data, size_t size, MeshBuffer*& mbo)
{
	void* dest = nullptr;

	D3D12_RANGE uploadRange;
	uploadRange.Begin = 0;
	uploadRange.End = size;
	mbo->m_uploadBuffer->Map(0, &uploadRange, &dest);
	memcpy(dest, data, size);
	mbo->m_uploadBuffer->Unmap(0, &uploadRange);

	m_commandList->CopyBufferRegion(mbo->m_defaultBuffer, 0, mbo->m_uploadBuffer, 0, size);

	D3D12_RESOURCE_BARRIER barrier = {};
	barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
	barrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
	barrier.Transition.pResource = mbo->m_defaultBuffer;
	barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST;
	barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
	barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

	m_commandList->ResourceBarrier(1, &barrier);

	ExecuteCommandList();
	SignalAndWait();
	ResetCommandList();
}

void DX12Renderer::BindMeshBuffer(Model* model, Texture* texture)
{
	UNUSED(texture);

	m_commandList->SetGraphicsRootSignature(m_rootSig[RootSig::MESH_SHADER_PIPELINE]);

	m_commandList->SetDescriptorHeaps(1, &model->m_modelDescHeap);
	m_commandList->SetGraphicsRootDescriptorTable(8, model->m_modelDescHeap->GetGPUDescriptorHandleForHeapStart());
}

ConstantBuffer* DX12Renderer::CreateConstantBuffer(size_t const size, std::wstring bufferDebugName)
{
#if DX12_RENDERER

	ConstantBuffer* buffer = new ConstantBuffer(size);
	
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

	D3D12_RESOURCE_DESC resourceDesc = {};
	resourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	resourceDesc.Alignment = 0;
	resourceDesc.Width = size;
	resourceDesc.Height = 1;
	resourceDesc.DepthOrArraySize = 1;
	resourceDesc.MipLevels = 1;
	resourceDesc.Format = DXGI_FORMAT_UNKNOWN;
	resourceDesc.SampleDesc = {1, 0};
	resourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	resourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	HRESULT hr = m_device->CreateCommittedResource(&hpUpload, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_uploadBuffer));
	hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &resourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&buffer->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 vbo!"); 
	}

	std::wstring uploadBufferName = bufferDebugName + std::wstring(L" Upload Buffer");
	std::wstring defaultBufferName = bufferDebugName + std::wstring(L" Default Buffer");

	buffer->m_uploadBuffer->SetName(uploadBufferName.c_str());
	buffer->m_defaultBuffer->SetName(defaultBufferName.c_str());

	return buffer;
#endif
}

void DX12Renderer::CopyCPUToGPU(void const* data, size_t size, ConstantBuffer*& cbo)
{
#if DX12_RENDERER

	void* dest = nullptr;

	D3D12_RANGE uploadRange;
	uploadRange.Begin = 0;
	uploadRange.End = size;
	cbo->m_uploadBuffer->Map(0, &uploadRange, &dest);
	memcpy(dest, data, size);
	cbo->m_uploadBuffer->Unmap(0, &uploadRange);

	m_commandList->CopyBufferRegion(cbo->m_defaultBuffer, 0, cbo->m_uploadBuffer, 0, size);
#endif
}

void DX12Renderer::BindConstantBuffer(int slot, ConstantBuffer* cbo, RootSig pipelineMode)
{
#if DX12_RENDERER
	if (pipelineMode == RootSig::DEFAULT_PIPELINE)
	{
		m_commandList->SetGraphicsRootSignature(m_rootSig[RootSig::DEFAULT_PIPELINE]);
	}
	else if (pipelineMode == RootSig::DIFFUSE_PIPELINE)
	{
		m_commandList->SetGraphicsRootSignature(m_rootSig[RootSig::DIFFUSE_PIPELINE]);
	}
	else if (pipelineMode == RootSig::MESH_SHADER_PIPELINE)
	{
		m_commandList->SetGraphicsRootSignature(m_rootSig[RootSig::MESH_SHADER_PIPELINE]);
	}

	m_commandList->SetGraphicsRootConstantBufferView(slot, cbo->m_defaultBuffer->GetGPUVirtualAddress());
#endif
}

Shader* DX12Renderer::CreateShader(char const* shaderName, char const* shaderSource, VertexType type)
{
	UNUSED(shaderName);
	UNUSED(shaderSource);
	UNUSED(type);

	return nullptr;
}

Shader* DX12Renderer::CreateShader(char const* shaderName, VertexType type)
{
#if DX12_RENDERER

	std::string fileName = "Data/Shaders/" + std::string(shaderName);
	std::string shaderSource;

	fileName += ".hlsl";

	FileReadToString(shaderSource, fileName);

	ShaderConfig shaderConfig;
	shaderConfig.m_name = shaderName;

	Shader* shader = new Shader(shaderConfig);

	CompileShader(&shader->m_vertexShader_5, fileName.c_str(), shaderSource.c_str(), shader->m_config.m_vertexEntryPoint.c_str(), "vs_5_1");
	CompileShader(&shader->m_pixelShader_5, fileName.c_str() ,shaderSource.c_str(), shader->m_config.m_pixelEntryPoint.c_str(), "ps_5_1");

	if (type == VertexType::PC)
	{ 
		D3D12_INPUT_ELEMENT_DESC* inputElementDesc = new D3D12_INPUT_ELEMENT_DESC[2];

		inputElementDesc[0] = {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
		inputElementDesc[1] = {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};

		D3D12_INPUT_LAYOUT_DESC inputLayoutDesc = {};

		inputLayoutDesc.NumElements = 2;
		inputLayoutDesc.pInputElementDescs = inputElementDesc;

		m_pipelineStateDescriptor->InputLayout = inputLayoutDesc;
		m_pipelineStateDescriptor->PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
		m_pipelineStateDescriptor->NumRenderTargets = 1;
		m_pipelineStateDescriptor->RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
		m_pipelineStateDescriptor->NodeMask = 0;
		m_pipelineStateDescriptor->Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
	}
	else if (type == VertexType::PCU)
	{
		D3D12_INPUT_ELEMENT_DESC* inputElementDesc = new D3D12_INPUT_ELEMENT_DESC[3];

		inputElementDesc[0] =	{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
		inputElementDesc[1] = 	{"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
		inputElementDesc[2] = 	{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};

		D3D12_INPUT_LAYOUT_DESC inputLayoutDesc = {};

		inputLayoutDesc.NumElements = 3;
		inputLayoutDesc.pInputElementDescs = inputElementDesc;

		m_pipelineStateDescriptor->InputLayout = inputLayoutDesc;
		m_pipelineStateDescriptor->PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
		m_pipelineStateDescriptor->NumRenderTargets = 1;
		m_pipelineStateDescriptor->RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
		m_pipelineStateDescriptor->NodeMask = 0;
		m_pipelineStateDescriptor->Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
	}
	else if (type == VertexType::PCUTBN)
	{
		D3D12_INPUT_ELEMENT_DESC* inputElementDesc = new D3D12_INPUT_ELEMENT_DESC[6];

		inputElementDesc[0] =	{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
		inputElementDesc[1] =	{"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
		inputElementDesc[2] =	{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
		inputElementDesc[3] =	{"TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
		inputElementDesc[4] =	{"BITANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
		inputElementDesc[5] =	{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};

		D3D12_INPUT_LAYOUT_DESC inputLayoutDesc = {};

		inputLayoutDesc.NumElements = 6;
		inputLayoutDesc.pInputElementDescs = inputElementDesc;

		m_pipelineStateDescriptor->InputLayout = inputLayoutDesc;
		m_pipelineStateDescriptor->PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
		m_pipelineStateDescriptor->NumRenderTargets = 1;
		m_pipelineStateDescriptor->RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
		m_pipelineStateDescriptor->NodeMask = 0;
		m_pipelineStateDescriptor->Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
	}

	m_loadedShaders.push_back(shader);

	return shader;
#endif
	return nullptr;
}

Shader* DX12Renderer::CreateMeshShader(std::wstring shaderName, VertexType type)
{
	UNUSED(type);

#if DX12_RENDERER

	std::wstring shaderSource;
	ShaderConfig shaderConfig;

	Shader* shader = new Shader(shaderConfig);

	DXCCompileShader(&shader->m_meshShader_6, shaderName.c_str(), shaderSource.c_str(), L"MeshMain", L"ms_6_5");
	DXCCompileShader(&shader->m_pixelShader_6, shaderName.c_str() ,shaderSource.c_str(), L"PixelMain", L"ps_6_5");

	m_loadedShaders.push_back(shader);

	return shader;
#endif
	return nullptr;
}

Shader* DX12Renderer::CreateAmplificationShader(std::wstring shaderName, VertexType type)
{
	UNUSED(type);

#if DX12_RENDERER

	std::wstring shaderSource;
	ShaderConfig shaderConfig;

	Shader* shader = new Shader(shaderConfig);

	DXCCompileShader(&shader->m_ampShader_6, shaderName.c_str(), shaderSource.c_str(), L"AmpMain", L"as_6_5");

	m_loadedShaders.push_back(shader);

	return shader;
#endif
	return nullptr;
}

void DX12Renderer::CreateMeshShadingPipelineState()
{
	m_meshPipelineStateDescriptor = new D3DX12_MESH_SHADER_PIPELINE_STATE_DESC();

	const D3D12_DEPTH_STENCILOP_DESC defaultStencilOp = { D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS };

	m_meshPipelineStateDescriptor->pRootSignature = m_rootSig[RootSig::MESH_SHADER_PIPELINE];
	m_meshPipelineStateDescriptor->AS.pShaderBytecode = m_amplificationShader->m_ampShader_6->GetBufferPointer();
	m_meshPipelineStateDescriptor->AS.BytecodeLength = m_amplificationShader->m_ampShader_6->GetBufferSize();
	m_meshPipelineStateDescriptor->CachedPSO = {};
	m_meshPipelineStateDescriptor->Flags = {};
	m_meshPipelineStateDescriptor->PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	m_meshPipelineStateDescriptor->MS.pShaderBytecode = m_meshShader->m_meshShader_6->GetBufferPointer();
	m_meshPipelineStateDescriptor->MS.BytecodeLength = m_meshShader->m_meshShader_6->GetBufferSize();
	m_meshPipelineStateDescriptor->PS.pShaderBytecode = m_meshShader->m_pixelShader_6->GetBufferPointer();
	m_meshPipelineStateDescriptor->PS.BytecodeLength = m_meshShader->m_pixelShader_6->GetBufferSize();
	m_meshPipelineStateDescriptor->NumRenderTargets      = 1;
	m_meshPipelineStateDescriptor->RTVFormats[0]         = m_renderTargets[0]->GetDesc().Format;
	m_meshPipelineStateDescriptor->BlendState.AlphaToCoverageEnable = false;
	m_meshPipelineStateDescriptor->BlendState.IndependentBlendEnable = false;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].BlendEnable = true;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_SRC_ALPHA;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_INV_SRC_ALPHA;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].LogicOpEnable = false;
	m_meshPipelineStateDescriptor->BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
	m_meshPipelineStateDescriptor->SampleMask = 0xFFFFFFF;
	m_meshPipelineStateDescriptor->SampleDesc = {1, 0};
	m_meshPipelineStateDescriptor->RasterizerState = {};
	m_meshPipelineStateDescriptor->RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
	m_meshPipelineStateDescriptor->RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
	m_meshPipelineStateDescriptor->RasterizerState.FrontCounterClockwise = true;
	m_meshPipelineStateDescriptor->RasterizerState.DepthClipEnable = true;	
	m_meshPipelineStateDescriptor->RasterizerState.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;
	m_meshPipelineStateDescriptor->DSVFormat = DXGI_FORMAT_D24_UNORM_S8_UINT;
	m_meshPipelineStateDescriptor->DepthStencilState.DepthEnable = true;
	m_meshPipelineStateDescriptor->DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
	m_meshPipelineStateDescriptor->DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
	m_meshPipelineStateDescriptor->DepthStencilState.StencilEnable = false;
	m_meshPipelineStateDescriptor->DepthStencilState.StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK;
	m_meshPipelineStateDescriptor->DepthStencilState.StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK;
	m_meshPipelineStateDescriptor->DepthStencilState.FrontFace = defaultStencilOp;
	m_meshPipelineStateDescriptor->DepthStencilState.BackFace = defaultStencilOp;

	auto psoStream = CD3DX12_PIPELINE_MESH_STATE_STREAM(*m_meshPipelineStateDescriptor);

	D3D12_PIPELINE_STATE_STREAM_DESC streamDesc;
	streamDesc.pPipelineStateSubobjectStream = &psoStream;
	streamDesc.SizeInBytes                   = sizeof(psoStream);

	HRESULT hr = m_device->CreatePipelineState(&streamDesc, IID_PPV_ARGS(&m_meshPSO));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create mesh shading pipeline"); 
	}
}

Texture* DX12Renderer::CreateComputeInputTexture(Image const& image)
{
	Texture* texture = new Texture();
	texture->m_dimensions = image.GetDimensions();
	texture->m_name = image.GetImageFilePath();

	D3D12_HEAP_PROPERTIES hpDefault{};
	hpDefault.Type = D3D12_HEAP_TYPE_DEFAULT;
	hpDefault.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpDefault.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpDefault.CreationNodeMask = 0;
	hpDefault.VisibleNodeMask = 0;

	D3D12_RESOURCE_DESC defaultResourceDesc = {};
	defaultResourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
	defaultResourceDesc.Alignment = 0;
	defaultResourceDesc.Width = image.GetDimensions().x;
	defaultResourceDesc.Height = image.GetDimensions().y;
	defaultResourceDesc.DepthOrArraySize = 1;
	defaultResourceDesc.MipLevels = 1;
	defaultResourceDesc.Format = DXGI_FORMAT_R32_FLOAT;
	defaultResourceDesc.SampleDesc = {1, 0};
	defaultResourceDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
	defaultResourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	HRESULT hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &defaultResourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&texture->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 texture!"); 
	}

	texture->m_defaultBuffer->SetName(L"Texture Default Buffer");

	return texture;
}

Texture* DX12Renderer::CreateRWTexture(Image const& image)
{
	Texture* texture = new Texture();
	texture->m_dimensions = image.GetDimensions();
	texture->m_name = image.GetImageFilePath();

	D3D12_HEAP_PROPERTIES hpDefault{};
	hpDefault.Type = D3D12_HEAP_TYPE_DEFAULT;
	hpDefault.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpDefault.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpDefault.CreationNodeMask = 0;
	hpDefault.VisibleNodeMask = 0;

	D3D12_RESOURCE_DESC defaultResourceDesc = {};
	defaultResourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
	defaultResourceDesc.Alignment = 0;
	defaultResourceDesc.Width = image.GetDimensions().x;
	defaultResourceDesc.Height = image.GetDimensions().y;
	defaultResourceDesc.DepthOrArraySize = 1;
	defaultResourceDesc.MipLevels = 1;
	defaultResourceDesc.Format = DXGI_FORMAT_R32_FLOAT;
	defaultResourceDesc.SampleDesc = {1, 0};
	defaultResourceDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
	defaultResourceDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

	HRESULT hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &defaultResourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&texture->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 RWtexture!"); 
	}

	texture->m_defaultBuffer->SetName(L"RWTexture Default Buffer");

	return texture;
}

Texture* DX12Renderer::CreateZBufferTexture(Image const& image)
{
	Texture* texture = new Texture();
	texture->m_dimensions = image.GetDimensions();
	texture->m_name = image.GetImageFilePath();

	D3D12_HEAP_PROPERTIES hpDefault{};
	hpDefault.Type = D3D12_HEAP_TYPE_DEFAULT;
	hpDefault.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	hpDefault.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	hpDefault.CreationNodeMask = 0;
	hpDefault.VisibleNodeMask = 0;

	D3D12_RESOURCE_DESC defaultResourceDesc = {};
	defaultResourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
	defaultResourceDesc.Alignment = 0;
	defaultResourceDesc.Width = image.GetDimensions().x;
	defaultResourceDesc.Height = image.GetDimensions().y;
	defaultResourceDesc.DepthOrArraySize = 1;
	defaultResourceDesc.MipLevels = 1;
	defaultResourceDesc.Format = DXGI_FORMAT_R32_FLOAT;
	defaultResourceDesc.SampleDesc = {1, 0};
	defaultResourceDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
	defaultResourceDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;

	HRESULT hr = m_device->CreateCommittedResource(&hpDefault, D3D12_HEAP_FLAG_NONE, &defaultResourceDesc, D3D12_RESOURCE_STATE_COMMON, 0, IID_PPV_ARGS(&texture->m_defaultBuffer));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 RWtexture!"); 
	}

	texture->m_defaultBuffer->SetName(L"Z Texture Default Buffer");

	return texture;
}

Shader* DX12Renderer::CreateComputeShader(std::wstring shaderName)
{
#if DX12_RENDERER

	std::wstring shaderSource;
	ShaderConfig shaderConfig;

	Shader* shader = new Shader(shaderConfig);

	DXCCompileShader(&shader->m_computeShader_6, shaderName.c_str(), shaderSource.c_str(), L"ComputeMain", L"cs_6_5");

	return shader;
#endif
	return nullptr;
}

void DX12Renderer::CreateComputeShadingPipelineState()
{
	m_computePipelineStateDescriptor = new D3D12_COMPUTE_PIPELINE_STATE_DESC();

	const D3D12_DEPTH_STENCILOP_DESC defaultStencilOp = { D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS };

	m_computePipelineStateDescriptor->pRootSignature = m_rootSig[RootSig::COMPUTE_PIPELINE];
	m_computePipelineStateDescriptor->CachedPSO = {};
	m_computePipelineStateDescriptor->Flags = {};
	m_computePipelineStateDescriptor->CS.pShaderBytecode = m_testComputeShader->m_computeShader_6->GetBufferPointer();
	m_computePipelineStateDescriptor->CS.BytecodeLength = m_testComputeShader->m_computeShader_6->GetBufferSize();

	auto computePSOStream = D3D12_COMPUTE_PIPELINE_STATE_DESC(*m_computePipelineStateDescriptor);

	//D3D12_PIPELINE_STATE_STREAM_DESC streamDesc;
	//streamDesc.pPipelineStateSubobjectStream = &computePSOStream;
	//streamDesc.SizeInBytes                   = sizeof(computePSOStream);

	HRESULT hr = m_device->CreateComputePipelineState(&computePSOStream, IID_PPV_ARGS(&m_computePSO));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create mesh shading pipeline"); 
	}
}

bool DX12Renderer::CompileShader(ID3DBlob** shaderBlob, char const* name, char const* source, char const* entryPoint, char const* target)
{
	UINT flags = D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION | D3DCOMPILE_WARNINGS_ARE_ERRORS | D3DCOMPILE_ALL_RESOURCES_BOUND;

	ID3DBlob* errorBuff = nullptr;

	HRESULT hr = D3DCompile(
		source,
		strlen(source),
		name,
		nullptr,
		nullptr,
		entryPoint,
		target,
		flags,
		0,
		shaderBlob,
		&errorBuff
	);

	if (FAILED(hr))
	{
		OutputDebugStringA((char*)errorBuff->GetBufferPointer());
		ERROR_AND_DIE("Could not compile shader");
	}

	return true;
}

bool DX12Renderer::DXCCompileShader(IDxcBlob** shaderBlob, std::wstring name, std::wstring source, std::wstring entryPoint, std::wstring target)
{
	UNUSED(source);

	IDxcBlobEncoding* blobEncoder = nullptr;

	std::wstring fileName = L"Data/Shaders/" + name;
	fileName += L".hlsl";

	HRESULT hr = m_dxcUtils->LoadFile(fileName.c_str(), nullptr, &blobEncoder);

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not load shader file");
	}

	std::vector<LPCWSTR> shaderArgs;

	std::wstring pdbFileName = L"Data/Shaders/" + name + L".pdb";

	shaderArgs.push_back(L"-E");
	shaderArgs.push_back(entryPoint.c_str());

	shaderArgs.push_back(L"-T");
	shaderArgs.push_back(target.c_str());

	shaderArgs.push_back(L"/I");
	shaderArgs.push_back(L"Data/Shaders");

	shaderArgs.push_back(L"-Zi");
	shaderArgs.push_back(L"-Fd");
	shaderArgs.push_back(pdbFileName.c_str());
	shaderArgs.push_back(L"-Qembed_debug");

	shaderArgs.push_back(L"-Fre");
	shaderArgs.push_back(L"refPath");

	DxcBuffer buffer{};

	buffer.Ptr = blobEncoder->GetBufferPointer();
	buffer.Size = blobEncoder->GetBufferSize();

	IDxcResult* result = nullptr;

	hr = m_dxcCompiler->Compile(&buffer, shaderArgs.data(), (UINT32)shaderArgs.size(), m_dxcIncludeHandler, IID_PPV_ARGS(&result));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not compile shader");
	}

	result->GetStatus(&hr);

	IDxcBlobUtf8* errors = nullptr;

	hr = result->GetOutput(DXC_OUT_ERRORS, IID_PPV_ARGS(&errors), nullptr);

	if (errors && errors->GetStringLength())
	{
		OutputDebugStringA((char*)errors->GetStringPointer());
		ERROR_AND_DIE("Could not compile shader");
	}

	if (FAILED(hr))
	{
		OutputDebugStringA((char*)errors->GetBufferPointer());
		ERROR_AND_DIE("Could not compile shader");
	}

	HRESULT status;

	hr = result->GetStatus(&status);

	if (FAILED(hr) || FAILED(status))
	{
		ERROR_AND_DIE("Could not compile shader");
	}

	hr = result->GetOutput(DXC_OUT_OBJECT, IID_PPV_ARGS(shaderBlob), nullptr);

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not compile shader");
	}

	IDxcBlob* pdbData = nullptr;
	IDxcBlobUtf16* pdbPathFromCompiler = nullptr;

	hr = result->GetOutput(DXC_OUT_PDB, IID_PPV_ARGS(&pdbData), &pdbPathFromCompiler);

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not compile shader pdb");
	}

	WriteIDXCBlobToFile(pdbData, pdbPathFromCompiler->GetStringPointer());

	return true;
}

void DX12Renderer::WriteIDXCBlobToFile(IDxcBlob* pBlob, const wchar_t* fileName) {
	if (!pBlob || !fileName) 
	{
		// Handle null pointers as appropriate
		return;
	}

	// Get the size of the data in the blob
	SIZE_T dataSize = pBlob->GetBufferSize();

	// Get a pointer to the data
	const void* data = pBlob->GetBufferPointer();

	// Open a file stream
	std::ofstream fileStream(fileName, std::ios::binary);

	if (!fileStream) {
		// Handle file open error
		return;
	}

	// Write the data to the file
	fileStream.write(reinterpret_cast<const char*>(data), dataSize);

	if (!fileStream) {
		// Handle write error
		fileStream.close();
		return;
	}

	// Close the file stream
	fileStream.close();
}

void DX12Renderer::BindShader(Shader* shader)
{
#if DX12_RENDERER

	if(shader == nullptr)
	{
		m_pipelineStateDescriptor->VS.pShaderBytecode = m_defaultShader->m_vertexShader_5->GetBufferPointer();
		m_pipelineStateDescriptor->VS.BytecodeLength = m_defaultShader->m_vertexShader_5->GetBufferSize();

		m_pipelineStateDescriptor->PS.pShaderBytecode = m_defaultShader->m_pixelShader_5->GetBufferPointer();
		m_pipelineStateDescriptor->PS.BytecodeLength = m_defaultShader->m_pixelShader_5->GetBufferSize();

		return;
	}

	m_pipelineStateDescriptor->VS.pShaderBytecode = shader->m_vertexShader_5->GetBufferPointer();
	m_pipelineStateDescriptor->VS.BytecodeLength = shader->m_vertexShader_5->GetBufferSize();

	m_pipelineStateDescriptor->PS.pShaderBytecode = shader->m_pixelShader_5->GetBufferPointer();
	m_pipelineStateDescriptor->PS.BytecodeLength = shader->m_pixelShader_5->GetBufferSize();

#endif
}

void DX12Renderer::SetCullingConstants(RootSig pipelineMode, Camera* camera, int rootSigSlot)
{
	Constants cameraConstant;

	cameraConstant.CullingViewMatrix = camera->GetViewMatrix();
	cameraConstant.CullingProjectionMatrix = camera->GetProjectionMatrix();

	CopyCPUToGPU(&cameraConstant, m_constantsCBO->m_size, m_constantsCBO);
	BindConstantBuffer(rootSigSlot, m_constantsCBO, pipelineMode);
}

void DX12Renderer::SetModelConstants(RootSig pipelineMode, int rootSigSlot, Mat44 const& modelMatrix, Rgba8 const& modelColor, ConstantBuffer* modelCBO)
{
	ModelConstants modelConstants;

	float modelColorInFloats[4];
	modelColor.GetAsFloats(modelColorInFloats);

	modelConstants.ModelColor = Vec4(modelColorInFloats[0], modelColorInFloats[1], modelColorInFloats[2], modelColorInFloats[3]);
	modelConstants.ModelMatrix = modelMatrix;

	if (modelCBO == nullptr)
	{
		CopyCPUToGPU(&modelConstants, m_modelCBO->m_size, m_modelCBO);
		BindConstantBuffer(rootSigSlot, m_modelCBO, pipelineMode);
	}
	else
	{
		CopyCPUToGPU(&modelConstants, modelCBO->m_size, modelCBO);
		BindConstantBuffer(rootSigSlot, modelCBO, pipelineMode);
	}
}

void DX12Renderer::SetModelConstants(std::vector<Mat44> const& modelMatrix, std::vector<Rgba8> const& modelColor)
{
	UNUSED(modelMatrix);
	UNUSED(modelColor);

	// TODO
	/*
	std::vector<ModelConstants> modelConstants;

	for (int i = 0; i < 2; i++)
	{
		ModelConstants model;
		Rgba8 modelRgba;

		float modelColorInFloats[4];
		modelColor[i].GetAsFloats(modelColorInFloats);

		model.ModelColor = Vec4(modelColorInFloats[0], modelColorInFloats[1], modelColorInFloats[2], modelColorInFloats[3]);
		model.ModelMatrix = modelMatrix[i];

		modelConstants.push_back(model);
	}
	
	CopyCPUToGPU(modelConstants.data(), m_modelCBO->m_size, m_modelCBO);
	BindConstantBuffer(k_modelConstantSlot, m_modelCBO);*/
}

void DX12Renderer::SetDirectionalLightConstants(Vec3 sunDirection, float sunIntensity, float ambientIntensity, int rootSigSlot, RootSig rootSig)
{
	DirectionalLightConstants lightConstants;

	lightConstants.SunDirection			= sunDirection.GetNormalized();
	lightConstants.SunIntensity			= sunIntensity;
	lightConstants.AmbientIntensity		= ambientIntensity;

	CopyCPUToGPU(&lightConstants, m_directionalLightCBO->m_size, m_directionalLightCBO);
	BindConstantBuffer(rootSigSlot, m_directionalLightCBO, rootSig);
}

void DX12Renderer::SetPointLightConstants(std::vector<Vec3> pointPosition, std::vector<Rgba8> pointColor)
{
	UNUSED(pointPosition);
	UNUSED(pointColor);

}

void DX12Renderer::SetSpotLightConstants(std::vector<Vec3> spotLightPosition, std::vector<float> cutOff, std::vector<Vec3> spotLightDirection, std::vector<Rgba8> spotColor)
{
	UNUSED(spotLightPosition);
	UNUSED(cutOff);
	UNUSED(spotLightDirection);
	UNUSED(spotColor);

}

void DX12Renderer::SetMeshletDebugConstants(ProfilerConstants debugConstants)
{
	CopyCPUToGPU(&debugConstants, m_meshletDebugCBO->m_size, m_meshletDebugCBO);
	BindConstantBuffer(k_meshletDebugConstantSlot, m_meshletDebugCBO, RootSig::MESH_SHADER_PIPELINE);
}

void DX12Renderer::SetBlendMode(BlendMode blendMode)
{
	switch (blendMode)
	{
	case BlendMode::ALPHA:
	{
		m_desiredBlendMode = BlendMode::ALPHA;

		break;
	}
	case BlendMode::ADDITIVE:
	{
		m_desiredBlendMode = BlendMode::ADDITIVE;

		break;
	}
	case BlendMode::OPAQUE:
	{
		m_desiredBlendMode = BlendMode::OPAQUE;

		break;
	}
	}
}

void DX12Renderer::SetBlendStatesIfChanged()
{
	if (m_currentBlendMode != m_desiredBlendMode)
	{
		
	}
}

void DX12Renderer::SetSamplerMode(SamplerMode sampleMode)
{
	switch (sampleMode)
	{
	case SamplerMode::POINT_CLAMP:
	{
		m_desiredSamplerMode = SamplerMode::POINT_CLAMP;

		break;									   
	}											   
	case SamplerMode::BILINEAR_WRAP:
	{			
		m_desiredSamplerMode = SamplerMode::BILINEAR_WRAP;

		break;		
	}		
	}
}

void DX12Renderer::SetSamplerStatesIfChanged()
{
}

void DX12Renderer::SetRasterizerMode(RasterizerMode rasterizerMode)
{
	switch (rasterizerMode)
	{
	case RasterizerMode::SOLID_CULL_NONE:
	{
		m_desiredRasterizerMode = RasterizerMode::SOLID_CULL_NONE;

		break;									   
	}											   
	case RasterizerMode::SOLID_CULL_BACK:					   
	{		
		m_desiredRasterizerMode = RasterizerMode::SOLID_CULL_BACK;

		break;		
	}											   
	case RasterizerMode::WIREFRAME_CULL_NONE:					   
	{			
		m_desiredRasterizerMode = RasterizerMode::WIREFRAME_CULL_NONE;

		break;		
	}										   
	case RasterizerMode::WIREFRAME_CULL_BACK:					   
	{			
		m_desiredRasterizerMode = RasterizerMode::WIREFRAME_CULL_BACK;

		break;		
	}
	}
}

void DX12Renderer::SetRasterizerStatesIfChanged()
{
	if (m_currentRasterizerMode != m_desiredRasterizerMode)
	{
	}
}

void DX12Renderer::SetDepthMode(DepthMode depthMode)
{
	switch (depthMode)
	{
	case DepthMode::ENABLED:
	{
		m_desiredDepthStencilMode = DepthMode::ENABLED;

		break;									   
	}											   
	case DepthMode::DISABLED:					   
	{			
		m_desiredDepthStencilMode = DepthMode::DISABLED;

		break;		
	}		
	}
}

void DX12Renderer::SetDepthStatesIfChanged()
{
}

void DX12Renderer::SetShadingMode(ShadingMode shadingMode)
{
	switch (shadingMode)
	{
	case ShadingMode::DEFAULT:
	{
		m_desiredShadingMode = ShadingMode::DEFAULT;

		break;									   
	}											   
	case ShadingMode::DIFFUSE:					   
	{			
		m_desiredShadingMode = ShadingMode::DIFFUSE;

		break;		
	}		
	}
}

void DX12Renderer::CreateRootSignature(RootSig rootSig)
{
	switch (rootSig)
	{
	case DEFAULT_PIPELINE:
	{
		D3D12_DESCRIPTOR_RANGE  descriptorTableRanges[1];
		descriptorTableRanges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		descriptorTableRanges[0].NumDescriptors = 1;
		descriptorTableRanges[0].BaseShaderRegister = 0;
		descriptorTableRanges[0].RegisterSpace = 0;
		descriptorTableRanges[0].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		// create a descriptor table
		D3D12_ROOT_DESCRIPTOR_TABLE descriptorTable;
		descriptorTable.NumDescriptorRanges = _countof(descriptorTableRanges);
		descriptorTable.pDescriptorRanges = &descriptorTableRanges[0];

		D3D12_ROOT_PARAMETER rootParam[3] = {};
		rootParam[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[0].Descriptor.ShaderRegister = 2;
		rootParam[0].Descriptor.RegisterSpace = 0;
		rootParam[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[1].Descriptor.ShaderRegister = 3;
		rootParam[1].Descriptor.RegisterSpace = 0;
		rootParam[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
		rootParam[2].DescriptorTable = descriptorTable;
		rootParam[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL; 

		D3D12_STATIC_SAMPLER_DESC sampler = {};
		sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
		sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.MipLODBias = 0;
		sampler.MaxAnisotropy = 0;
		sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
		sampler.BorderColor = D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK;
		sampler.MinLOD = 0.0f;
		sampler.MaxLOD = D3D12_FLOAT32_MAX;
		sampler.ShaderRegister = 0;
		sampler.RegisterSpace = 0;
		sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

		D3D12_VERSIONED_ROOT_SIGNATURE_DESC rsDesc = {};
		rsDesc.Version = D3D_ROOT_SIGNATURE_VERSION_1_0;
		rsDesc.Desc_1_0.pParameters = rootParam;
		rsDesc.Desc_1_0.NumParameters = 3;
		rsDesc.Desc_1_0.NumStaticSamplers = 1;
		rsDesc.Desc_1_0.pStaticSamplers = &sampler;
		rsDesc.Desc_1_0.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

		ID3DBlob* pSerializedRS = nullptr;
		ID3DBlob* pErrorBlob = nullptr;

		HRESULT hr = D3D12SerializeVersionedRootSignature(&rsDesc, &pSerializedRS, &pErrorBlob);

		if (FAILED(hr))
		{
			OutputDebugStringA((char*)pErrorBlob->GetBufferPointer());
			ERROR_AND_DIE("Bruh3");
		}

		hr = m_device->CreateRootSignature(0, pSerializedRS->GetBufferPointer(), pSerializedRS->GetBufferSize(), IID_PPV_ARGS(&m_rootSig[RootSig::DEFAULT_PIPELINE]));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 rs!"); 
		}
		break;
	}
	case DIFFUSE_PIPELINE:
	{
		D3D12_DESCRIPTOR_RANGE  descriptorTableRanges[1];
		descriptorTableRanges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		descriptorTableRanges[0].NumDescriptors = 1;
		descriptorTableRanges[0].BaseShaderRegister = 0;
		descriptorTableRanges[0].RegisterSpace = 0;
		descriptorTableRanges[0].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		// create a descriptor table
		D3D12_ROOT_DESCRIPTOR_TABLE descriptorTable;
		descriptorTable.NumDescriptorRanges = _countof(descriptorTableRanges);
		descriptorTable.pDescriptorRanges = &descriptorTableRanges[0];

		D3D12_ROOT_PARAMETER rootParam[4] = {};
		rootParam[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[0].Descriptor.ShaderRegister = 1;
		rootParam[0].Descriptor.RegisterSpace = 0;
		rootParam[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[1].Descriptor.ShaderRegister = 2;
		rootParam[1].Descriptor.RegisterSpace = 0;
		rootParam[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[2].Descriptor.ShaderRegister = 3;
		rootParam[2].Descriptor.RegisterSpace = 0;
		rootParam[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
		rootParam[3].DescriptorTable = descriptorTable;
		rootParam[3].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL; 

		D3D12_STATIC_SAMPLER_DESC sampler = {};
		sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_POINT;
		sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.MipLODBias = 0;
		sampler.MaxAnisotropy = 0;
		sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
		sampler.BorderColor = D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK;
		sampler.MinLOD = 0.0f;
		sampler.MaxLOD = D3D12_FLOAT32_MAX;
		sampler.ShaderRegister = 0;
		sampler.RegisterSpace = 0;
		sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

		D3D12_VERSIONED_ROOT_SIGNATURE_DESC rsDesc = {};
		rsDesc.Version = D3D_ROOT_SIGNATURE_VERSION_1_0;
		rsDesc.Desc_1_0.pParameters = rootParam;
		rsDesc.Desc_1_0.NumParameters = 4;
		rsDesc.Desc_1_0.NumStaticSamplers = 1;
		rsDesc.Desc_1_0.pStaticSamplers = &sampler;
		rsDesc.Desc_1_0.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

		ID3DBlob* pSerializedRS = nullptr;
		ID3DBlob* pErrorBlob = nullptr;

		HRESULT hr = D3D12SerializeVersionedRootSignature(&rsDesc, &pSerializedRS, &pErrorBlob);

		if (FAILED(hr))
		{
			OutputDebugStringA((char*)pErrorBlob->GetBufferPointer());
			ERROR_AND_DIE("Bruh3");
		}

		hr = m_device->CreateRootSignature(0, pSerializedRS->GetBufferPointer(), pSerializedRS->GetBufferSize(), IID_PPV_ARGS(&m_rootSig[RootSig::DIFFUSE_PIPELINE]));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 rs!"); 
		}
		break;
	}
	case COMPUTE_PIPELINE:
	{
		D3D12_DESCRIPTOR_RANGE  depthBufferDescriptorTableRanges[2];
		depthBufferDescriptorTableRanges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		depthBufferDescriptorTableRanges[0].NumDescriptors = 1;
		depthBufferDescriptorTableRanges[0].BaseShaderRegister = 0;
		depthBufferDescriptorTableRanges[0].RegisterSpace = 0;
		depthBufferDescriptorTableRanges[0].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		depthBufferDescriptorTableRanges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
		depthBufferDescriptorTableRanges[1].NumDescriptors = 1;
		depthBufferDescriptorTableRanges[1].BaseShaderRegister = 0;
		depthBufferDescriptorTableRanges[1].RegisterSpace = 0;
		depthBufferDescriptorTableRanges[1].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		// create a descriptor table
		D3D12_ROOT_DESCRIPTOR_TABLE descriptorTable1;
		descriptorTable1.NumDescriptorRanges = _countof(depthBufferDescriptorTableRanges);
		descriptorTable1.pDescriptorRanges = &depthBufferDescriptorTableRanges[0];

		D3D12_ROOT_PARAMETER rootParam[2] = {};
		rootParam[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
		rootParam[0].DescriptorTable = descriptorTable1;
		rootParam[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		D3D12_STATIC_SAMPLER_DESC sampler = {};
		sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_POINT;
		sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
		sampler.MipLODBias = 0;
		sampler.MaxAnisotropy = 0;
		sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
		sampler.BorderColor = D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK;
		sampler.MinLOD = 0.0f;
		sampler.MaxLOD = D3D12_FLOAT32_MAX;
		sampler.ShaderRegister = 0;
		sampler.RegisterSpace = 0;
		sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		D3D12_VERSIONED_ROOT_SIGNATURE_DESC rsDesc = {};
		rsDesc.Version = D3D_ROOT_SIGNATURE_VERSION_1_0;
		rsDesc.Desc_1_0.pParameters = rootParam;
		rsDesc.Desc_1_0.NumParameters = 1;
		rsDesc.Desc_1_0.NumStaticSamplers = 1;
		rsDesc.Desc_1_0.pStaticSamplers = &sampler;
		rsDesc.Desc_1_0.Flags = D3D12_ROOT_SIGNATURE_FLAG_NONE;

		ID3DBlob* pSerializedRS = nullptr;
		ID3DBlob* pErrorBlob = nullptr;

		HRESULT hr = D3D12SerializeVersionedRootSignature(&rsDesc, &pSerializedRS, &pErrorBlob);

		if (FAILED(hr))
		{
			OutputDebugStringA((char*)pErrorBlob->GetBufferPointer());
			ERROR_AND_DIE("Bruh3");
		}

		hr = m_device->CreateRootSignature(0, pSerializedRS->GetBufferPointer(), pSerializedRS->GetBufferSize(), IID_PPV_ARGS(&m_rootSig[RootSig::COMPUTE_PIPELINE]));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 rs!"); 
		}
		break;
	}
	case MESH_SHADER_PIPELINE:
	{
		D3D12_DESCRIPTOR_RANGE  structuredBufferDescriptorTableRanges[10];

		structuredBufferDescriptorTableRanges[0].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		structuredBufferDescriptorTableRanges[0].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[0].BaseShaderRegister = 0;
		structuredBufferDescriptorTableRanges[0].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[0].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
		
		structuredBufferDescriptorTableRanges[1].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		structuredBufferDescriptorTableRanges[1].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[1].BaseShaderRegister = 1;
		structuredBufferDescriptorTableRanges[1].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[1].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		structuredBufferDescriptorTableRanges[2].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		structuredBufferDescriptorTableRanges[2].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[2].BaseShaderRegister = 2;
		structuredBufferDescriptorTableRanges[2].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[2].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
		
		structuredBufferDescriptorTableRanges[3].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		structuredBufferDescriptorTableRanges[3].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[3].BaseShaderRegister = 3;
		structuredBufferDescriptorTableRanges[3].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[3].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		structuredBufferDescriptorTableRanges[4].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		structuredBufferDescriptorTableRanges[4].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[4].BaseShaderRegister = 4;
		structuredBufferDescriptorTableRanges[4].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[4].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		structuredBufferDescriptorTableRanges[5].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		structuredBufferDescriptorTableRanges[5].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[5].BaseShaderRegister = 5;
		structuredBufferDescriptorTableRanges[5].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[5].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		structuredBufferDescriptorTableRanges[6].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		structuredBufferDescriptorTableRanges[6].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[6].BaseShaderRegister = 6;
		structuredBufferDescriptorTableRanges[6].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[6].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		structuredBufferDescriptorTableRanges[7].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
		structuredBufferDescriptorTableRanges[7].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[7].BaseShaderRegister = 7;
		structuredBufferDescriptorTableRanges[7].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[7].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		structuredBufferDescriptorTableRanges[8].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
		structuredBufferDescriptorTableRanges[8].NumDescriptors = 8;
		structuredBufferDescriptorTableRanges[8].BaseShaderRegister = 0;
		structuredBufferDescriptorTableRanges[8].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[8].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
		
		structuredBufferDescriptorTableRanges[9].RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_UAV;
		structuredBufferDescriptorTableRanges[9].NumDescriptors = 1;
		structuredBufferDescriptorTableRanges[9].BaseShaderRegister = 11;
		structuredBufferDescriptorTableRanges[9].RegisterSpace = 0;
		structuredBufferDescriptorTableRanges[9].OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

		// create a descriptor table
		D3D12_ROOT_DESCRIPTOR_TABLE descriptorTable;
		descriptorTable.NumDescriptorRanges = _countof(structuredBufferDescriptorTableRanges);
		descriptorTable.pDescriptorRanges = structuredBufferDescriptorTableRanges;

		D3D12_ROOT_PARAMETER rootParam[9] = {};
		rootParam[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[0].Descriptor.ShaderRegister = 0;
		rootParam[0].Descriptor.RegisterSpace = 0;
		rootParam[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[1].Descriptor.ShaderRegister = 1;
		rootParam[1].Descriptor.RegisterSpace = 0;
		rootParam[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[2].Descriptor.ShaderRegister = 2;
		rootParam[2].Descriptor.RegisterSpace = 0;
		rootParam[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[3].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[3].Descriptor.ShaderRegister = 4;
		rootParam[3].Descriptor.RegisterSpace = 0;
		rootParam[3].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[4].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[4].Descriptor.ShaderRegister = 5;
		rootParam[4].Descriptor.RegisterSpace = 0;
		rootParam[4].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[5].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[5].Descriptor.ShaderRegister = 6;
		rootParam[5].Descriptor.RegisterSpace = 0;
		rootParam[5].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[6].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[6].Descriptor.ShaderRegister = 7;
		rootParam[6].Descriptor.RegisterSpace = 0;
		rootParam[6].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[7].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
		rootParam[7].Descriptor.ShaderRegister = 8;
		rootParam[7].Descriptor.RegisterSpace = 0;
		rootParam[7].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		rootParam[8].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
		rootParam[8].DescriptorTable = descriptorTable;
		rootParam[8].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

		D3D12_STATIC_SAMPLER_DESC sampler = {};
		sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
		sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
		sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
		sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_WRAP;
		sampler.MipLODBias = 0;
		sampler.MaxAnisotropy = 0;
		sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
		sampler.BorderColor = D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK;
		sampler.MinLOD = 0.0f;
		sampler.MaxLOD = D3D12_FLOAT32_MAX;
		sampler.ShaderRegister = 0;
		sampler.RegisterSpace = 0;
		sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;

		D3D12_VERSIONED_ROOT_SIGNATURE_DESC rsDesc = {};
		rsDesc.Version = D3D_ROOT_SIGNATURE_VERSION_1_0;
		rsDesc.Desc_1_0.pParameters = rootParam;
		rsDesc.Desc_1_0.NumParameters = 9;
		rsDesc.Desc_1_0.NumStaticSamplers = 1;
		rsDesc.Desc_1_0.pStaticSamplers = &sampler;
		rsDesc.Desc_1_0.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

		ID3DBlob* pSerializedRS = nullptr;
		ID3DBlob* pErrorBlob = nullptr;

		HRESULT hr = D3D12SerializeVersionedRootSignature(&rsDesc, &pSerializedRS, &pErrorBlob);

		if (FAILED(hr))
		{
			OutputDebugStringA((char*)pErrorBlob->GetBufferPointer());
			ERROR_AND_DIE("Bruh3");
		}

		hr = m_device->CreateRootSignature(0, pSerializedRS->GetBufferPointer(), pSerializedRS->GetBufferSize(), IID_PPV_ARGS(&m_rootSig[RootSig::MESH_SHADER_PIPELINE]));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 rs!"); 
		}
		break;
	}
	}
}

void DX12Renderer::CreateAllRootSignatures()
{
	CreateRootSignature(RootSig::DEFAULT_PIPELINE);
	CreateRootSignature(RootSig::DIFFUSE_PIPELINE);
	CreateRootSignature(RootSig::COMPUTE_PIPELINE);
	CreateRootSignature(RootSig::MESH_SHADER_PIPELINE);
}

void DX12Renderer::CreateAllPipelineStates()
{
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE);

	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE);
	CreatePipelineState(PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE);
}

void DX12Renderer::CreatePipelineState(PipelineState pipelineState)
{
	switch (pipelineState)
	{
	case PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE: //1
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE: //2
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE: //3
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE: //4
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE: //5
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE: //6
	{		
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE: //7
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE: //8
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE: //9
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE: //10
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE: //11
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE: //12
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE: //13
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE: //14
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE: //15
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE: //16
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE: //17
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE: //18
	{		
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE: //19
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE: //20
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE: //21
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE: //22
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE: //23
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE: //24
	{
		SetPipelineStateDesc(ShadingMode::DEFAULT, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE: //1
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE: //2
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE: //3
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE: //4
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE: //5
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE: //6
	{		
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE: //7
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE: //8
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE: //9
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE: //10
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE: //11
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE: //12
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_LESS_EQUAL, D3D12_DEPTH_WRITE_MASK_ALL);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE: //13
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE: //14
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE: //15
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE: //16
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE: //17
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE: //18
	{		
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_BACK, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE: //19
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE: //20
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE: //21
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_SOLID, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE: //22
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_SRC_ALPHA, D3D12_BLEND_INV_SRC_ALPHA, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE: //23
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ONE, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE] = pso;

		break;
	}
	case PipelineState::DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE: //24
	{
		SetPipelineStateDesc(ShadingMode::DIFFUSE, D3D12_BLEND_ONE, D3D12_BLEND_ZERO, D3D12_FILL_MODE_WIREFRAME, D3D12_CULL_MODE_NONE, D3D12_COMPARISON_FUNC_ALWAYS, D3D12_DEPTH_WRITE_MASK_ZERO);

		ID3D12PipelineState* pso = nullptr;

		HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

		if (FAILED(hr))
		{
			ERROR_AND_DIE("Could not create D3D12 pso!"); 
		}

		m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE] = pso;

		break;
	}
	default:
	{
		break;
	}
	}
}

void DX12Renderer::SetPipelineStateDesc(ShadingMode shadingMode, D3D12_BLEND srcBlend, D3D12_BLEND destBlend, D3D12_FILL_MODE filleMode, D3D12_CULL_MODE cullMode, D3D12_COMPARISON_FUNC depthCompFunc, D3D12_DEPTH_WRITE_MASK depthWriteMask)
{

	if (shadingMode == ShadingMode::DEFAULT)
	{
		m_pipelineStateDescriptor->pRootSignature = m_rootSig[RootSig::DEFAULT_PIPELINE];
		m_pipelineStateDescriptor->VS.pShaderBytecode = m_defaultShader->m_vertexShader_5->GetBufferPointer();
		m_pipelineStateDescriptor->VS.BytecodeLength = m_defaultShader->m_vertexShader_5->GetBufferSize();

		m_pipelineStateDescriptor->PS.pShaderBytecode = m_defaultShader->m_pixelShader_5->GetBufferPointer();
		m_pipelineStateDescriptor->PS.BytecodeLength = m_defaultShader->m_pixelShader_5->GetBufferSize();
	}
	else if(shadingMode == ShadingMode::DIFFUSE)
	{
		m_pipelineStateDescriptor->pRootSignature = m_rootSig[RootSig::DIFFUSE_PIPELINE];
		m_pipelineStateDescriptor->VS.pShaderBytecode = m_diffuseShader->m_vertexShader_5->GetBufferPointer();
		m_pipelineStateDescriptor->VS.BytecodeLength = m_diffuseShader->m_vertexShader_5->GetBufferSize();

		m_pipelineStateDescriptor->PS.pShaderBytecode = m_diffuseShader->m_pixelShader_5->GetBufferPointer();
		m_pipelineStateDescriptor->PS.BytecodeLength = m_diffuseShader->m_pixelShader_5->GetBufferSize();
	}

	m_pipelineStateDescriptor->BlendState.AlphaToCoverageEnable = false;
	m_pipelineStateDescriptor->BlendState.IndependentBlendEnable = false;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].BlendEnable = true;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].SrcBlend = srcBlend;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].DestBlend = destBlend;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].SrcBlendAlpha = m_pipelineStateDescriptor->BlendState.RenderTarget[0].SrcBlend;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].DestBlendAlpha = m_pipelineStateDescriptor->BlendState.RenderTarget[0].DestBlend;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].BlendOpAlpha = m_pipelineStateDescriptor->BlendState.RenderTarget[0].BlendOp;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].LogicOpEnable = false;
	m_pipelineStateDescriptor->BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

	m_pipelineStateDescriptor->SampleMask = 0xFFFFFFF;
	m_pipelineStateDescriptor->SampleDesc = {1, 0};

	m_pipelineStateDescriptor->RasterizerState = {};

	m_pipelineStateDescriptor->RasterizerState.FillMode = filleMode;
	m_pipelineStateDescriptor->RasterizerState.CullMode = cullMode;
	m_pipelineStateDescriptor->RasterizerState.FrontCounterClockwise = true;
	m_pipelineStateDescriptor->RasterizerState.DepthClipEnable = true;
	m_pipelineStateDescriptor->RasterizerState.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;

	m_pipelineStateDescriptor->DSVFormat = DXGI_FORMAT_D24_UNORM_S8_UINT;
	m_pipelineStateDescriptor->DepthStencilState.DepthEnable = true;
	m_pipelineStateDescriptor->DepthStencilState.DepthFunc = depthCompFunc;
	m_pipelineStateDescriptor->DepthStencilState.DepthWriteMask = depthWriteMask;

	const D3D12_DEPTH_STENCILOP_DESC defaultStencilOp = { D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS };

	m_pipelineStateDescriptor->DepthStencilState.StencilEnable = false;
	m_pipelineStateDescriptor->DepthStencilState.StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK;
	m_pipelineStateDescriptor->DepthStencilState.StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK;
	m_pipelineStateDescriptor->DepthStencilState.FrontFace = defaultStencilOp;
	m_pipelineStateDescriptor->DepthStencilState.BackFace = defaultStencilOp;
}

void DX12Renderer::SetPipelineState()
{
	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DEFAULT && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DEFAULT_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::ENABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_BACK && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_WCB_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_SCN_DEPTH_ENABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::SOLID_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_OP_SAMPLE_POINT_RAST_SCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ALPHA && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AL_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::ADDITIVE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE]);
	}

	if (m_desiredShadingMode == ShadingMode::DIFFUSE && m_desiredBlendMode == BlendMode::OPAQUE && m_desiredDepthStencilMode == DepthMode::DISABLED && m_desiredRasterizerMode == RasterizerMode::WIREFRAME_CULL_NONE && m_desiredSamplerMode == SamplerMode::POINT_CLAMP)
	{
		m_commandList->SetPipelineState(m_PSO[DIFFUSE_BLEND_AD_SAMPLE_POINT_RAST_WCN_DEPTH_DISABLE]);
	}
}

void DX12Renderer::CreateZPrepassPipelineState()
{
	m_pipelineStateDescriptor = {};
	m_pipelineStateDescriptor = new D3D12_GRAPHICS_PIPELINE_STATE_DESC();

	D3D12_INPUT_ELEMENT_DESC* inputElementDesc = new D3D12_INPUT_ELEMENT_DESC[3];

	inputElementDesc[0] =	{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
	inputElementDesc[1] = 	{"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};
	inputElementDesc[2] = 	{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0};

	D3D12_INPUT_LAYOUT_DESC inputLayoutDesc = {};

	inputLayoutDesc.NumElements = 3;
	inputLayoutDesc.pInputElementDescs = inputElementDesc;

	m_pipelineStateDescriptor->RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;                 // Use solid fill
	m_pipelineStateDescriptor->RasterizerState.CullMode = D3D12_CULL_MODE_BACK;                  // Back-face culling
	m_pipelineStateDescriptor->RasterizerState.FrontCounterClockwise = FALSE;                    // Default winding order
	m_pipelineStateDescriptor->RasterizerState.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;             // Default depth bias
	m_pipelineStateDescriptor->RasterizerState.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;  // Default depth bias clamp
	m_pipelineStateDescriptor->RasterizerState.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;  // Default slope bias
	m_pipelineStateDescriptor->RasterizerState.DepthClipEnable = TRUE;                           // Enable depth clipping
	m_pipelineStateDescriptor->RasterizerState.MultisampleEnable = FALSE;                        // No MSAA for Z-prepass
	m_pipelineStateDescriptor->RasterizerState.AntialiasedLineEnable = FALSE;                    // Not needed for 3D geometry
	m_pipelineStateDescriptor->RasterizerState.ForcedSampleCount = 0;                            // No forced sampling
	m_pipelineStateDescriptor->RasterizerState.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF; // No conservative raster

	m_pipelineStateDescriptor->InputLayout = inputLayoutDesc;
	m_pipelineStateDescriptor->PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	m_pipelineStateDescriptor->NodeMask = 0;
	m_pipelineStateDescriptor->Flags = D3D12_PIPELINE_STATE_FLAG_NONE;

	m_pipelineStateDescriptor->RTVFormats[0] = DXGI_FORMAT_UNKNOWN;
	m_pipelineStateDescriptor->NumRenderTargets = 0;
	m_pipelineStateDescriptor->DSVFormat = DXGI_FORMAT_D24_UNORM_S8_UINT;  // Your depth buffer format
	m_pipelineStateDescriptor->DepthStencilState.DepthEnable = TRUE;
	m_pipelineStateDescriptor->DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
	m_pipelineStateDescriptor->DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS;
	m_pipelineStateDescriptor->DepthStencilState.StencilEnable = FALSE;                        // No stencil

	m_pipelineStateDescriptor->pRootSignature = m_rootSig[RootSig::DEFAULT_PIPELINE];

	m_pipelineStateDescriptor->VS.pShaderBytecode = m_defaultShader->m_vertexShader_5->GetBufferPointer();
	m_pipelineStateDescriptor->VS.BytecodeLength = m_defaultShader->m_vertexShader_5->GetBufferSize();

	m_pipelineStateDescriptor->PS.pShaderBytecode = nullptr;
	m_pipelineStateDescriptor->PS.BytecodeLength = 0;

	m_pipelineStateDescriptor->SampleMask = 0xFFFFFFF;
	m_pipelineStateDescriptor->SampleDesc = {1, 0};

	ID3D12PipelineState* pso = nullptr;

	HRESULT hr = m_device->CreateGraphicsPipelineState(m_pipelineStateDescriptor, IID_PPV_ARGS(&pso));

	if (FAILED(hr))
	{
		ERROR_AND_DIE("Could not create D3D12 pso!"); 
	}

	m_zPrepassPSO = pso;
}

void DX12Renderer::ClearScreen(Rgba8 const& clearColor)
{
	UNUSED(clearColor);

	D3D12_RESOURCE_BARRIER rbDesc = {};
	rbDesc.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
	rbDesc.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
	rbDesc.Transition.pResource = m_renderTargets[m_frameIndex];
	rbDesc.Transition.Subresource = 0;
	rbDesc.Transition.StateBefore = D3D12_RESOURCE_STATE_PRESENT;
	rbDesc.Transition.StateAfter = D3D12_RESOURCE_STATE_RENDER_TARGET;

	m_commandList->ResourceBarrier(1, &rbDesc);

	D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle = m_rtvDescHeap->GetCPUDescriptorHandleForHeapStart();
	cpuHandle.ptr += (size_t)m_frameIndex * m_heapIncrement;

	D3D12_CPU_DESCRIPTOR_HANDLE dsvHandle = m_dsvDescHeap->GetCPUDescriptorHandleForHeapStart();

	float colorAsFloats[4];
	clearColor.GetAsFloats(colorAsFloats);

	m_commandList->ClearRenderTargetView(cpuHandle, colorAsFloats, 0, 0);
	m_commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
	m_commandList->OMSetRenderTargets(1, &cpuHandle, false, &dsvHandle);
}

void DX12Renderer::BeginCamera(Camera& camera, RootSig pipelineMode, int rootSigSlot)
{
	CameraConstants cameraConstant;

	cameraConstant.ViewMatrix = camera.GetViewMatrix();
	cameraConstant.ProjectionMatrix = camera.GetProjectionMatrix();

	CopyCPUToGPU(&cameraConstant, camera.m_cameraCBO->m_size, camera.m_cameraCBO);
	BindConstantBuffer(rootSigSlot, camera.m_cameraCBO, pipelineMode);

	AABB2 dxViewport = camera.GetDXViewport();

	D3D12_VIEWPORT viewport;
	viewport.TopLeftX = dxViewport.m_mins.x;
	viewport.TopLeftY = dxViewport.m_mins.y;
	viewport.Width = dxViewport.GetDimensions().x;
	viewport.Height = dxViewport.GetDimensions().y;
	viewport.MinDepth = 0.0f;
	viewport.MaxDepth = 1.0f;

	D3D12_RECT scissorRect;
	scissorRect.left = 0;
	scissorRect.right = (LONG)viewport.Width;
	scissorRect.top = 0;
	scissorRect.bottom = (LONG)viewport.Height;

	m_commandList->RSSetViewports(1, &viewport);
	m_commandList->RSSetScissorRects(1, &scissorRect);
}

void DX12Renderer::EndCamera(Camera const& camera)
{
	UNUSED(camera);
}

void DX12Renderer::DrawVertexBuffer(VertexBuffer* vbo, int vertexCount, int vertexStride, int vertexOffset, PrimitiveType type)
{	
	BindVertexBuffer(vbo, vertexStride);
	SetPipelineState();

	if (type == PrimitiveType::TRIANGLE_LIST)
	{
		m_commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	}
	else if (type == PrimitiveType::LINE_LIST)
	{
		m_commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
	}

	m_commandList->DrawInstanced(vertexCount, 1, vertexOffset, 0);
}

void DX12Renderer::DrawVertexBufferIndexed(VertexBuffer* vbo, IndexBuffer* ibo, int vertexStride)
{
	BindVertexBuffer(vbo, vertexStride);
	BindIndexBuffer(ibo);
	SetPipelineState();
	m_commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	m_commandList->DrawIndexedInstanced((UINT)ibo->m_size / sizeof(unsigned int), 1, 0, 0, 0);
}

void DX12Renderer::DrawVertexArray(int numVertexes, Vertex_PCU const* vertexes)
{
	CopyCPUToGPU(vertexes, numVertexes * 24, m_immediateVBO);
	DrawVertexBuffer(m_immediateVBO, numVertexes, 24);
}

void DX12Renderer::DrawVertexArray(int numVertexes, Vertex_PCU const* vertexes, VertexBuffer* vbo)
{
	CopyCPUToGPU(vertexes, numVertexes * 24, vbo);
	DrawVertexBuffer(vbo, numVertexes, 24);
}

void DX12Renderer::DrawVertexArrayIndexed(int numVertexes, Vertex_PCU const* vertexes, std::vector<unsigned int> const& indices)
{
	UNUSED(numVertexes);
	UNUSED(vertexes);
	UNUSED(indices); 
}

void DX12Renderer::DrawVertexArray(int numVertexes, Vertex_PCUTBN const* vertexes)
{
	UNUSED(numVertexes);
	UNUSED(vertexes);
}

void DX12Renderer::DrawVertexArrayIndexed(int numVertexes, Vertex_PCUTBN const* vertexes, std::vector<unsigned int> const& indices)
{
	UNUSED(numVertexes);
	UNUSED(vertexes);
	UNUSED(indices);
}

void DX12Renderer::DrawMesh(int numOfMeshlets)
{
	m_commandList->SetPipelineState(m_meshPSO);
	m_commandList->DispatchMesh(numOfMeshlets, 1, 1);
}

void DX12Renderer::DispatchComputeShader()
{
	D3D12_GPU_DESCRIPTOR_HANDLE gpuHandle = m_ZBuffersDescHeap->GetGPUDescriptorHandleForHeapStart();
	UINT descriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

	for (int i = 0; i < m_hiZBuffers.size(); i++)
	{
		m_commandList->SetPipelineState(m_computePSO);
		m_commandList->SetComputeRootSignature(m_rootSig[RootSig::COMPUTE_PIPELINE]);

		m_commandList->SetDescriptorHeaps(1, &m_ZBuffersDescHeap);
		m_commandList->SetComputeRootDescriptorTable(0, gpuHandle);

		D3D12_RESOURCE_BARRIER barrier = {};
		barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
		barrier.Transition.pResource = m_hiZBuffers[i]->m_defaultBuffer;
		barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST; // Example previous state
		barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
		barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

		m_commandList->ResourceBarrier(1, &barrier);

		IntVec2 dispatchThreadgrps = IntVec2(m_hiZBuffers[i]->GetDimensions().x * 2, m_hiZBuffers[i]->GetDimensions().y * 2);

		dispatchThreadgrps.x = (dispatchThreadgrps.x + 15) / 16;
		dispatchThreadgrps.y = (dispatchThreadgrps.y + 15) / 16;

		m_commandList->Dispatch(dispatchThreadgrps.x, dispatchThreadgrps.y, 1);

		barrier = {};
		barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
		barrier.Transition.pResource = m_hiZBuffers[i]->m_defaultBuffer;
		barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS; // Example previous state
		barrier.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
		barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
		
		m_commandList->ResourceBarrier(1, &barrier);

		gpuHandle.ptr += descriptorSize;
		gpuHandle.ptr += descriptorSize;
	}
}

void DX12Renderer::GenerateZBuffer()
{
	D3D12_CPU_DESCRIPTOR_HANDLE dsvHandle = m_dsvDescHeap->GetCPUDescriptorHandleForHeapStart();

	m_commandList->SetPipelineState(m_zPrepassPSO);
	m_commandList->OMSetRenderTargets(0, nullptr, FALSE, &dsvHandle);

	m_commandList->SetGraphicsRootSignature(m_rootSig[RootSig::DEFAULT_PIPELINE]);
	m_commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	m_commandList->DrawInstanced(0, 1, 0, 0);
}

#endif