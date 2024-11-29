#pragma once

#include "Engine/Core/EngineCommon.hpp"

#include "Game/EngineBuildPreferences.hpp"

struct ID3D11Buffer;

struct ID3D12Resource;

class VertexBuffer
{
public:
#if DX11_RENDERER
	ID3D11Buffer* m_buffer = nullptr;
#elif DX12_RENDERER
	ID3D12Resource* m_uploadBuffer = nullptr;
	ID3D12Resource* m_defaultBuffer = nullptr;
#endif
	size_t m_size = 0;
public:
	VertexBuffer(size_t size);
	VertexBuffer(VertexBuffer const& copy) = delete;
	~VertexBuffer();
private:
	//friend class Renderer;
	friend class DX11Renderer;
	friend class DX12Renderer;
};