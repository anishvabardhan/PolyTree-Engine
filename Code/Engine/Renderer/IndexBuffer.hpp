#pragma once

#include "Game/EngineBuildPreferences.hpp"

struct ID3D11Buffer;

struct ID3D12Resource;

class IndexBuffer
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
	IndexBuffer(size_t count);
	IndexBuffer(IndexBuffer const& copy) = delete;
	~IndexBuffer();
private:
	//friend class Renderer;
	friend class DX11Renderer;
	friend class DX12Renderer;
};