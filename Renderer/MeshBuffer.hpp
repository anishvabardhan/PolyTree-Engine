#pragma once

#include "Engine/Core/EngineCommon.hpp"

#include "Game/EngineBuildPreferences.hpp"

struct ID3D12Resource;

class MeshBuffer
{
#if DX12_RENDERER
public:
	ID3D12Resource* m_uploadBuffer = nullptr;
	ID3D12Resource* m_defaultBuffer = nullptr;
#endif
	size_t m_size = 0;
public:
	MeshBuffer(size_t size);
	MeshBuffer(MeshBuffer const& copy) = delete;
	~MeshBuffer();
private:
	//friend class Renderer;
	friend class DX12Renderer;
};