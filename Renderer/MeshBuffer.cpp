#include "Engine/Renderer/MeshBuffer.hpp"

#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

#if DX12_RENDERER
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#endif

MeshBuffer::MeshBuffer(size_t size)
	: m_size(size)
{
}

MeshBuffer::~MeshBuffer()
{
#if DX12_RENDERER
	DX_SAFE_RELEASE(m_uploadBuffer);
	DX_SAFE_RELEASE(m_defaultBuffer);
#endif
}
