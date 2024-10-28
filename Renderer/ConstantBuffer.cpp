#include "ConstantBuffer.hpp"

#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"

#if DX11_RENDERER
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>
#elif DX12_RENDERER
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#endif


ConstantBuffer::ConstantBuffer(size_t size)
	: m_size(size)
{
}

ConstantBuffer::~ConstantBuffer()
{
#if DX11_RENDERER
	DX_SAFE_RELEASE(m_buffer);
#elif DX12_RENDERER
	DX_SAFE_RELEASE(m_uploadBuffer);
	DX_SAFE_RELEASE(m_defaultBuffer);
#endif
}
