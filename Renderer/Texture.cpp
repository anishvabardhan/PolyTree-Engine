#include "Engine/Renderer/Texture.hpp"

#include "Engine/Renderer/Renderer.hpp"

#if DX11_RENDERER
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>
#elif DX12_RENDERER
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#endif

Texture::Texture()
{
}

Texture::~Texture()
{
#if DX11_RENDERER
	DX_SAFE_RELEASE(m_texture);
	DX_SAFE_RELEASE(m_shaderResourceView);
	DX_SAFE_RELEASE(m_unorderedAccessView);
	DX_SAFE_RELEASE(m_renderTargetView);
#elif DX12_RENDERER
	DX_SAFE_RELEASE(m_uploadBuffer);
	DX_SAFE_RELEASE(m_defaultBuffer);
	DX_SAFE_RELEASE(m_shaderResourceView);
#endif
}
