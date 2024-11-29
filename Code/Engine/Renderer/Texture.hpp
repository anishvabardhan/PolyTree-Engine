#pragma once

#include "Engine/Core/EngineCommon.hpp"
#include "Engine/Math/IntVec2.hpp"

#include "Game/EngineBuildPreferences.hpp"

#include <string>

struct ID3D11Texture2D;
struct ID3D11ShaderResourceView;
struct ID3D11UnorderedAccessView;
struct ID3D11RenderTargetView;

struct ID3D12Resource;
struct ID3D12DescriptorHeap;

class Texture
{
	friend class Renderer; // Only the Renderer can create new Texture objects!
	friend class DX11Renderer;
	friend class DX12Renderer;
	friend class Image;

public:
	Texture(); // can't instantiate directly; must ask Renderer to do it for you
	Texture(Texture const& copy) = delete; // No copying allowed!  This represents GPU memory.
	~Texture();

public:
	IntVec2				GetDimensions() const { return m_dimensions; }
	std::string const& GetImageFilePath() const { return m_name; }
public:
	std::string			m_name;
	IntVec2				m_dimensions;
#if DX11_RENDERER
	ID3D11Texture2D* m_texture = nullptr;
	ID3D11ShaderResourceView* m_shaderResourceView = nullptr;
	ID3D11UnorderedAccessView* m_unorderedAccessView = nullptr;
	ID3D11RenderTargetView* m_renderTargetView = nullptr;
#elif DX12_RENDERER
	ID3D12Resource* m_uploadBuffer = nullptr;
	ID3D12Resource* m_defaultBuffer = nullptr;
	ID3D12DescriptorHeap* m_shaderResourceView = nullptr;
	ID3D12DescriptorHeap* m_UAVDescHeap = nullptr;
#endif
};