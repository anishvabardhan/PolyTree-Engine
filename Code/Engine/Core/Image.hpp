#pragma once

#include "Engine/Core/EngineCommon.hpp"

#include "Engine/Math/IntVec2.hpp"

class Image
{
	std::string					m_imageFilePath;
	IntVec2						m_dimensions = IntVec2(0, 0);
public:
	std::vector< Rgba8 >		m_rgbaTexels;
								Image(char const* imageFilePath);
								Image(IntVec2 size, Rgba8 color);
	std::string const&			GetImageFilePath() const;
	IntVec2						GetDimensions() const;
	void const*					GetRawData() const;
	Rgba8						GetTexelColor(IntVec2 const& texelCoords) const;
	void						SetTexelColor(IntVec2 const& texelCoords, Rgba8 const& newColor);
};