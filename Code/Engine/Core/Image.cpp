#include "Engine/Core/Image.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "ThirdParty/stb_image/stb_image.h"

#include "Engine/Core/Rgba8.hpp"

Image::Image(char const* imageFilePath)
	: m_imageFilePath(imageFilePath)
{
	int bytesPerTexel = 0;
	int numComponentsRequested = 0;

	stbi_set_flip_vertically_on_load(1);
	unsigned char* texelData = stbi_load(m_imageFilePath.c_str(), &m_dimensions.x, &m_dimensions.y, &bytesPerTexel, numComponentsRequested);

	int totaltexels = m_dimensions.x * m_dimensions.y;

	for (int index = 0; index < totaltexels * bytesPerTexel; index += bytesPerTexel)
	{
		if (bytesPerTexel == 3)
		{
			m_rgbaTexels.push_back(Rgba8(texelData[index], texelData[index + 1], texelData[index + 2], 255));
		}
		else if (bytesPerTexel == 4)
		{
			m_rgbaTexels.push_back(Rgba8(texelData[index], texelData[index + 1], texelData[index + 2], texelData[index + 3]));
		}
	}
}

Image::Image(IntVec2 size, Rgba8 color)
	: m_dimensions(size)
{
	//int bytesPerTexel = 4;

	int totaltexels = m_dimensions.x * m_dimensions.y;

	for (int index = 0; index < totaltexels; index ++)
	{
		m_rgbaTexels.push_back(color);
	}
}

std::string const& Image::GetImageFilePath() const
{
	return m_imageFilePath;
}

IntVec2 Image::GetDimensions() const
{
	return m_dimensions;
}

void const* Image::GetRawData() const
{
	return m_rgbaTexels.data();
}

Rgba8 Image::GetTexelColor(IntVec2 const& texelCoords) const
{
	int texelIndex = texelCoords.x + (texelCoords.y * m_dimensions.x);

	if (texelCoords.x < 0 || texelCoords.x > m_dimensions.x || texelCoords.y < 0 || texelCoords.y > m_dimensions.y)
	{
		return Rgba8(0, 0, 0, 0);
	}

	return m_rgbaTexels[texelIndex];
}

void Image::SetTexelColor(IntVec2 const& texelCoords, Rgba8 const& newColor)
{
	int texelIndex = texelCoords.x + (texelCoords.y * m_dimensions.x);

	if (texelCoords.x < 0 || texelCoords.x > m_dimensions.x || texelCoords.y < 0 || texelCoords.y > m_dimensions.y)
	{
		m_rgbaTexels[texelIndex] = Rgba8(0, 0, 0, 0);
		return;
	}

	m_rgbaTexels[texelIndex] = newColor;
}
