#include "Engine/Core/Rgba8.hpp"

#include "Engine/Core/StringUtils.hpp"
#include "Engine/Math/MathUtils.hpp"

Rgba8 const Rgba8::AQUA						= Rgba8(154, 255, 251, 255);
Rgba8 const Rgba8::LIGHT_BLUE				= Rgba8(45, 170, 214, 255);
Rgba8 const Rgba8::PURPLE					= Rgba8(163, 73, 164, 255);
Rgba8 const Rgba8::TRANSLUCENT_YELLOW		= Rgba8(255, 242, 0, 100);
Rgba8 const Rgba8::YELLOW					= Rgba8(255, 242, 0, 255);
Rgba8 const Rgba8::SAND						= Rgba8(242, 210, 169, 255);
Rgba8 const Rgba8::LIGHT_ORANGE				= Rgba8(255, 165, 0, 255);
Rgba8 const Rgba8::DARK_ORANGE				= Rgba8(255, 127, 39, 255);
Rgba8 const Rgba8::TRANSLUCENT_RED			= Rgba8(255, 0, 0, 100); 
Rgba8 const Rgba8::LIGHT_RED				= Rgba8(255, 105, 105, 255); // make the color lighter
Rgba8 const Rgba8::RED						= Rgba8(255, 0, 0, 255);
Rgba8 const Rgba8::DARK_RED					= Rgba8(136, 0, 21, 255);
Rgba8 const Rgba8::LIGHT_GREEN				= Rgba8(150, 255, 146, 255); // make the color lighter 
Rgba8 const Rgba8::TRANSLUCENT_GREEN		= Rgba8(0, 255, 0, 100);
Rgba8 const Rgba8::GREEN					= Rgba8(0, 255, 0, 255);
Rgba8 const Rgba8::DARK_GREEN				= Rgba8(24, 128, 55, 255);
Rgba8 const Rgba8::BLUE						= Rgba8(0, 0, 255, 255);
Rgba8 const Rgba8::DARK_BLUE				= Rgba8(0, 18, 154, 255);
Rgba8 const Rgba8::WHITE					= Rgba8(255, 255, 255, 255);
Rgba8 const Rgba8::BLACK					= Rgba8(0, 0, 0, 255);
Rgba8 const Rgba8::TRANSLUCENT_BLACK		= Rgba8(0, 0, 0, 25);
Rgba8 const Rgba8::BROWN					= Rgba8(150, 75, 0, 255);
Rgba8 const Rgba8::DARK_GRAY				= Rgba8(128, 128, 128, 255);
Rgba8 const Rgba8::LIGHT_GRAY				= Rgba8(212, 212, 212, 255);
Rgba8 const Rgba8::TRANSLUCENT				= Rgba8(255, 255, 255, 50);
Rgba8 const Rgba8::LESS_TRANSLUCENT			= Rgba8(255, 255, 255, 150);
Rgba8 const Rgba8::MAGENTA					= Rgba8(235, 72, 235, 255);
Rgba8 const Rgba8::CYAN						= Rgba8(0,   255, 255, 255);

Rgba8::Rgba8()
{
}

Rgba8::Rgba8(unsigned char redByte, unsigned char greenByte, unsigned char blueByte, unsigned char alphaByte)
	: r(redByte), g(greenByte), b(blueByte), a(alphaByte)
{
}

Rgba8::~Rgba8()
{
}

Rgba8 Rgba8::SetFromText(char const* text)
{
	Strings splitValues = SplitStringOnDelimiter(text, ',');

	unsigned char rValue = static_cast<unsigned char>(std::stoi(splitValues[0]));
	unsigned char gValue = static_cast<unsigned char>(std::stoi(splitValues[1]));
	unsigned char bValue = static_cast<unsigned char>(std::stoi(splitValues[2]));
	unsigned char aValue = 255;

	if (splitValues.size() == 4)
	{
		aValue = static_cast<unsigned char>(std::stoi(splitValues[3]));
	}

	r = rValue > 255 ? 255 : rValue;
	g = gValue > 255 ? 255 : gValue;
	b = bValue > 255 ? 255 : bValue;
	a = aValue > 255 ? 255 : aValue;
		
	return Rgba8(r, g, b, a);
}

void Rgba8::GetAsFloats(float* colorAsFloats) const
{
	colorAsFloats[0] = static_cast<float>(r) / 255.0f;
	colorAsFloats[1] = static_cast<float>(g) / 255.0f;
	colorAsFloats[2] = static_cast<float>(b) / 255.0f;
	colorAsFloats[3] = static_cast<float>(a) / 255.0f;
}

Rgba8 Interpolate(Rgba8 start, Rgba8 end, float fractionToEnd)
{
	float r = Interpolate(NormalizeByte(start.r), NormalizeByte(end.r), fractionToEnd);
	float g = Interpolate(NormalizeByte(start.g), NormalizeByte(end.g), fractionToEnd);
	float b = Interpolate(NormalizeByte(start.b), NormalizeByte(end.b), fractionToEnd);
	float a = Interpolate(NormalizeByte(start.a), NormalizeByte(end.a), fractionToEnd);

	return Rgba8(DenormalizeByte(r), DenormalizeByte(g), DenormalizeByte(b), DenormalizeByte(a));
}