#pragma once

struct Rgba8
{
public:
	unsigned char		r = 255;
	unsigned char		g = 255;
	unsigned char		b = 255;
	unsigned char		a = 255;

    static const Rgba8 LESS_TRANSLUCENT;
    static const Rgba8 TRANSLUCENT;
    static const Rgba8 WHITE;
    static const Rgba8 TRANSLUCENT_RED;
    static const Rgba8 LIGHT_RED;
    static const Rgba8 RED;
    static const Rgba8 DARK_RED;
    static const Rgba8 GREEN;
    static const Rgba8 TRANSLUCENT_GREEN;
    static const Rgba8 LIGHT_GREEN;
    static const Rgba8 DARK_GREEN;
    static const Rgba8 BLUE;
    static const Rgba8 DARK_BLUE;
    static const Rgba8 BLACK;
    static const Rgba8 TRANSLUCENT_BLACK;
    static const Rgba8 BROWN;
    static const Rgba8 LIGHT_ORANGE;
    static const Rgba8 DARK_ORANGE;
    static const Rgba8 TRANSLUCENT_YELLOW;
    static const Rgba8 YELLOW;
    static const Rgba8 SAND;
    static const Rgba8 PURPLE;
    static const Rgba8 DARK_GRAY;
    static const Rgba8 LIGHT_GRAY;
    static const Rgba8 LIGHT_BLUE;
    static const Rgba8 AQUA;
    static const Rgba8 MAGENTA;
    static const Rgba8 CYAN;
public:
						Rgba8();
						Rgba8(unsigned char redByte, unsigned char greenByte, unsigned char blueByte, unsigned char alphaByte);
						~Rgba8();

	Rgba8				SetFromText(char const* text);
	void				GetAsFloats(float* colorAsFloats) const;
};

Rgba8					Interpolate(Rgba8 start, Rgba8 end, float fractionToEnd);