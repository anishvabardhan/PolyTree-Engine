#include "XmlUtils.hpp"

#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Math/EulerAngles.hpp"
#include "Engine/Math/IntVec2.hpp"
#include "Engine/Math/FloatRange.hpp"
#include "Engine/Core/Rgba8.hpp"

int ParseXmlAttribute(XmlElement const& element, char const* attributeName, int defaultValue)
{
	int intValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		int temp = std::atoi(valueText);
		intValue = temp;
	}

	return intValue;
}

char ParseXmlAttribute(XmlElement const& element, char const* attributeName, char defaultValue)
{
	char charValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		char temp = valueText[0];
		charValue = temp;
	}

	return charValue;
}

bool ParseXmlAttribute(XmlElement const& element, char const* attributeName, bool defaultValue)
{
	bool boolValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		std::string valueTextString = valueText;

		if(valueTextString.compare("true") == 0)
			boolValue = true;

		else if (valueTextString.compare("false") == 0)
			boolValue = false;
	}

	return boolValue;
}

float ParseXmlAttribute(XmlElement const& element, char const* attributeName, float defaultValue)
{
	float floatValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		float temp = static_cast<float>(std::atof(valueText));
		floatValue = temp;
	}

	return floatValue;
}

FloatRange ParseXmlAttribute(XmlElement const& element, char const* attributeName, FloatRange defaultValue)
{
	FloatRange floatRangeValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		Strings floatValues = SplitStringOnDelimiter(std::string(valueText), '~');

		float rangeStart = static_cast<float>(std::atof(floatValues[0].c_str()));
		float rangeEnd = static_cast<float>(std::atof(floatValues[1].c_str()));

		floatRangeValue = FloatRange(rangeStart, rangeEnd);
	}

	return floatRangeValue;
}

Rgba8 ParseXmlAttribute(XmlElement const& element, char const* attributeName, Rgba8 const& defaultValue)
{
	Rgba8 rgbaValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		Rgba8 temp;
		temp.SetFromText(valueText);

		rgbaValue = temp;
	}

	return rgbaValue;
}

Vec2 ParseXmlAttribute(XmlElement const& element, char const* attributeName, Vec2 const& defaultValue)
{
	Vec2 vec2Value = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		Vec2 temp;
		temp.SetFromText(valueText);

		vec2Value = temp;
	}

	return vec2Value;
}

Vec3 ParseXmlAttribute(XmlElement const& element, char const* attributeName, Vec3 const& defaultValue)
{
	Vec3 vec3Value = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		Vec3 temp;
		temp.SetFromText(valueText);

		vec3Value = temp;
	}

	return vec3Value;
}

EulerAngles ParseXmlAttribute(XmlElement const& element, char const* attributeName, EulerAngles const& defaultValue)
{
	EulerAngles eulerValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		EulerAngles temp;
		temp.SetFromText(valueText);

		eulerValue = temp;
	}

	return eulerValue;
}

IntVec2 ParseXmlAttribute(XmlElement const& element, char const* attributeName, IntVec2 const& defaultValue)
{
	IntVec2 intVec2Value = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		IntVec2 temp;
		temp.SetFromText(valueText);

		intVec2Value = temp;
	}

	return intVec2Value;
}

std::string ParseXmlAttribute(XmlElement const& element, char const* attributeName, std::string const& defaultValue)
{
	std::string stringValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		stringValue = valueText;
	}

	return stringValue;
}

Strings ParseXmlAttribute(XmlElement const& element, char const* attributeName, Strings const& defaultValues)
{
	Strings stringValues = defaultValues;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		Strings temp = SplitStringOnDelimiter(valueText, ',');
		stringValues = temp;
	}

	return stringValues;
}

std::string ParseXmlAttribute(XmlElement const& element, char const* attributeName, char const* defaultValue)
{
	std::string stringValue = defaultValue;

	char const* valueText = element.Attribute(attributeName);

	if (valueText)
	{
		stringValue = std::string(valueText);
	}

	return stringValue;
}
