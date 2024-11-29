#include "NamedStrings.hpp"

#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Core/Rgba8.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/IntVec2.hpp"

void NamedStrings::PopulateFromXmlElementAttributes(XmlElement const& element, bool isAdditional)
{
	
	const XmlAttribute* attribute = element.FirstAttribute();

	while (attribute != nullptr)
	{
		if (isAdditional)
		{
			auto it = m_keyValuePairs.find(attribute->Name());
			if (it != m_keyValuePairs.end())
			{
				// Attribute already exists, decide whether to append or override.
				it->second += "," + std::string(attribute->Value());
			}
			else
			{
				// Attribute does not exist, add it.
				m_keyValuePairs[attribute->Name()] = attribute->Value();
			}
		}
		else
		{
			// Default behavior for adding new attributes.
			m_keyValuePairs[attribute->Name()] = attribute->Value();
		}
		attribute = attribute->Next();
	}
}

bool NamedStrings::HasArgument(std::string const& keyName)
{
	std::map<std::string, std::string>::const_iterator it = m_keyValuePairs.find(keyName);

    if (it != m_keyValuePairs.end())
    {
        return true;
    }
    return false;
}

void NamedStrings::SetValue(std::string const& keyName, std::string const& newValue)
{
	m_keyValuePairs[keyName] = newValue;
}

std::string NamedStrings::GetValue(std::string const& keyName, std::string const& defaultValue) const
{
	std::map<std::string, std::string>::const_iterator found = m_keyValuePairs.find(keyName);
	
	if (found == m_keyValuePairs.end())
		return defaultValue;
	else
	{
		return found->second;
	}
}

bool NamedStrings::GetValue(std::string const& keyName, bool defaultValue) const
{
	std::map<std::string, std::string>::const_iterator found = m_keyValuePairs.find(keyName);
	
	if (found == m_keyValuePairs.end())
		return defaultValue;
	else
	{
		bool value;

		std::string text = found->second.c_str();

		if (text == "true")
			value = true;
		else
			value = false;

		return value;
	}
}

int NamedStrings::GetValue(std::string const& keyName, int defaultValue) const
{
	std::map<std::string, std::string>::const_iterator found = m_keyValuePairs.find(keyName);
	
	if (found == m_keyValuePairs.end())
		return defaultValue;
	else
	{
		int value;
		
		value = std::atoi(found->second.c_str());

		return value;
	}
}

float NamedStrings::GetValue(std::string const& keyName, float defaultValue) const
{
	std::map<std::string, std::string>::const_iterator found = m_keyValuePairs.find(keyName);

	if (found == m_keyValuePairs.end())
		return defaultValue;
	else
	{
		float value;

		value = static_cast<float>(std::atof(found->second.c_str()));

		return value;
	}
}

std::string NamedStrings::GetValue(std::string const& keyName, char const* defaultValue) const
{
	std::map<std::string, std::string>::const_iterator found = m_keyValuePairs.find(keyName);

	if (found == m_keyValuePairs.end())
		return defaultValue;
	else
	{
		std::string value;

		value = found->second.c_str();

		return value;
	}
}

Rgba8 NamedStrings::GetValue(std::string const& keyName, Rgba8 const& defaultValue) const
{
	std::map<std::string, std::string>::const_iterator found = m_keyValuePairs.find(keyName);

	if (found == m_keyValuePairs.end())
		return defaultValue;
	else
	{
		Rgba8 value;

		value.SetFromText(found->second.c_str());

		return value;
	}
}

Vec2 NamedStrings::GetValue(std::string const& keyName, Vec2 const& defaultValue) const
{
	std::map<std::string, std::string>::const_iterator found = m_keyValuePairs.find(keyName);

	if (found == m_keyValuePairs.end())
		return defaultValue;
	else
	{
		Vec2 value;

		value.SetFromText(found->second.c_str());

		return value;
	}
}

IntVec2 NamedStrings::GetValue(std::string const& keyName, IntVec2 const& defaultValue) const
{
	std::map<std::string, std::string>::const_iterator found = m_keyValuePairs.find(keyName);

	if (found == m_keyValuePairs.end())
		return defaultValue;
	else
	{
		IntVec2 value;

		value.SetFromText(found->second.c_str());

		return value;
	}
}
