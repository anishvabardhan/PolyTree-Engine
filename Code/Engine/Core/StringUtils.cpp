#include "Engine/Core/StringUtils.hpp"

#include <stdarg.h>
#include <sstream>

#include "Engine/Core/ErrorWarningAssert.hpp"

//-----------------------------------------------------------------------------------------------
constexpr int STRINGF_STACK_LOCAL_TEMP_LENGTH = 2048;

//-----------------------------------------------------------------------------------------------
const std::string Stringf( char const* format, ... )
{
	char textLiteral[ STRINGF_STACK_LOCAL_TEMP_LENGTH ];
	va_list variableArgumentList;
	va_start( variableArgumentList, format );
	vsnprintf_s( textLiteral, STRINGF_STACK_LOCAL_TEMP_LENGTH, _TRUNCATE, format, variableArgumentList );	
	va_end( variableArgumentList );
	textLiteral[ STRINGF_STACK_LOCAL_TEMP_LENGTH - 1 ] = '\0'; // In case vsnprintf overran (doesn't auto-terminate)

	return std::string( textLiteral );
}


//-----------------------------------------------------------------------------------------------
const std::string Stringf( int maxLength, char const* format, ... )
{
	char textLiteralSmall[ STRINGF_STACK_LOCAL_TEMP_LENGTH ];
	char* textLiteral = textLiteralSmall;
	if( maxLength > STRINGF_STACK_LOCAL_TEMP_LENGTH )
		textLiteral = new char[ maxLength ];

	va_list variableArgumentList;
	va_start( variableArgumentList, format );
	vsnprintf_s( textLiteral, maxLength, _TRUNCATE, format, variableArgumentList );	
	va_end( variableArgumentList );
	textLiteral[ maxLength - 1 ] = '\0'; // In case vsnprintf overran (doesn't auto-terminate)

	std::string returnValue( textLiteral );
	if( maxLength > STRINGF_STACK_LOCAL_TEMP_LENGTH )
		delete[] textLiteral;

	return returnValue;
}

Strings SplitStringOnDelimiter(std::string const& originalString, char delimiterToSplitOn)
{
	std::istringstream ss(originalString);
	std::string token;
	Strings tokens;

	while (std::getline(ss, token, delimiterToSplitOn))
	{
		tokens.push_back(token);
	}

	return tokens;
}

Strings SplitStringWithQuotes(std::string const& originalString, char delimiterToSplitOn, bool removeInsideQuotes)
{
    Strings result;
    bool insideQuotes = false;
    std::string currentToken;

    for (char ch : originalString)
    {
        if (ch == '"')
        {
            // Toggle the insideQuotes flag
            insideQuotes = !insideQuotes;
            if (!removeInsideQuotes)
            {
                // Keep the quote if we are not removing them
                currentToken += ch;
            }
        }
        else if (ch == delimiterToSplitOn && !insideQuotes)
        {
            // Add the current token to the result and clear for the next one
            if (!currentToken.empty())
            {
                result.push_back(currentToken);
                currentToken.clear();
            }
        }
        else
        {
            // Add character to the current token
            currentToken += ch;
        }
    }

    // Add the last token if it exists
    if (!currentToken.empty())
    {
        result.push_back(currentToken);
    }

    // Trim quotes from tokens if they were inside quotes
    if (removeInsideQuotes)
    {
        for (std::string& token : result)
        {
            if (token.size() >= 2 && token.front() == '"' && token.back() == '"')
            {
                token = token.substr(1, token.size() - 2);  // Remove the leading and trailing quotes
            }
        }
    }

    return result;
}

void TrimString(std::string& originalString, char delimiterToTrim)
{
    if (!originalString.empty() && originalString.front() == delimiterToTrim)
    {
        originalString.erase(0, 1); // Remove the first character
    }
    if (!originalString.empty() && originalString.back() == delimiterToTrim)
    {
        originalString.pop_back(); // Remove the last character
    }
}

int SplitStringOnDelimiter(Strings& outString, std::string const& originalString, std::string const& delimiterToSplitOn)
{
	char const* p = strstr(originalString.c_str(), delimiterToSplitOn.c_str());
	
	if(p == nullptr)
		return 0;

	std::string::size_type start = 0;
	std::string::size_type end = 0;

	while ((end = originalString.find(delimiterToSplitOn, start)) != std::string::npos) {
		outString.push_back(originalString.substr(start, end - start));
		start = end + delimiterToSplitOn.length();
	}
	outString.push_back(originalString.substr(start));

	return 1;
}
//
//Strings SplitStringOnDelimiter(std::string const& originalString, std::string const& delimiterToSplitOn, bool removeEmpty)
//{
//	return Strings();
//}
