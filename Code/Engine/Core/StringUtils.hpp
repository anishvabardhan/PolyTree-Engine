#pragma once
//-----------------------------------------------------------------------------------------------
#include <string>
#include <vector>

typedef std::vector<std::string> Strings;

//-----------------------------------------------------------------------------------------------
const std::string Stringf( char const* format, ... );
const std::string Stringf( int maxLength, char const* format, ... );

Strings SplitStringOnDelimiter(std::string const& originalString, char delimiterToSplitOn);
Strings SplitStringWithQuotes(std::string const& originalString, char delimiterToSplitOn, bool removeInsideQuotes = false);
void TrimString(std::string& originalString, char delimiterToTrim);
//Strings SplitStringOnDelimiter(std::string const& originalString, char delimiterToSplitOn = ',', bool removeEmpty = false);
int SplitStringOnDelimiter(Strings& outString, std::string const& originalString, std::string const& delimiterToSplitOn);
//Strings SplitStringOnDelimiter(std::string const& originalString, std::string const& delimiterToSplitOn, bool removeEmpty = false);