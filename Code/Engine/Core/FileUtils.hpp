#pragma once

#include <vector>
#include <string>

int FileReadToBuffer(std::vector<uint8_t>& outBuffer, std::string& fileName);
int FileReadToString(std::string& outString, std::string& fileName);
void WriteBufferToFile(std::vector<unsigned char>& inBuffer, std::string& fileName);
bool CreateFolder(std::string const& folderPathName);
