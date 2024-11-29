#include "Engine/Core/FileUtils.hpp"

#include "Engine/Core/ErrorWarningAssert.hpp"

#include <windows.h>

int FileReadToBuffer(std::vector<uint8_t>& outBuffer, std::string& fileName)
{
	FILE* fileptr = nullptr;

	fopen_s(&fileptr, fileName.c_str(), "rb");

	if (fileptr)
	{
		fseek(fileptr, 0, SEEK_END);

		size_t fileSize = (size_t)ftell(fileptr);

		outBuffer.resize(fileSize);

		fseek(fileptr, 0, SEEK_SET);

		fread(outBuffer.data(), fileSize, 1, fileptr);

		fclose(fileptr);

		return 0;
	}
	else
	{
		return 1;
	}
}

int FileReadToString(std::string& outString, std::string& fileName)
{
	std::vector<uint8_t> outBuffer;

	int result = FileReadToBuffer(outBuffer, fileName);

	outBuffer.push_back('\0');

	for (size_t index = 0; index < outBuffer.size(); index++)
	{
		outString.push_back(outBuffer[index]);
	}

	return result;
}

void WriteBufferToFile(std::vector<unsigned char>& outBuffer, std::string& fileName)
{
	FILE* fileptr = nullptr;

	int error = fopen_s(&fileptr, fileName.c_str(), "wb");

	if (error == 0)
	{
		fwrite(outBuffer.data(), sizeof(unsigned char), outBuffer.size(), fileptr);
		fclose(fileptr);
	}
}

bool CreateFolder(std::string const& folderPathName)
{
	return CreateDirectoryA(folderPathName.c_str(), nullptr);
}
