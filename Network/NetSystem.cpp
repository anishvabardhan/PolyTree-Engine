#include "Engine/Network/NetSystem.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Core/DevConsole.hpp"
#include "Engine/Core/NamedStrings.hpp"

#define NOMINMAX
#include <algorithm>

#define WIN32_LEAN_AND_MEAN	
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")


extern Net* g_theNet;
Net::Net(NetSystemConfig const& config)
	:m_config(config)
{
}

void Net::StartUp()
{
	m_sendBuffer = new char[m_config.m_sendBufferSize];
	m_recvBuffer = new char[m_config.m_recvBufferSize];

	if (IsClient())
	{
		WSADATA data;
		int result = WSAStartup(MAKEWORD(2, 2), &data);
		if (result != 0)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			WSACleanup();
			return;
		}

		// Create the client socket 
		m_clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (m_clientSocket == INVALID_SOCKET)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			WSACleanup();
			return;
		}

		// Set the socket to non-blocking mode
		unsigned long blockingMode = 1;
		result = ioctlsocket(m_clientSocket, FIONBIO, &blockingMode);
		if (result == SOCKET_ERROR)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_clientSocket);
			WSACleanup();
			return;
		}

		Strings hostInfo = SplitStringOnDelimiter(m_config.m_hostAddressString, ':');

		// Convert the host address
		IN_ADDR addr;
		result = inet_pton(AF_INET, hostInfo[0].c_str(), &addr);
		if (result == 0)
		{
			// Log error (Invalid IP address format)
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_clientSocket);
			WSACleanup();
			return;
		}
		else if (result == -1)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_clientSocket);
			WSACleanup();
			return;
		}

		// Save address and port

		m_hostAddress = ntohl(addr.S_un.S_addr);
		m_hostPort = (unsigned short)(atoi(hostInfo[1].c_str()));
	}
	else if (IsServer())
	{
		WSADATA data;
		int result = WSAStartup(MAKEWORD(2, 2), &data);
		if (result != 0)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			// Log error (e.g., "WSAStartup failed with error code: " + errorCode)
			return;
		}

		// Create the listen socket
		m_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (m_listenSocket == INVALID_SOCKET)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			// Log error (e.g., "Listen socket creation failed with error code: " + errorCode)
			WSACleanup();
			return;
		}

		// Set the socket to non-blocking mode
		unsigned long blockingMode = 1;
		result = ioctlsocket(m_listenSocket, FIONBIO, &blockingMode);
		if (result == SOCKET_ERROR)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			// Log error (e.g., "Failed to set non-blocking mode with error code: " + errorCode)
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_listenSocket);
			WSACleanup();
			return;
		}

		// Bind the listen socket to a port
		Strings hostInfo = SplitStringOnDelimiter(m_config.m_hostAddressString, ':');

		m_hostAddress = INADDR_ANY;
		m_hostPort = (unsigned short)(atoi(hostInfo[1].c_str()));

		sockaddr_in addr = {};
		addr.sin_family = AF_INET;
		addr.sin_addr.S_un.S_addr = htonl(m_hostAddress);
		addr.sin_port = htons(m_hostPort);

		result = bind(m_listenSocket, (sockaddr*)&addr, sizeof(addr));
		if (result == SOCKET_ERROR)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			// Log error (e.g., "Bind failed with error code: " + errorCode)
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_listenSocket);
			WSACleanup();
			return;
		}

		// Start listening for incoming connections
		result = listen(m_listenSocket, SOMAXCONN);
		if (result == SOCKET_ERROR)
		{
			[[maybe_unused]] int errorCode = WSAGetLastError();
			// Log error (e.g., "Listen failed with error code: " + errorCode)
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_listenSocket);
			WSACleanup();
			return;
		}
	}

	g_theEventSystem->SubscribeEventCallbackFunction("BURSTTEST", Net::BurstTest);
	g_theEventSystem->SubscribeEventCallbackFunction("REMOTECOMMAND", Net::RemoteCommand);
}

void Net::BeginFrame()
{
	if (IsClient())
	{
		sockaddr_in addr;
		addr.sin_family = AF_INET;
		addr.sin_addr.S_un.S_addr = htonl(m_hostAddress);
		addr.sin_port = htons(m_hostPort);

		switch (m_state)
		{
		case ConnectionState::DISCONNECTED:
		{
			// Attempt to connect if we haven't already.
			int result = connect(m_clientSocket, (sockaddr*)(&addr), (int)sizeof(addr));

			if (result == SOCKET_ERROR)
			{
				int errorCode = WSAGetLastError();
				
				if (errorCode == WSAEWOULDBLOCK || errorCode == WSAEALREADY)
				{
					m_state = ConnectionState::CONNECTING; // Proceed to CONNECTING state if it's in progress.
				}
				else if (errorCode == WSAENOTSOCK)
				{
					shutdown(m_clientSocket, SD_BOTH);
					closesocket(m_clientSocket);
					
					m_clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
					if (m_clientSocket == INVALID_SOCKET)
					{
						errorCode = WSAGetLastError();
						g_theConsole->AddLine(Rgba8::RED, Stringf("Failed to recreate Client Error code: %d", errorCode));
						WSACleanup();
						return;
					}

					// Set the socket to non-blocking mode
					unsigned long blockingMode = 1;
					result = ioctlsocket(m_clientSocket, FIONBIO, &blockingMode);
					if (result == SOCKET_ERROR)
					{
						errorCode = WSAGetLastError();
						g_theConsole->AddLine(Rgba8::RED, Stringf("Failed to set non-blocking mode Error code: %d", errorCode));
						shutdown(m_clientSocket, SD_BOTH);
						closesocket(m_clientSocket);
						WSACleanup();
						return;
					}

					m_state = ConnectionState::DISCONNECTED;
				}
				else
				{
					m_state = ConnectionState::ERROR_STATE; // Connection attempt failed with an unrecoverable error.
					return;
				}
			}
			else
			{
				g_theConsole->AddLine(Rgba8::BLUE, "Connection established immediately.");
				m_state = ConnectionState::CONNECTED;
				m_isConnectionEstablished = true;
			}
			break;
		}
		case ConnectionState::CONNECTING:
		{
			// Check the status of our connection attempt.
			fd_set writeSockets;
			fd_set exceptSockets;
			FD_ZERO(&writeSockets);
			FD_ZERO(&exceptSockets);
			FD_SET(m_clientSocket, &writeSockets);
			FD_SET(m_clientSocket, &exceptSockets);

			timeval waitTime = {};

			int selectResult = select(0, nullptr, &writeSockets, &exceptSockets, &waitTime);
			if (selectResult == SOCKET_ERROR)
			{
				int errorCode = WSAGetLastError();
				g_theConsole->AddLine(Rgba8::RED, Stringf("select() returned SOCKET_ERROR, error code: %d", errorCode));
				WSACleanup();
				m_state = ConnectionState::ERROR_STATE;
				return;
			}

			if (selectResult == 0)
			{
				return; // Do nothing and keep checking the socket status each frame
			}

			if (selectResult > 0)
			{
				g_theConsole->AddLine(Rgba8::YELLOW, Stringf("select() result: %d", selectResult));
				if (FD_ISSET(m_clientSocket, &exceptSockets))
				{
					g_theConsole->AddLine(Rgba8::RED, "Connection failed. Retrying connection");
					m_state = ConnectionState::DISCONNECTED;
				}
				if (FD_ISSET(m_clientSocket, &writeSockets))
				{
					// Connection successful
					g_theConsole->AddLine(Rgba8::BLUE, Stringf("Connection to client was established! %lld", m_clientSocket));
					m_state = ConnectionState::CONNECTED;
					m_isConnectionEstablished = true;
				}
			}
			break;
		}
		case ConnectionState::CONNECTED:
		{
			// Already connected so perform data transmission
			if (m_isConnectionEstablished)
			{
				if (!SendAndReceiveData())
				{
					m_state = ConnectionState::DISCONNECTED;
				}
			}
			break;
		}
		case ConnectionState::ERROR_STATE:
		{
			g_theConsole->AddLine(Rgba8::RED, "Error state reached. Resetting connection.");
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_clientSocket);
			m_clientSocket = INVALID_SOCKET;
			m_isConnectionEstablished = false;
			m_state = ConnectionState::DISCONNECTED;
			break;
		}
		}
	}
	else if (IsServer())
	{
		// Accept a new client connection if we don't already have one
		if (m_clientSocket == INVALID_SOCKET)
		{
			m_clientSocket = accept(m_listenSocket, nullptr, nullptr); // Attempt to accept a client connection
			if (m_clientSocket == INVALID_SOCKET)
			{
				[[maybe_unused]] int errorCode = WSAGetLastError();
				return; // No client is trying to connect, continue waiting
			}

			// Set the client socket to non-blocking mode
			unsigned long blockingMode = 1;
			int result = ioctlsocket(m_clientSocket, FIONBIO, &blockingMode);
			if (result == SOCKET_ERROR)
			{
				int errorCode = WSAGetLastError();
				g_theConsole->AddLine(Rgba8::RED, Stringf("Failed to set non-blocking mode with error code: " + errorCode));
				shutdown(m_clientSocket, SD_BOTH);
				closesocket(m_clientSocket);
				m_clientSocket = INVALID_SOCKET;
				return;
			}
		}

		// If we have a client connection, send and receive data
		if (m_clientSocket != INVALID_SOCKET)
		{
			m_isConnectionEstablished = true;
			
			if (!SendAndReceiveData())
			{
				m_isConnectionEstablished = false;
				m_state = ConnectionState::DISCONNECTED;
			}	
		}
	}
}

void Net::EndFrame()
{
}

void Net::ShutDown()
{
	if (m_sendBuffer != nullptr)
	{
		delete[] m_sendBuffer;
		m_sendBuffer = nullptr;
	}

	if (m_recvBuffer != nullptr)
	{
		delete[] m_recvBuffer;
		m_recvBuffer = nullptr;
	}

	if (IsClient())
	{
		if (m_clientSocket != INVALID_SOCKET)
		{
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_clientSocket);
			m_clientSocket = INVALID_SOCKET;
		}
		WSACleanup();
	}
	else if (IsServer())
	{
		if (m_listenSocket != INVALID_SOCKET)
		{
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_listenSocket);
			m_listenSocket = INVALID_SOCKET;
		}
		WSACleanup();
	}
}

bool Net::IsServer() const
{
	return m_config.m_modeString == "Server" || m_mode == Mode::SERVER;
}

bool Net::IsClient() const
{
	return m_config.m_modeString == "Client" || m_mode == Mode::CLIENT;
}

bool Net::RemoteCommand(EventArgs& args)
{
	if (!args.HasArgument("command"))
	{
		g_theConsole->AddLine(DevConsole::WARNING, "Error: Arguments are missing or incorrect");
	}
	else
	{
		std::string result = args.GetValue("command", "");
		TrimString(result, '"');
		result += "\0";
		g_theNet->m_sendQueue.push_back(result);
	}
	return true;
}

bool Net::BurstTest(EventArgs& args)
{
	UNUSED(args);

	int numMessages = 20;
	for (int i = 0; i < numMessages; i++)
	{
		std::string message = "RemoteCommand command=\"Echo Message=" + std::to_string(i) + "\"\0";
		g_theConsole->Execute(message, false);
	}
	return true;
}

void Net::ExecuteRecvMessage(std::string const& message)
{
	g_theConsole->Execute(message, false);
}

bool Net::SendAndReceiveData()
{
	// Send data if are connected 
	int recvResult;
	while (!m_sendQueue.empty())
	{
		const std::string& first = m_sendQueue.front();
		
		int msgLength = (int)first.length();
		int numOfBytesSent = 0;

		while (numOfBytesSent < msgLength)
		{
			int numOfBytesToSend = std::min(msgLength - numOfBytesSent, m_config.m_sendBufferSize - 1);

			memset(m_sendBuffer, 0, m_config.m_sendBufferSize);

			strncpy_s(m_sendBuffer, m_config.m_sendBufferSize, first.c_str() + numOfBytesSent, numOfBytesToSend);
			
			if (numOfBytesSent + numOfBytesToSend >= msgLength)
			{
				m_sendBuffer[numOfBytesToSend] = '\0';
				numOfBytesToSend++;
			}

			int result = send(m_clientSocket, m_sendBuffer, numOfBytesToSend, 0);
			
			if (result == SOCKET_ERROR)
			{
				int errorCode = WSAGetLastError();
				
				if (errorCode == WSAEWOULDBLOCK)
				{
					return true;
				}
				else
				{
					g_theConsole->AddLine(DevConsole::INFO_MAJOR, "Send Failed closing connection!");
					shutdown(m_clientSocket, SD_BOTH);
					closesocket(m_clientSocket);
					m_clientSocket = INVALID_SOCKET;
					m_isConnectionEstablished = false;
					return false;
				}
			}

			numOfBytesSent += result;
		}

		m_sendQueue.pop_front();
	}

	// Receive data
	recvResult = recv(m_clientSocket, m_recvBuffer, m_config.m_recvBufferSize, 0);
	if (recvResult > 0)
	{
		m_recvQueue.append(m_recvBuffer, recvResult);
		memset(m_recvBuffer, 0, m_config.m_recvBufferSize);

		size_t pos;

		while ((pos = m_recvQueue.find('\0')) != std::string::npos)
		{
			std::string completeMsg = m_recvQueue.substr(0, pos);

			ExecuteRecvMessage(completeMsg);

			m_recvQueue.erase(0, pos + 1);
		}
	}
	else if (recv == 0)
	{
		g_theConsole->AddLine(DevConsole::INFO_MAJOR, "Server has disconnected graefully!");
		shutdown(m_clientSocket, SD_BOTH);
		closesocket(m_clientSocket);
		m_clientSocket = INVALID_SOCKET;
		m_isConnectionEstablished = false;
		return false;
	}
	else
	{
		int errorCode = WSAGetLastError();

		if (errorCode != WSAEWOULDBLOCK)
		{
			g_theConsole->AddLine(DevConsole::INFO_MAJOR, Stringf("Received failed with error code: %d", errorCode));
			shutdown(m_clientSocket, SD_BOTH);
			closesocket(m_clientSocket);
			m_clientSocket = INVALID_SOCKET;
			m_isConnectionEstablished = false;
			return false;
		}
	}

	return true;
}

NetSystemConfig const& Net::GetConfig() const
{
	return m_config;
}