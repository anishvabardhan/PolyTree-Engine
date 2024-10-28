#pragma once
#include <string>
#include <vector>
#include <deque>
#include <cstdint>
#include "Engine/Core/EventSystem.hpp"
#include "Engine/Core/EngineCommon.hpp"

#ifdef ERROR
#undef ERROR
#endif // 

struct NetSystemConfig
{
	std::string m_modeString;
	std::string m_hostAddressString;
	int m_sendBufferSize = 2048;
	int m_recvBufferSize = 2048;
};

enum class ConnectionState
{
	DISCONNECTED,
	CONNECTING,
	CONNECTED,
	ERROR_STATE
};

enum class Mode
{
	NONE = 0,
	CLIENT,
	SERVER,
};

class Net
{
public:
	Net(NetSystemConfig const& config);

	void StartUp();
	void BeginFrame();
	void EndFrame();
	void ShutDown();

	bool IsServer() const;
	bool IsClient() const;
	static bool RemoteCommand(EventArgs& args);
	static bool BurstTest(EventArgs& args);
	void ExecuteRecvMessage(std::string const& message);

	bool SendAndReceiveData();

	NetSystemConfig const& GetConfig() const;

public:
	Mode m_mode = Mode::NONE;

protected:
	NetSystemConfig m_config;

	uintptr_t m_clientSocket = static_cast<uintptr_t>(-1); // Equivalent to INVALID_SOCKET
	uintptr_t m_listenSocket = static_cast<uintptr_t>(-1); // Equivalent to INVALID_SOCKET

	unsigned long m_hostAddress = 0;
	unsigned short m_hostPort = 0;

	char* m_recvBuffer = nullptr; // Store size of recv buffer from xml 
	char* m_sendBuffer = nullptr; // Store size of send buffer from xml

	std::deque<std::string> m_sendQueue; // Send buffers
	std::string m_recvQueue; // Receive buffers

	bool m_hasTriedToConnect = false;
	bool m_isConnectionEstablished = false;

private:
	ConnectionState m_state = ConnectionState::DISCONNECTED;
};