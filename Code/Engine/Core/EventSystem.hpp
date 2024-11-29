#pragma once

#include "Engine/Core/StringUtils.hpp"

#include <vector>
#include <map>
#include <string>

class NamedStrings;

typedef NamedStrings EventArgs;
typedef bool(*EventCallbackFunction)(EventArgs&);

struct EventSubscription
{
	EventCallbackFunction m_functionPtr = nullptr;
};

typedef std::vector<EventCallbackFunction> SubscriptionList;

struct EventSystemConfig
{

};

class EventSystem
{
protected:
	EventSystemConfig m_config;
	std::map<std::string, SubscriptionList> m_subscriptionByEventName;
public:
	EventSystem(EventSystemConfig const& config);
	~EventSystem();

	void StartUp();
	void ShutDown();
	void BeginFrame();
	void EndFrame();

	void SubscribeEventCallbackFunction(std::string eventName, EventCallbackFunction functionPtr);
	void UnsubscribeEventCallbackFunction(std::string eventName, EventCallbackFunction functionPtr);
	bool FireEvent(std::string const& eventName, EventArgs& args);
	bool FireEvent(std::string const& eventName);

	Strings GetAllCommands() const;
};

void SubscribeEventCallbackFunction(std::string eventName, EventCallbackFunction functionPtr);
void UnsubscribeEventCallbackFunction(std::string eventName, EventCallbackFunction functionPtr);
bool FireEvent(std::string const& eventName, EventArgs& args);
bool FireEvent(std::string const& eventName);