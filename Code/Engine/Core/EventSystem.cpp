#include "Engine/Core/EventSystem.hpp"

#include "Engine/Core/NamedStrings.hpp"
#include "Engine/Core/DevConsole.hpp"
#include "Engine/Core/EngineCommon.hpp"

#include <algorithm>
#include <cctype>

extern DevConsole* g_theConsole;

EventSystem* g_theEventSystem = nullptr;

EventSystem::EventSystem(EventSystemConfig const& config)
	: m_config(config)
{
}

EventSystem::~EventSystem()
{
}

void EventSystem::StartUp()
{
}

void EventSystem::ShutDown()
{
}

void EventSystem::BeginFrame()
{
}

void EventSystem::EndFrame()
{
}

void EventSystem::SubscribeEventCallbackFunction(std::string eventName, EventCallbackFunction functionPtr)
{
	for (size_t index = 0; index < m_subscriptionByEventName[eventName].size(); index++)
	{
		EventCallbackFunction& funcionCallback = m_subscriptionByEventName[eventName][index];

		if (funcionCallback == nullptr)
		{
			funcionCallback = functionPtr;
			return;
		}
	}

	m_subscriptionByEventName[eventName].push_back(functionPtr);
}

void EventSystem::UnsubscribeEventCallbackFunction(std::string eventName, EventCallbackFunction functionPtr)
{
	for (size_t index = 0; index < m_subscriptionByEventName[eventName].size(); index++)
	{
		if (m_subscriptionByEventName[eventName][index] == functionPtr)
		{
			m_subscriptionByEventName[eventName][index] = nullptr;
		}
	}
}

bool EventSystem::FireEvent(std::string const& eventName, EventArgs& args)
{
	std::string uppercaseEventName = eventName;
	std::transform(uppercaseEventName.begin(), uppercaseEventName.end(), uppercaseEventName.begin(), [](unsigned char c) -> unsigned char { return (unsigned char)std::toupper(c); });

	auto found = m_subscriptionByEventName.find(uppercaseEventName);

	if (found != m_subscriptionByEventName.end())
	{
		for (size_t index = 0; index < m_subscriptionByEventName[uppercaseEventName].size(); index++)
		{
			EventCallbackFunction funcionCallback = found->second[index];
			
			bool callbackSuccess = funcionCallback(args);

			if(callbackSuccess)
				break;
		}

		return true;
	}

	return false;
}

bool EventSystem::FireEvent(std::string const& eventName)
{
	std::string uppercaseEventName = eventName;
	std::transform(uppercaseEventName.begin(), uppercaseEventName.end(), uppercaseEventName.begin(), [](unsigned char c) -> unsigned char { return (unsigned char)std::toupper(c); });

	auto found = m_subscriptionByEventName.find(uppercaseEventName);

	EventArgs args;

	if (found != m_subscriptionByEventName.end())
	{
		for (size_t index = 0; index < m_subscriptionByEventName[uppercaseEventName].size(); index++)
		{
			EventCallbackFunction funcionCallback = found->second[index];
			
			bool callbackSuccess = funcionCallback(args);

			if (callbackSuccess)
				break;
		}

		return true;
	}

	return false;
}

Strings EventSystem::GetAllCommands() const
{
	Strings commands;

	for (auto i : m_subscriptionByEventName)
	{
		commands.push_back(i.first);
	}

	return commands;
}

void SubscribeEventCallbackFunction(std::string eventName, EventCallbackFunction functionPtr)
{
	g_theEventSystem->SubscribeEventCallbackFunction(eventName, functionPtr);
}

void UnsubscribeEventCallbackFunction(std::string eventName, EventCallbackFunction functionPtr)
{
	g_theEventSystem->UnsubscribeEventCallbackFunction(eventName, functionPtr);
}

bool FireEvent(std::string const& eventName, EventArgs& args)
{
	return g_theEventSystem->FireEvent(eventName, args);
}

bool FireEvent(std::string const& eventName)
{
	return g_theEventSystem->FireEvent(eventName);
}
