// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "SurgSim/Blocks/EventManager.h"

#include "boost/thread/lock_guard.hpp"
#include "boost/thread/mutex.hpp"

#include <future>

namespace
{

class Contains
{
public:
	Contains(const std::shared_ptr<SurgSim::Framework::Component>& component) :
		receiver(component)
	{

	}

	bool operator()(const std::pair<std::weak_ptr<SurgSim::Framework::Component>,
					SurgSim::Blocks::EventManager::EventCallback>& r)
	{
		if (!r.first.expired())
		{
			auto candidate = r.first.lock();
			return candidate.get() == receiver.get();
		}
		return false;
	}

private:
	const std::shared_ptr<SurgSim::Framework::Component>& receiver;
};

}

namespace SurgSim
{
namespace Blocks
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::EventManager, EventManager);

EventManager::EventManager(const std::string& name) : Behavior(name)
{
	m_timer.setMaxNumberOfFrames(1);
}

bool EventManager::doInitialize()
{
	return true;
}

bool EventManager::doWakeUp()
{
	return true;
}

void EventManager::update(double dt)
{
	std::vector<Subscriber> subscribers;
	std::vector<Subscriber> broadcast;
	std::vector<Event> events;
	{
		boost::lock_guard<boost::mutex> lock(m_eventMutex);
		std::swap(m_events, events);
	}
	{
		boost::lock_guard<boost::mutex> lock(m_subscriberMutex);
		broadcast = m_broadcast;
	}


	for (const auto& event : events)
	{
		{
			boost::lock_guard<boost::mutex> lock(m_subscriberMutex);
			subscribers = m_subscribers[event.name];
		}

		sendEvent(event, subscribers);
		sendEvent(event, broadcast);
	}
}

void EventManager::publish(const std::string& sender, const std::string& eventName, const boost::any& data)
{
	boost::lock_guard<boost::mutex> lock(m_eventMutex);
	m_events.emplace_back(m_timer.getCumulativeTime(), sender, eventName, data);
}

void EventManager::subscribe(const std::string& event, const std::shared_ptr<SurgSim::Framework::Component>& subscriber,
							 const EventCallback& callback)
{
	SURGSIM_ASSERT(subscriber != nullptr) << "Subscriber can't be nullptr.";
	SURGSIM_ASSERT(callback != nullptr) << "Callback can't be nullptr.";

	boost::lock_guard<boost::mutex> lock(m_subscriberMutex);

	const auto& receivers = m_subscribers[event];
	auto entry = std::find_if(receivers.begin(), receivers.end(), Contains(subscriber));
	if (entry == receivers.end())
	{
		m_subscribers[event].emplace_back(subscriber, callback);
	}

}

void EventManager::subscribe(const std::shared_ptr<SurgSim::Framework::Component>& subscriber,
							 const EventCallback& callback)
{
	SURGSIM_ASSERT(subscriber != nullptr) << "Subscriber can't be nullptr.";
	SURGSIM_ASSERT(callback != nullptr) << "Callback can't be nullptr.";

	boost::lock_guard<boost::mutex> lock(m_subscriberMutex);
	auto entry = std::find_if(m_broadcast.begin(), m_broadcast.end(), Contains(subscriber));
	if (entry == m_broadcast.end())
	{
		m_broadcast.emplace_back(subscriber, callback);
	}
}

void EventManager::unsubscribe(const std::string& event,
							   const std::shared_ptr<SurgSim::Framework::Component>& subscriber)
{
	SURGSIM_ASSERT(subscriber != nullptr) << "Subscriber can't be nullptr.";

	boost::lock_guard<boost::mutex> lock(m_subscriberMutex);

	auto entry = m_subscribers.find(event);
	if (entry != m_subscribers.end())
	{
		auto& subscribers = entry->second;
		subscribers.erase(
			std::remove_if(subscribers.begin(), subscribers.end(), Contains(subscriber)), subscribers.end()
		);
	}
}

void EventManager::unsubscribe(const std::shared_ptr<SurgSim::Framework::Component>& subscriber)
{
	SURGSIM_ASSERT(subscriber != nullptr) << "Subscriber can't be nullptr.";

	boost::lock_guard<boost::mutex> lock(m_subscriberMutex);
	m_broadcast.erase(
		std::remove_if(m_broadcast.begin(), m_broadcast.end(), Contains(subscriber)),
		m_broadcast.end()
	);

	for (auto& entry : m_subscribers)
	{
		auto& subscribers = entry.second;
		subscribers.erase(
			std::remove_if(subscribers.begin(), subscribers.end(), Contains(subscriber)), subscribers.end()
		);
	}
}

void EventManager::sendEvent(const Event& event, const std::vector<Subscriber>& subscribers)
{
	for (const auto& subscriber : subscribers)
	{
		if (!subscriber.first.expired())
		{
			subscriber.second(event);
		}
	}
}

}
}


