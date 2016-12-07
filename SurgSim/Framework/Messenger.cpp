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


#include "SurgSim/Framework/Messenger.h"

#include <boost/thread/lock_guard.hpp>

namespace SurgSim
{
namespace Framework
{

namespace
{

struct Contains
{
	explicit Contains(const std::shared_ptr<Component>& component) :
		receiver(component)
	{

	}

	bool operator()(const std::pair<std::weak_ptr<Component>, Messenger::EventCallback>& r)
	{

		auto candidate = r.first.lock();
		return candidate.get() == receiver.get();
	}

	const std::shared_ptr<Framework::Component>& receiver;
};

struct Expired
{
	bool operator()(const std::pair<std::weak_ptr<Component>, Messenger::EventCallback>& r)
	{
		return r.first.expired();
	}
};

}

Messenger::Messenger()
{
	m_timer.setMaxNumberOfFrames(1);
}

void Messenger::update()
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
		m_universalSubscribers.erase(std::remove_if(m_universalSubscribers.begin(), m_universalSubscribers.end(),
									 Expired()), m_universalSubscribers.end());
		broadcast = m_universalSubscribers;
	}

	// Should probably group events here
	for (const auto& event : events)
	{
		{
			boost::lock_guard<boost::mutex> lock(m_subscriberMutex);
			auto& temp = m_subscribers[event.name];
			temp.erase(std::remove_if(temp.begin(), temp.end(), Expired()), temp.end());
			subscribers = m_subscribers[event.name];
		}

		sendEvent(event, subscribers);
		sendEvent(event, broadcast);
	}
}

void Messenger::publish(const std::string& event, const std::string& sender, const boost::any& data)
{
	boost::lock_guard<boost::mutex> lock(m_eventMutex);
	m_events.emplace_back(event, sender, m_timer.getCurrentTime(), data);
}

void Messenger::publish(const std::string& event, const std::shared_ptr<Component>& sender, const boost::any& data)
{
	publish(event, sender->getFullName(), data);
}

void Messenger::subscribe(const std::string& event, const std::shared_ptr<SurgSim::Framework::Component>& subscriber,
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

void Messenger::subscribe(const std::shared_ptr<SurgSim::Framework::Component>& subscriber,
						  const EventCallback& callback)
{
	SURGSIM_ASSERT(subscriber != nullptr) << "Subscriber can't be nullptr.";
	SURGSIM_ASSERT(callback != nullptr) << "Callback can't be nullptr.";

	boost::lock_guard<boost::mutex> lock(m_subscriberMutex);
	auto entry = std::find_if(m_universalSubscribers.begin(), m_universalSubscribers.end(), Contains(subscriber));
	if (entry == m_universalSubscribers.end())
	{
		m_universalSubscribers.emplace_back(subscriber, callback);
	}
}

void Messenger::unsubscribe(const std::string& event,
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

void Messenger::unsubscribe(const std::shared_ptr<SurgSim::Framework::Component>& subscriber)
{
	SURGSIM_ASSERT(subscriber != nullptr) << "Subscriber can't be nullptr.";

	boost::lock_guard<boost::mutex> lock(m_subscriberMutex);
	m_universalSubscribers.erase(
		std::remove_if(m_universalSubscribers.begin(), m_universalSubscribers.end(), Contains(subscriber)),
		m_universalSubscribers.end()
	);

	for (auto& entry : m_subscribers)
	{
		auto& subscribers = entry.second;
		subscribers.erase(
			std::remove_if(subscribers.begin(), subscribers.end(), Contains(subscriber)), subscribers.end()
		);
	}
}

void Messenger::sendEvent(const Event& event, const std::vector<Subscriber>& subscribers)
{
	for (const auto& subscriber : subscribers)
	{
		auto shared = subscriber.first.lock();
		if (shared != nullptr)
		{
			subscriber.second(event);
		}
	}
}

}
}


