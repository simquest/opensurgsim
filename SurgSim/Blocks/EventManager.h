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


#ifndef SURGSIM_BLOCKS_EVENTMANAGER_H
#define SURGSIM_BLOCKS_EVENTMANAGER_H


#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Timer.h"


namespace SurgSim
{
namespace Blocks
{
SURGSIM_STATIC_REGISTRATION(EventManager);

class EventManager : public SurgSim::Framework::Behavior
{
public:

	struct Event
	{
		Event(double time, const std::string& sender, const std::string& name, const boost::any& data) :
			time(time), sender(sender), name(name), data(data) {}

		double time;
		std::string sender;
		std::string name;
		boost::any data;
	};

	typedef std::function<void(const Event&)> EventCallback;

	EventManager(const std::string& name);

	SURGSIM_CLASSNAME(EventManager);

	virtual void update(double dt) override;
	virtual bool doInitialize() override;
	virtual bool doWakeUp() override;

	void publish(const std::string& sender, const std::string& event, const boost::any& data = boost::any());

	void subscribe(const std::string& event, const std::shared_ptr<SurgSim::Framework::Component>& subscriber,
				   const EventCallback& callback);

	void subscribe(const std::shared_ptr<SurgSim::Framework::Component>& subscriber,
				   const EventCallback& callback);

	void unsubscribe(const std::string& event, const std::shared_ptr<SurgSim::Framework::Component>& subscriber);
	void unsubscribe(const std::shared_ptr<SurgSim::Framework::Component>& subscriber);

private:

	SurgSim::Framework::Timer m_timer;

	typedef std::pair<std::weak_ptr<SurgSim::Framework::Component>, EventCallback> Subscriber;

	std::unordered_map<std::string, std::vector<Subscriber>> m_subscribers;
	std::vector<Subscriber> m_broadcast;

	boost::mutex m_subscriberMutex;
	boost::mutex m_eventMutex;

	std::vector<Event> m_events;

	void sendEvent(const Event& event, const std::vector<Subscriber>& receivers);

};
}
}

#endif