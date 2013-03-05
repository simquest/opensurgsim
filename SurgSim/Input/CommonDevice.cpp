// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include "CommonDevice.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>


namespace SurgSim
{
namespace Input
{

struct CommonDevice::State
{
	/// Constructor.
	State()
	{
	}

	/// A structure describing an entry in the listener list.
	struct ListenerEntry
	{
		/// Constructor.
		/// \param listenerArg The listener.
		/// \param forOutput true if this listener can provide output data, false if it can only listen to input.
		ListenerEntry(std::shared_ptr<DeviceListenerInterface>&& listenerArg, bool forOutput) :
			listener(std::move(listenerArg)),
			canProvideOutput(forOutput)
		{
		}

		std::shared_ptr<DeviceListenerInterface> listener;
		bool canProvideOutput;
	};

	/// The list of listeners and associated metadata.
	std::vector<ListenerEntry> listenerList;

	/// The mutex that protects the listener list.
	boost::mutex listenerListMutex;
};


CommonDevice::CommonDevice(const std::string& name, const SurgSim::DataStructures::DataGroup& inputData) :
	m_name(name), m_inputData(inputData), m_state(new State)
{
}


CommonDevice::CommonDevice(const std::string& name, SurgSim::DataStructures::DataGroup&& inputData) :
	m_name(name), m_inputData(std::move(inputData)), m_state(new State)
{
}

std::string CommonDevice::getName() const
{
	return m_name;
}

bool CommonDevice::addListener(std::shared_ptr<DeviceListenerInterface> listener)
{
	boost::lock_guard<boost::mutex> lock(m_state->listenerListMutex);
	for (auto it = m_state->listenerList.begin();  it != m_state->listenerList.end();  ++it)
	{
		if (it->listener == listener)
		{
			return false;
		}
	}
	m_state->listenerList.emplace_back(State::ListenerEntry(std::move(listener), true));
	return true;
}

bool CommonDevice::addInputListener(std::shared_ptr<DeviceListenerInterface> listener)
{
	boost::lock_guard<boost::mutex> lock(m_state->listenerListMutex);
	for (auto it = m_state->listenerList.begin();  it != m_state->listenerList.end();  ++it)
	{
		if (it->listener == listener)
		{
			return false;
		}
	}
	m_state->listenerList.emplace_back(State::ListenerEntry(std::move(listener), false));
	return true;
}

bool CommonDevice::removeListener(std::shared_ptr<DeviceListenerInterface> listener)
{
	boost::lock_guard<boost::mutex> lock(m_state->listenerListMutex);
	for (auto it = m_state->listenerList.begin();  it != m_state->listenerList.end();  ++it)
	{
		if (it->listener == listener)
		{
			m_state->listenerList.erase(it);
			// The iterator is now invalid.
			return true;
		}
	}
	return false;
}

void CommonDevice::pushInput()
{
	boost::lock_guard<boost::mutex> lock(m_state->listenerListMutex);
	for (auto it = m_state->listenerList.begin();  it != m_state->listenerList.end();  ++it)
	{
		it->listener->handleInput(m_name, m_inputData);
	}
}

bool CommonDevice::pullOutput()
{
	boost::lock_guard<boost::mutex> lock(m_state->listenerListMutex);
	for (auto it = m_state->listenerList.begin();  it != m_state->listenerList.end();  ++it)
	{
		if (it->canProvideOutput)
		{
			bool gotOutput = it->listener->requestOutput(m_name, &m_outputData);
			if (gotOutput)
			{
				return true;
			}

			// If we're here, then the listener has refused to provide output, even though it was registered via
			// \ref addListener and not \ref addInputListener. Keep going down the listener list in the hope
			// someone else can handle it.
		}
	}

	// If we haven't received an update, the old data is meaningless.
	m_outputData.resetAll();

	return false;
}

};  // namespace Input
};  // namespace SurgSim
