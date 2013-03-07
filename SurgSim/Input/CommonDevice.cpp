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

	/// The list of input consumers.
	std::vector<std::shared_ptr<InputConsumerInterface>> inputConsumerList;

	/// The output producer, if any.
	std::shared_ptr<OutputProducerInterface> outputProducer;

	/// The mutex that protects the consumers and the producer.
	boost::mutex consumerProducerMutex;
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

bool CommonDevice::addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	if (! inputConsumer)
	{
		return false;
	}

	boost::lock_guard<boost::mutex> lock(m_state->consumerProducerMutex);
	for (auto it = m_state->inputConsumerList.begin();  it != m_state->inputConsumerList.end();  ++it)
	{
		if (*it == inputConsumer)
		{
			return false;
		}
	}
	m_state->inputConsumerList.emplace_back(std::move(inputConsumer));
	return true;
}

bool CommonDevice::removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	if (! inputConsumer)
	{
		return false;
	}

	boost::lock_guard<boost::mutex> lock(m_state->consumerProducerMutex);
	for (auto it = m_state->inputConsumerList.begin();  it != m_state->inputConsumerList.end();  ++it)
	{
		if (*it == inputConsumer)
		{
			m_state->inputConsumerList.erase(it);
			// The iterator is now invalid.
			return true;
		}
	}
	return false;
}

bool CommonDevice::setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	if (! outputProducer)
	{
		return false;
	}

	boost::lock_guard<boost::mutex> lock(m_state->consumerProducerMutex);
	if (m_state->outputProducer == outputProducer)
	{
		return false;
	}
	m_state->outputProducer = std::move(outputProducer);
	return true;
}

bool CommonDevice::removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	if (! outputProducer)
	{
		return false;
	}

	boost::lock_guard<boost::mutex> lock(m_state->consumerProducerMutex);
	if (m_state->outputProducer == outputProducer)
	{
		m_state->outputProducer.reset();
		return true;
	}
	return false;
}

void CommonDevice::pushInput()
{
	boost::lock_guard<boost::mutex> lock(m_state->consumerProducerMutex);
	for (auto it = m_state->inputConsumerList.begin();  it != m_state->inputConsumerList.end();  ++it)
	{
		(*it)->handleInput(m_name, m_inputData);
	}
}

bool CommonDevice::pullOutput()
{
	boost::lock_guard<boost::mutex> lock(m_state->consumerProducerMutex);
	if (m_state->outputProducer)
	{
		bool gotOutput = m_state->outputProducer->requestOutput(m_name, &m_outputData);
		if (gotOutput)
		{
			return true;
		}

		// If we're here, then the producer has refused to provide output.
	}

	// If we haven't received an update, the old data is meaningless.
	m_outputData.resetAll();

	return false;
}


};  // namespace Input
};  // namespace SurgSim
