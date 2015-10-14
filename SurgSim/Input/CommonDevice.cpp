// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Input/CommonDevice.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"

namespace SurgSim
{
namespace Input
{



CommonDevice::CommonDevice(const std::string& name) :
	m_name(name),
	m_nameForCallback(name),
	m_inputData(DataStructures::DataGroup())
{
}

CommonDevice::CommonDevice(const std::string& name, const DataStructures::DataGroup& inputData) :
	m_name(name),
	m_nameForCallback(name),
	m_inputData(inputData)
{
}

CommonDevice::CommonDevice(const std::string& name, DataStructures::DataGroup&& inputData) :
	m_name(name),
	m_nameForCallback(name),
	m_inputData(std::move(inputData))
{
}

CommonDevice::~CommonDevice()
{
	clearInputConsumers();
	clearOutputProducer();
}

std::string CommonDevice::getName() const
{
	return m_name;
}

std::string CommonDevice::getClassName() const
{
	SURGSIM_LOG_WARNING(Framework::Logger::getDefaultLogger())
		<< "getClassName() called on CommonDevice base class, this is wrong in almost all cases," <<
			" this means there is a class that does not have getClassName() defined.";
	return "SurgSim::Devices::CommonDevice";
}


void CommonDevice::setNameForCallback(const std::string& name)
{
	m_nameForCallback = name;
}

std::string CommonDevice::getNameForCallback() const
{
	return m_nameForCallback;
}

bool CommonDevice::addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	if (inputConsumer == nullptr)
	{
		return false;
	}

	boost::lock_guard<boost::mutex> lock(m_consumerProducerMutex);
	for (const auto& input : m_inputConsumerList)
	{
		if (input.lock() == inputConsumer)
		{
			return false;
		}
	}
	
	// NB: callbacks are called with the local m_nameForCallback.
	// This allows e.g. filters to call their callbacks with a name different from their "real" name.
	inputConsumer->initializeInput(m_nameForCallback, m_inputData);
	m_inputConsumerList.emplace_back(std::move(inputConsumer));
	return true;
}

bool CommonDevice::removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer)
{
	if (inputConsumer == nullptr)
	{
		return false;
	}

	boost::lock_guard<boost::mutex> lock(m_consumerProducerMutex);
	for (auto it = m_inputConsumerList.begin();  it != m_inputConsumerList.end();  ++it)
	{
		if (it->lock() == inputConsumer)
		{
			m_inputConsumerList.erase(it);
			// The iterator is now invalid.
			return true;
		}
	}
	return false;
}

void CommonDevice::clearInputConsumers()
{
	boost::lock_guard<boost::mutex> lock(m_consumerProducerMutex);
	m_inputConsumerList.clear();
}

bool CommonDevice::setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	if (outputProducer == nullptr)
	{
		return false;
	}

	boost::lock_guard<boost::mutex> lock(m_consumerProducerMutex);
	if (m_outputProducer.lock() == outputProducer)
	{
		return false;
	}
	m_outputProducer = std::move(outputProducer);
	return true;
}

bool CommonDevice::removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer)
{
	if (! outputProducer)
	{
		return false;
	}

	boost::lock_guard<boost::mutex> lock(m_consumerProducerMutex);
	if (m_outputProducer.lock() == outputProducer)
	{
		m_outputProducer.reset();
		return true;
	}
	return false;
}

void CommonDevice::clearOutputProducer()
{
	boost::lock_guard<boost::mutex> lock(m_consumerProducerMutex);
	m_outputProducer.reset();
}


bool CommonDevice::hasOutputProducer()
{
	return (m_outputProducer.lock() != nullptr);
}

void CommonDevice::pushInput()
{
	boost::lock_guard<boost::mutex> lock(m_consumerProducerMutex);
	for (auto it = m_inputConsumerList.begin();  it != m_inputConsumerList.end();  ++it)
	{
		// NB: callbacks are called with the local m_nameForCallback.
		// This allows e.g. filters to call their callbacks with a name different from their "real" name.
		auto inputConsumer = it->lock();
		if (inputConsumer != nullptr)
		{
			inputConsumer->handleInput(m_nameForCallback, m_inputData);
		}
	}
}

bool CommonDevice::pullOutput()
{
	boost::lock_guard<boost::mutex> lock(m_consumerProducerMutex);
	auto outputProducer = m_outputProducer.lock();
	if (outputProducer != nullptr)
	{
		// NB: callbacks are called with the local m_nameForCallback.
		// This allows e.g. filters to call their callbacks with a name different from their "real" name.
		bool gotOutput = outputProducer->requestOutput(m_nameForCallback, &m_outputData);
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

DataStructures::DataGroup& CommonDevice::getInputData()
{
	return m_inputData;
}

const DataStructures::DataGroup& CommonDevice::getOutputData() const
{
	return m_outputData;
}


};  // namespace Input
};  // namespace SurgSim
