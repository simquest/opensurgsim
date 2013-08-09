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

#include <SurgSim/Framework/LoggerManager.h>

#include <iostream>

#include <boost/thread/lock_guard.hpp>
#include <boost/algorithm/string.hpp>

namespace SurgSim
{
namespace Framework
{

LoggerManager::LoggerManager():
	m_loggers(),
	m_defaultOutput(std::make_shared<StreamOutput>(std::cerr)),
	m_globalThreshold(LOG_LEVEL_WARNING),
	m_mutex()
{
	m_loggers["default"] = std::make_shared<Logger>("default", m_defaultOutput);
}

LoggerManager::~LoggerManager()
{
}


void LoggerManager::setDefaultOutput(std::shared_ptr<LogOutput> output)
{
	m_defaultOutput = output;
}

std::shared_ptr<LogOutput> LoggerManager::getDefaultOutput() const
{
	return m_defaultOutput;
}


void LoggerManager::setThreshold(int threshold)
{
	m_globalThreshold = threshold;
	boost::lock_guard<boost::mutex> lock(m_mutex);
	for (auto it = m_loggers.cbegin(); it != m_loggers.cend(); ++it)
	{
		it->second->setThreshold(threshold);
	}
}

void LoggerManager::setThreshold(const std::string& path, int threshold)
{
	m_globalThreshold = threshold;
	boost::lock_guard<boost::mutex> lock(m_mutex);
	for (auto it = m_loggers.cbegin(); it != m_loggers.cend(); ++it)
	{
		if (boost::istarts_with(it->first, path))
		{
			it->second->setThreshold(threshold);
		}
	}
}

int LoggerManager::getThreshold() const
{
	return m_globalThreshold;
}


void LoggerManager::setLogger(std::string name, std::shared_ptr<Logger> logger)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_loggers[name] = logger;
}

std::shared_ptr<Logger> LoggerManager::getLogger(const std::string& name)
{
	auto result = lookUpLogger(name);
	if ( result == nullptr )
	{
		result = createLogger(name, m_defaultOutput);
	}

	return result;
}


std::shared_ptr<Logger> LoggerManager::getDefaultLogger()
{
	return getLogger("default");
}


std::shared_ptr<Logger> LoggerManager::createConsoleLogger(const std::string& name)
{
	auto result = lookUpLogger(name);
	if ( result == nullptr )
	{
		static std::shared_ptr<StreamOutput> output(std::make_shared<StreamOutput>(std::cerr));
		result = createLogger(name, output);
	}

	return result;
}


std::shared_ptr<Logger> LoggerManager::lookUpLogger(const std::string& name)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	return m_loggers[name];
}

std::shared_ptr<Logger> LoggerManager::createLogger(const std::string& name, std::shared_ptr<LogOutput> output)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	std::shared_ptr<Logger> result(std::make_shared<Logger>(name, output));
	result->setThreshold(m_globalThreshold);
	m_loggers[name] = result;

	return result;
}



}; // Framework
}; // SurgSim
