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

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Framework/LogOutput.h"
#include "SurgSim/Framework/LoggerManager.h"

#include <iostream>

#include <boost/thread/locks.hpp>
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
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		m_globalThreshold = threshold;
	}
	setThreshold("", threshold);
}

void LoggerManager::setThreshold(const std::string& path, int threshold)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_thresholds.push_back(std::make_pair(path, threshold));
	for (auto it = m_loggers.cbegin(); it != m_loggers.cend(); ++it)
	{
		if (boost::istarts_with(it->first, path))
		{
			if (it->second !=  nullptr)
			{
				it->second->setThreshold(threshold);
			}
		}
	}
}

int LoggerManager::getThreshold() const
{
	return m_globalThreshold;
}


std::shared_ptr<Logger> LoggerManager::getLogger(const std::string& name)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);

	auto it = m_loggers.find(name);
	std::shared_ptr<Logger> result;

	if (it != m_loggers.end() && (it->second != nullptr))
	{
		result = it->second;
	}
	else
	{
		result = std::make_shared<Logger>(Logger(name, m_defaultOutput));
		result->setThreshold(m_globalThreshold);

		for (const auto& subgroupThreshold : m_thresholds)
		{
			if (boost::istarts_with(name, subgroupThreshold.first))
			{
				result->setThreshold(subgroupThreshold.second);
			}
		}
		m_loggers[name] = result;
	}
	return result;
}


std::shared_ptr<Logger> LoggerManager::getDefaultLogger()
{
	return getLogger("default");
}

}; // Framework
}; // SurgSim
