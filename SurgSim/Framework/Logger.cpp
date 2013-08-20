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

#include <iostream>


namespace SurgSim
{
namespace Framework
{
	Logger::Logger(const std::string& name, std::shared_ptr<LogOutput> output) :
		m_threshold(LOG_LEVEL_DEBUG), // include all logging levels
		m_name(name),
		m_output(output)
	{
	}

	std::shared_ptr<LoggerManager> Logger::getLoggerManager()
	{
		static std::shared_ptr<LoggerManager> loggerManager = std::make_shared<LoggerManager>();
		return loggerManager;
	}
}
}
