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

#ifndef SURGSIM_FRAMEWORK_LOG_MESSAGE_H
#define SURGSIM_FRAMEWORK_LOG_MESSAGE_H

#include "SurgSim/Framework/LogMessageBase.h"

namespace SurgSim
{
namespace Framework
{

class Logger;

/// Specialization, handles flush on destruction
class LogMessage : public LogMessageBase
{
public:
	explicit LogMessage(Logger* logger, int level) : LogMessageBase(logger, level) {};
	explicit LogMessage(const std::unique_ptr<Logger>& logger, int level) : LogMessageBase(logger.get(), level) {};
	explicit LogMessage(const std::shared_ptr<Logger>& logger, int level) : LogMessageBase(logger.get(), level) {};
	~LogMessage()
	{
		flush();
	}
};


}; // namespace Framework
}; // namespace SurgSim

#endif  // SURGSIM_FRAMEWORK_LOG_MESSAGE_H
