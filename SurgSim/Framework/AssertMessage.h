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

#ifndef SURGSIM_FRAMEWORK_ASSERT_MESSAGE_H
#define SURGSIM_FRAMEWORK_ASSERT_MESSAGE_H

#include <memory>

#include "SurgSim/Framework/LogMessageBase.h"


namespace SurgSim
{
namespace Framework
{


/// An exception class thrown by SURGSIM_ASSERT() failures and SURGSIM_FAILURE().
/// \ingroup assertAPI
class AssertionFailure : public std::runtime_error
{
public:
	/// Constructor
	/// \param message Exception message
	explicit AssertionFailure(const std::string& message) : std::runtime_error(message) {};
};


/// An internal message class used for assertion failures.  Dies after logging.
/// \ingroup assertInternals
class AssertMessage : public LogMessageBase
{
public:
	/// Constructor.
	/// \param logger %Logger used to log this message.
	explicit AssertMessage(Logger* logger) : LogMessageBase(logger, LOG_LEVEL_CRITICAL) {};

	/// Constructor.
	/// \param logger %Logger used to log this message.
	explicit AssertMessage(const std::unique_ptr<Logger>& logger) : LogMessageBase(logger.get(), LOG_LEVEL_CRITICAL) {};

	/// Constructor.
	/// \param logger %Logger used to log this message.
	explicit AssertMessage(const std::shared_ptr<Logger>& logger) : LogMessageBase(logger.get(), LOG_LEVEL_CRITICAL) {};


	/// Destructor
	~AssertMessage()
	{
		flush();
		throw AssertionFailure(getMessage());
	}
};


};  // namespace Framework
};  // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_ASSERT_MESSAGE_H
